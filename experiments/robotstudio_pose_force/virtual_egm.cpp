// Isolated virtual-controller adapter. It never binds to a network interface.
#include "egm3.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <Eigen/Geometry>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using Socket = SOCKET;
using SockLen = int;
constexpr Socket invalidSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using Socket = int;
using SockLen = socklen_t;
constexpr Socket invalidSocket = -1;
#endif
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
using V = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
using Clock = std::chrono::steady_clock;
void need(bool b, const char *why) {
  if (!b)
    throw std::runtime_error(why);
}
struct Pose {
  V p, n;
  Q q;
  double t;
  int motion;
};
double number(const std::string &s) {
  size_t n;
  double x = std::stod(s, &n);
  need(n == s.size() && std::isfinite(x), "Invalid number");
  return x;
}
std::vector<Pose> load(const std::filesystem::path &path) {
  need(std::filesystem::file_size(path) <= 64 * 1024 * 1024,
       "Pose file too large");
  std::ifstream f(path);
  std::string line;
  std::getline(f, line);
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
  need(line == "face_id,pass_id,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,"
               "qy,qz,nx,ny,nz,time_s,collision,sweep_collision,motion",
       "Use whole-face poses CSV, not FrezaEgm CSV");
  std::vector<Pose> out;
  double face = -1;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    std::stringstream stream(line);
    std::string word;
    std::vector<double> a;
    while (std::getline(stream, word, ','))
      a.push_back(number(word));
    need(a.size() == 17, "Invalid CSV row");
    if (face < 0)
      face = a[0];
    need(a[0] == face && a[1] == 0 && a[2] == double(out.size()),
         "One continuous face with ordered samples required");
    need(a[14] == 0 && a[15] == 0, "Collision flagged: virtual replay refused");
    need(a[16] >= 0 && a[16] <= 3 && a[16] == std::floor(a[16]),
         "Invalid motion type");
    Pose p{{a[3], a[4], a[5]},
           {a[10], a[11], a[12]},
           Q(a[6], a[7], a[8], a[9]),
           a[13],
           int(a[16])};
    need(p.p.cwiseAbs().maxCoeff() <= 100000 &&
             std::abs(p.q.norm() - 1) < 1e-6 && std::abs(p.n.norm() - 1) < 1e-6,
         "Invalid pose or normal");
    need(p.t >= 0 && p.t <= 240, "Virtual replay duration must be <=240 s");
    if (out.empty())
      need(p.t == 0, "First time must be zero");
    else {
      auto &prev = out.back();
      double dt = p.t - prev.t;
      need(dt > 0 && dt <= 10, "Non-monotonic or sparse timeline");
      need((p.p - prev.p).norm() <= 10 && prev.q.angularDistance(p.q) <= .53,
           "Pose jump too large");
      need((p.p - prev.p).norm() / dt <= 50.001 &&
               prev.q.angularDistance(p.q) / dt <= .524,
           "Virtual speed limits exceeded");
    }
    out.push_back(p);
    need(out.size() <= 200000, "Pose budget exceeded");
  }
  need(out.size() >= 2, "Empty path");
  return out;
}
struct Udp {
  Socket s = invalidSocket;
  explicit Udp(int port) {
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    need(s != invalidSocket, "socket failed");
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(port);
    if (bind(s, reinterpret_cast<sockaddr *>(&a), sizeof a) != 0) {
      close();
      throw std::runtime_error("Local port busy: close other virtual adapter");
    }
  }
  void close() {
    if (s == invalidSocket)
      return;
#ifdef _WIN32
    closesocket(s);
#else
    ::close(s);
#endif
    s = invalidSocket;
  }
  ~Udp() { close(); }
};
size_t encode(const Pose &p, uint32_t seq, uint32_t tm, unsigned char *buffer,
              size_t cap) {
  EgmSensor m = EgmSensor_init_zero;
  m.has_header = true;
  m.header.has_seqno = true;
  m.header.seqno = seq;
  m.header.has_tm = true;
  m.header.tm = tm;
  m.header.mtype = EgmHeader_MessageType_MSGTYPE_CORRECTION;
  m.has_planned = true;
  m.planned.has_cartesian = true;
  auto &c = m.planned.cartesian;
  c.has_pos = true;
  c.pos.has_x = c.pos.has_y = c.pos.has_z = true;
  c.pos.x = p.p.x();
  c.pos.y = p.p.y();
  c.pos.z = p.p.z();
  c.has_orient = true;
  c.orient.has_u0 = c.orient.has_u1 = c.orient.has_u2 = c.orient.has_u3 = true;
  c.orient.u0 = p.q.w();
  c.orient.u1 = p.q.x();
  c.orient.u2 = p.q.y();
  c.orient.u3 = p.q.z();
  auto stream = pb_ostream_from_buffer(buffer, cap);
  need(pb_encode(&stream, EgmSensor_fields, &m), "Encoding failed");
  return stream.bytes_written;
}
Pose sample(const std::vector<Pose> &p, double t) {
  auto it = std::upper_bound(p.begin(), p.end(), t,
                             [](double x, const Pose &v) { return x < v.t; });
  if (it == p.end())
    return p.back();
  if (it == p.begin())
    return p.front();
  auto &a = *(it - 1);
  auto &b = *it;
  double k = (t - a.t) / (b.t - a.t);
  Pose out = b;
  out.p = (1 - k) * a.p + k * b.p;
  out.q = a.q.slerp(k, b.q);
  out.n = ((1 - k) * a.n + k * b.n).normalized();
  out.t = t;
  return out;
}
int run(const std::vector<std::string> &args) {
  try {
    need(args.size() == 4 && args[1] == "--virtual-only" &&
             args[2] == "--anchor-first-to-feedback",
         "Usage: MillingVirtualEgm --virtual-only --anchor-first-to-feedback "
         "whole-face.poses.csv\nOnly localhost virtual controller. First pose "
         "explicitly anchored to first feedback; NOT measured part placement.");
    auto path = load(std::filesystem::u8path(args[3]));
#ifdef _WIN32
    WSADATA w{};
    need(WSAStartup(MAKEWORD(2, 2), &w) == 0, "Winsock init failed");
#endif
    Udp egm(6512), force(6513);
    std::cout << "Virtual adapter ready: EGM 127.0.0.1:6512, force "
                 "127.0.0.1:6513. No remote controller permitted.\n"
              << std::flush;
    bool anchored = false, haveForce = false, havePeer = false,
         doneSent = false;
    uint32_t seq = 0, robotSeq = 0;
    double forceSeq = -1, requested = 0, applied = 0;
    auto boot = Clock::now(), start = boot, lastFeedback = boot,
         lastForce = boot, lastSend = boot, doneAt = boot;
    // Planner +Z points from cutting face to holder. The virtual pencil
    // TCP +Z points from flange toward tip, so their frames differ by Rx(pi).
    const Q cutterToTcp(0, 1, 0, 0);
    Q rotation = Q::Identity();
    V translation = V::Zero();
    sockaddr_in peer{};
    while (true) {
      auto now = Clock::now();
      auto age = [&](auto time) {
        return std::chrono::duration<double>(now - time).count();
      };
      if (doneSent && age(doneAt) > .5)
        break;
      need(age(boot) < 300, "Virtual session deadline");
      need(anchored || age(boot) < 30,
           "No valid EGM + force feedback within 30 s");
      if (anchored) {
        need(doneSent || age(lastFeedback) < .5,
             "EGM feedback lost; adapter stopped");
        need(doneSent || age(lastForce) < .25,
             "Force task stale; adapter stopped");
      }
      fd_set readable;
      FD_ZERO(&readable);
      FD_SET(egm.s, &readable);
      FD_SET(force.s, &readable);
      timeval tv{0, 10000};
      int ready = select(int(std::max(egm.s, force.s) + 1), &readable, nullptr,
                         nullptr, &tv);
      need(ready >= 0, "select failed");
      if (FD_ISSET(force.s, &readable)) {
        char data[128];
        sockaddr_in source{};
        SockLen size = sizeof source;
        int n = recvfrom(force.s, data, sizeof(data) - 1, 0,
                         reinterpret_cast<sockaddr *>(&source), &size);
        if (n > 0 && source.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
          std::string text(data, n);
          auto comma = text.find(',');
          need(comma != std::string::npos, "Malformed force sample");
          double id = number(text.substr(0, comma)),
                 d = number(text.substr(comma + 1));
          need(id >= 0 && id == std::floor(id) && std::abs(d) <= .500001,
               "Invalid force sample");
          if (id > forceSeq) {
            // Receiver independently limits correction; RAPID scheduling is not
            // realtime.
            forceSeq = id;
            requested = d;
            lastForce = Clock::now();
            haveForce = true;
            bool finished = anchored && age(start) > path.back().t + 1.2;
            std::string reply =
                (finished ? "DONE," : "RUN,") + text.substr(0, comma);
            sendto(force.s, reply.c_str(), int(reply.size()), 0,
                   reinterpret_cast<sockaddr *>(&source), sizeof source);
            if (finished && !doneSent) {
              doneSent = true;
              doneAt = Clock::now();
            }
          }
        }
      }
      if (!FD_ISSET(egm.s, &readable))
        continue;
      unsigned char data[2048];
      sockaddr_in source{};
      SockLen size = sizeof source;
      int n = recvfrom(egm.s, reinterpret_cast<char *>(data), sizeof data, 0,
                       reinterpret_cast<sockaddr *>(&source), &size);
      if (n <= 0 || source.sin_addr.s_addr != htonl(INADDR_LOOPBACK))
        continue;
      if (havePeer && (source.sin_port != peer.sin_port ||
                       source.sin_addr.s_addr != peer.sin_addr.s_addr))
        continue;
      EgmRobot feedback = EgmRobot_init_zero;
      auto in = pb_istream_from_buffer(data, n);
      if (!pb_decode(&in, EgmRobot_fields, &feedback) || !feedback.has_header ||
          !feedback.header.has_seqno ||
          feedback.header.mtype != EgmHeader_MessageType_MSGTYPE_DATA ||
          !feedback.has_feedBack || !feedback.feedBack.has_cartesian)
        continue;
      if (havePeer && int32_t(feedback.header.seqno - robotSeq) <= 0)
        continue;
      auto &c = feedback.feedBack.cartesian;
      if (!c.has_pos || !c.pos.has_x || !c.pos.has_y || !c.pos.has_z ||
          !c.has_orient || !c.orient.has_u0 || !c.orient.has_u1 ||
          !c.orient.has_u2 || !c.orient.has_u3)
        continue;
      V p(c.pos.x, c.pos.y, c.pos.z);
      Q q(c.orient.u0, c.orient.u1, c.orient.u2, c.orient.u3);
      if (!p.allFinite() || !q.coeffs().allFinite() ||
          std::abs(q.norm() - 1) > .001)
        continue;
      robotSeq = feedback.header.seqno;
      peer = source;
      havePeer = true;
      lastFeedback = Clock::now();
      now = Clock::now();
      if (!haveForce || age(lastForce) >= .25)
        continue;
      if (!anchored) {
        rotation = q.normalized() * cutterToTcp.conjugate() *
                   path.front().q.conjugate();
        translation = p - rotation * path.front().p;
        start = now;
        lastSend = now;
        anchored = true;
        std::cout << "Explicit demo placement: translation="
                  << translation.transpose()
                  << ", quaternion(wxyz)=" << rotation.w() << ","
                  << rotation.x() << "," << rotation.y() << "," << rotation.z()
                  << "\n"
                  << std::flush;
      }
      double t = age(start), dt = std::clamp(age(lastSend), 0., .05);
      auto target = sample(path, t);
      double desired = target.motion == 0 ? requested : 0.;
      applied += std::clamp(desired - applied, -dt, dt);
      target.p = rotation * (target.p + applied * target.n) + translation;
      target.q = (rotation * target.q * cutterToTcp).normalized();
      auto bytes = encode(target, seq++, uint32_t(t * 1000), data, sizeof data);
      need(sendto(egm.s, reinterpret_cast<const char *>(data), int(bytes), 0,
                  reinterpret_cast<sockaddr *>(&peer),
                  sizeof peer) == int(bytes),
           "EGM send failed");
      lastSend = now;
    }
    std::cout
        << "Virtual sequence finished. Verify VC event log and final pose.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
#ifdef _WIN32
int wmain(int argc, wchar_t **argv) {
  std::vector<std::string> a;
  for (int i = 0; i < argc; ++i)
    a.push_back(std::filesystem::path(argv[i]).u8string());
  return run(a);
}
#else
int main(int argc, char **argv) { return run({argv, argv + argc}); }
#endif
