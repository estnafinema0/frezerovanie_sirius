// Pose/CSV contract and anchoring derived from the supplied Sirius-EGM-6510
// adapter. This module adds a UI-owned session, explicit IP filtering and
// motion=4 (axial plunge). The original adapter and legacy transport are
// intact.
#include "egm_session.h"
#include "egm3.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
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
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace milling::egm {
namespace {
using Clock = std::chrono::steady_clock;
void need(bool value, const std::string &why) {
  if (!value)
    throw std::runtime_error(why);
}
std::string quote(const std::string &s) {
  std::string out = "\"";
  for (unsigned char c : s) {
    if (c == '\\' || c == '"')
      out += '\\';
    if (c >= 32)
      out += char(c);
  }
  return out + '"';
}
double number(const std::string &s) {
  // Independent of the machine's regional decimal separator.
  std::istringstream in(s);
  in.imbue(std::locale::classic());
  double x;
  in >> std::noskipws >> x;
  need(in && in.peek() == EOF && std::isfinite(x),
       "Invalid finite decimal number");
  return x;
}
in_addr ip(const std::string &s, bool binding) {
  in_addr address{};
  need(inet_pton(AF_INET, s.c_str(), &address) == 1,
       "Enter an explicit IPv4 address");
  uint32_t value = ntohl(address.s_addr);
  need((binding && value == 0) || (value != 0 && value < 0xe0000000u),
       "Unicast IPv4 address required");
  return address;
}
struct Network {
  Network() {
#ifdef _WIN32
    WSADATA data{};
    need(WSAStartup(MAKEWORD(2, 2), &data) == 0,
         "Winsock initialization failed");
#endif
  }
  ~Network() {
#ifdef _WIN32
    WSACleanup();
#endif
  }
};
struct Udp {
  Socket s = invalidSocket;
  explicit Udp(const Settings &cfg) {
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    need(s != invalidSocket, "Cannot create UDP socket");
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr = ip(cfg.bindIp, true);
    a.sin_port = htons(static_cast<uint16_t>(cfg.port));
    if (bind(s, reinterpret_cast<sockaddr *>(&a), sizeof a) != 0) {
      close();
      throw std::runtime_error(
          "Cannot bind local IP/port: address unavailable or port busy");
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
void vectorJson(std::ostream &out, const V &p) {
  out << '[' << p.x() << ',' << p.y() << ',' << p.z() << ']';
}
} // namespace

std::vector<unsigned char> encode(const Pose &p, uint32_t seq, uint32_t tm) {
  EgmSensor packet = EgmSensor_init_zero;
  packet.has_header = true;
  packet.header.has_seqno = packet.header.has_tm = true;
  packet.header.seqno = seq;
  packet.header.tm = tm;
  packet.header.mtype = EgmHeader_MessageType_MSGTYPE_CORRECTION;
  packet.has_planned = packet.planned.has_cartesian = true;
  auto &c = packet.planned.cartesian;
  c.has_pos = c.has_orient = true;
  c.pos.has_x = c.pos.has_y = c.pos.has_z = true;
  c.pos.x = p.p.x();
  c.pos.y = p.p.y();
  c.pos.z = p.p.z();
  c.orient.has_u0 = c.orient.has_u1 = c.orient.has_u2 = c.orient.has_u3 = true;
  c.orient.u0 = p.q.w();
  c.orient.u1 = p.q.x();
  c.orient.u2 = p.q.y();
  c.orient.u3 = p.q.z();
  std::vector<unsigned char> bytes(512);
  auto stream = pb_ostream_from_buffer(bytes.data(), bytes.size());
  need(pb_encode(&stream, EgmSensor_fields, &packet),
       "Protobuf encoding failed");
  bytes.resize(stream.bytes_written);
  return bytes;
}
Pose sample(const std::vector<Pose> &poses, double t) {
  auto it =
      std::upper_bound(poses.begin(), poses.end(), t,
                       [](double time, const Pose &p) { return time < p.t; });
  if (it == poses.end())
    return poses.back();
  if (it == poses.begin())
    return poses.front();
  const auto &a = *(it - 1), &b = *it;
  const double k = (t - a.t) / (b.t - a.t);
  return {(1 - k) * a.p + k * b.p, a.q.slerp(k, b.q).normalized(), t, b.motion};
}
Plan preflight(const std::string &csv) {
  need(csv.size() <= 64 * 1024 * 1024, "CSV exceeds 64 MiB");
  Plan plan;
  plan.csv = csv;
  std::istringstream file(csv);
  std::string line;
  auto getLine = [&] {
    bool got = bool(std::getline(file, line));
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    return got;
  };
  need(getLine(), "Empty trajectory CSV");
  const bool program =
      line == "stage,operation,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,"
              "qy,qz,time_s,motion,collision,sweep_collision";
  need(program ||
           line == "face_id,pass_id,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,"
                   "qx,qy,qz,nx,ny,nz,time_s,collision,sweep_collision,motion",
       "Expected complete program or whole-face poses CSV, not legacy XYZ CSV");
  double face = -1;
  while (getLine()) {
    try {
      need(line.size() <= 8192, "CSV row exceeds 8 KiB");
      need(!line.empty() && line.back() != ',', "Empty or incomplete CSV row");
      std::istringstream row(line);
      std::string field;
      std::vector<double> a;
      while (std::getline(row, field, ',')) {
        need(a.size() < 17, "Too many CSV fields");
        a.push_back(number(field));
      }
      need(a.size() == (program ? 14u : 17u), "Incorrect CSV field count");
      if (face < 0)
        face = a[0];
      need(a[2] == double(plan.poses.size()),
           "Sample IDs must be consecutive from zero");
      if (program)
        need((a[0] == 1 || a[0] == 2) && a[1] >= 1 && a[1] <= 200000 &&
                 a[1] == std::floor(a[1]),
             "Invalid stage/operation");
      else {
        need(a[0] == face && face >= 1 && face <= 1000 &&
                 face == std::floor(face) && a[1] == 0,
             "Select one complete face, pass_id=0");
        need(std::abs(V(a[10], a[11], a[12]).norm() - 1) < 1e-6,
             "Invalid surface normal");
      }
      need(a[program ? 12 : 14] == 0 && a[program ? 13 : 15] == 0,
           "Collision flagged; replay refused");
      double motion = a[program ? 11 : 16];
      need(motion >= 0 && motion <= 4 && motion == std::floor(motion),
           "Unsupported motion type (expected 0..4)");
      Pose p{V(a[3], a[4], a[5]), Q(a[6], a[7], a[8], a[9]),
             a[program ? 10 : 13], int(motion)};
      need(p.p.cwiseAbs().maxCoeff() <= 100000 &&
               std::abs(p.q.norm() - 1) < 1e-6,
           "Invalid position or unit quaternion");
      p.q.normalize();
      need(p.t >= 0 && p.t <= 240, "Duration must be <=240 s; reduce the "
                                   "operation scope (no automatic splitting)");
      if (plan.poses.empty()) {
        need(p.t == 0, "First sample time must be zero");
        plan.low = plan.high = p.p;
      } else {
        const auto &previous = plan.poses.back();
        double dt = p.t - previous.t, distance = (p.p - previous.p).norm(),
               angle = previous.q.angularDistance(p.q);
        need(dt > 0 && dt <= 10, "Timeline must increase, with gaps <=10 s");
        need(distance <= 10 && angle <= .53,
             "Pose jump exceeds 10 mm or 0.53 rad");
        need(distance / dt <= 50.001 && angle / dt <= .524,
             "Speed exceeds 50 mm/s or 30 deg/s; reduce planner feed");
        if (previous.q.coeffs().dot(p.q.coeffs()) < 0)
          p.q.coeffs() *= -1;
        plan.low = plan.low.cwiseMin(p.p);
        plan.high = plan.high.cwiseMax(p.p);
      }
      plan.scope = program ? 0 : int(face);
      plan.poses.push_back(p);
      need(plan.poses.size() <= 200000, "Trajectory exceeds 200000 poses");
    } catch (const std::exception &e) {
      throw std::runtime_error("Sample " + std::to_string(plan.poses.size()) +
                               ": " + e.what());
    }
  }
  need(plan.poses.size() >= 2,
       "Calculate a continuous trajectory with at least two poses");
  // Test the actual encoder and interpolation before opening any network
  // socket. Collision flags come from the planner's full swept-path checks;
  // sampling is a serialization check, not a substitute for collision/robot
  // reachability.
  for (unsigned tick = 0;; ++tick) {
    double t = std::min(tick * .004, plan.poses.back().t);
    auto p = sample(plan.poses, t);
    auto bytes = encode(p, tick, uint32_t(t * 1000));
    auto stream = pb_istream_from_buffer(bytes.data(), bytes.size());
    EgmSensor decoded = EgmSensor_init_zero;
    need(pb_decode(&stream, EgmSensor_fields, &decoded),
         "Protobuf preflight decode failed");
    const auto &c = decoded.planned.cartesian;
    need(decoded.has_header && decoded.header.has_seqno &&
             decoded.header.seqno == tick && decoded.header.has_tm &&
             decoded.header.tm == uint32_t(t * 1000) && decoded.has_planned &&
             decoded.planned.has_cartesian && c.has_pos && c.pos.has_x &&
             c.pos.has_y && c.pos.has_z && c.has_orient && !c.has_euler &&
             c.orient.has_u0 && c.orient.has_u1 && c.orient.has_u2 &&
             c.orient.has_u3 &&
             (V(c.pos.x, c.pos.y, c.pos.z) - p.p).norm() == 0 &&
             (Q(c.orient.u0, c.orient.u1, c.orient.u2, c.orient.u3).coeffs() -
              p.q.coeffs())
                     .norm() == 0,
         "Protobuf preflight pose/presence mismatch");
    ++plan.commands;
    if (t == plan.poses.back().t)
      break;
  }
  return plan;
}
Session::~Session() {
  shutdown_ = true;
  if (worker_.joinable())
    worker_.join();
}
bool Session::active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}
void Session::prepare(const std::string &csv) {
  need(!active(),
       "EGM session active; stop it in RAPID before replacing the path");
  auto plan = std::make_shared<Plan>(preflight(csv));
  std::lock_guard<std::mutex> lock(mutex_);
  need(!active_, "EGM session active");
  plan_ = std::move(plan);
  ++id_;
  anchored_ = false;
  sent_ = 0;
  elapsed_ = 0;
  state_ = "prepared";
  message_ = "All poses and protobuf commands checked; no socket opened";
}
void Session::invalidate() {
  std::lock_guard<std::mutex> lock(mutex_);
  need(!active_, "EGM session active");
  plan_.reset();
  ++id_;
  anchored_ = false;
  state_ = "idle";
  message_ = "Trajectory changed; prepare it again";
}
std::string Session::csv(unsigned id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  need(plan_ && id == id_, "Prepared trajectory is stale; prepare it again");
  return plan_->csv;
}
void Session::start(unsigned id, const Settings &cfg) {
  need(cfg.virtualConfirmed && cfg.anchorConfirmed,
       "Confirm a virtual controller and demonstration anchoring to its first "
       "feedback");
  need(cfg.port >= 1024 && cfg.port <= 65535, "UDP port must be 1024..65535");
  // inet_pton is also available before WSAStartup on supported Windows
  // versions.
  ip(cfg.bindIp, true);
  ip(cfg.peerIp, false);
  std::shared_ptr<const Plan> plan;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    need(!active_, "EGM session already active");
    need(plan_ && id == id_, "Prepared trajectory is stale; prepare it again");
    plan = plan_;
  }
  if (worker_.joinable())
    worker_.join();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;
    anchored_ = false;
    state_ = "starting";
    message_ = "Opening UDP listener; controller must send the first packet";
    settings_ = cfg;
    sent_ = 0;
    elapsed_ = 0;
    stop_ = false;
  }
  try {
    worker_ = std::thread(&Session::run, this, plan, cfg);
  } catch (...) {
    finish("error", "Cannot start EGM worker");
    throw;
  }
}
void Session::stop() { stop_ = true; }
void Session::finish(const char *state, const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  state_ = state;
  message_ = message;
}
std::string Session::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(15);
  out << "{\"state\":" << quote(state_) << ",\"message\":" << quote(message_)
      << ",\"active\":" << (active_ ? "true" : "false") << ",\"job\":" << id_
      << ",\"sent\":" << sent_ << ",\"elapsed\":" << elapsed_
      << ",\"bindIp\":" << quote(settings_.bindIp)
      << ",\"peerIp\":" << quote(settings_.peerIp)
      << ",\"port\":" << settings_.port;
  if (plan_) {
    out << ",\"scope\":" << plan_->scope << ",\"poses\":" << plan_->poses.size()
        << ",\"commands\":" << plan_->commands
        << ",\"duration\":" << plan_->poses.back().t
        << ",\"runSeconds\":" << std::ceil(plan_->poses.back().t) + 3
        << ",\"partMin\":";
    vectorJson(out, plan_->low);
    out << ",\"partMax\":";
    vectorJson(out, plan_->high);
  }
  out << ",\"anchored\":" << (anchored_ ? "true" : "false");
  if (anchored_) {
    out << ",\"translation\":";
    vectorJson(out, translation_);
    out << ",\"rotation\":[" << rotation_.w() << ',' << rotation_.x() << ','
        << rotation_.y() << ',' << rotation_.z() << ']';
    out << ",\"baseMin\":";
    vectorJson(out, baseLow_);
    out << ",\"baseMax\":";
    vectorJson(out, baseHigh_);
  }
  return out.str() + '}';
}
void Session::run(std::shared_ptr<const Plan> plan, Settings cfg) {
  try {
    Network network;
    Udp udp(cfg);
    auto expectedIp = ip(cfg.peerIp, false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = "waiting";
      message_ = "Listening; start MillingPose6510 in the virtual T_ROB1 (30 s "
                 "timeout)";
    }
    const Q cutterToTcp(
        0, 1, 0, 0); // One Rx(pi): planner +Z to holder, demo TCP +Z to tip.
    Q rotation = Q::Identity();
    V translation = V::Zero();
    sockaddr_in peer{};
    bool anchored = false, settled = false, holding = false;
    Pose hold{V::Zero(), Q::Identity(), 0, 0};
    uint32_t robotSeq = 0, seq = 0;
    double settledSince = -1;
    auto boot = Clock::now(), start = boot, lastFeedback = boot;
    while (!shutdown_) {
      auto now = Clock::now();
      auto age = [&](auto point) {
        return std::chrono::duration<double>(now - point).count();
      };
      if (!anchored && stop_) {
        finish("stopped", "Waiting cancelled; no target sent");
        return;
      }
      need(age(boot) < 300,
           "Session exceeded 300 s; check RAPID stop and Event Log");
      need(
          anchored || age(boot) < 30,
          "No valid controller feedback within 30 s; check IP, port and RAPID");
      if (anchored && age(lastFeedback) >= .5) {
        if (holding) {
          finish("stopped",
                 "Hold finished: feedback ceased; verify RAPID stopped");
          return;
        }
        if (settled) {
          finish("completed", "Endpoint reached and feedback ceased; verify "
                              "RobotStudio Event Log");
          return;
        }
        throw std::runtime_error(
            "EGM feedback lost before endpoint; session failed");
      }
      fd_set readable;
      FD_ZERO(&readable);
      FD_SET(udp.s, &readable);
      timeval timeout{0, 10000};
      int ready = select(int(udp.s + 1), &readable, nullptr, nullptr, &timeout);
      need(ready >= 0, "UDP select failed");
      if (!ready)
        continue;
      unsigned char buffer[2048];
      sockaddr_in source{};
      SockLen size = sizeof source;
      int n = recvfrom(udp.s, reinterpret_cast<char *>(buffer), sizeof buffer,
                       0, reinterpret_cast<sockaddr *>(&source), &size);
      if (n <= 0 || source.sin_addr.s_addr != expectedIp.s_addr)
        continue;
      if (anchored && (source.sin_addr.s_addr != peer.sin_addr.s_addr ||
                       source.sin_port != peer.sin_port))
        continue;
      EgmRobot feedback = EgmRobot_init_zero;
      auto in = pb_istream_from_buffer(buffer, size_t(n));
      if (!pb_decode(&in, EgmRobot_fields, &feedback) || !feedback.has_header ||
          !feedback.header.has_seqno ||
          feedback.header.mtype != EgmHeader_MessageType_MSGTYPE_DATA ||
          !feedback.has_feedBack || !feedback.feedBack.has_cartesian)
        continue;
      if (anchored && int32_t(feedback.header.seqno - robotSeq) <= 0)
        continue;
      const auto &c = feedback.feedBack.cartesian;
      if (!c.has_pos || !c.pos.has_x || !c.pos.has_y || !c.pos.has_z ||
          !c.has_orient || !c.orient.has_u0 || !c.orient.has_u1 ||
          !c.orient.has_u2 || !c.orient.has_u3)
        continue;
      V p(c.pos.x, c.pos.y, c.pos.z);
      Q q(c.orient.u0, c.orient.u1, c.orient.u2, c.orient.u3);
      if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 100000 ||
          !q.coeffs().allFinite() || std::abs(q.norm() - 1) > .001)
        continue;
      q.normalize();
      need(!feedback.has_mciState || !feedback.mciState.has_state ||
               feedback.mciState.state != EgmMCIState_MCIStateType_MCI_ERROR,
           "Controller reported MCI_ERROR; check RobotStudio Event Log");
      now = Clock::now();
      // Do not skip a long section of the planned motion after a stalled peer.
      need(!anchored || holding || stop_ || age(lastFeedback) < .1,
           "Feedback gap exceeded 100 ms; replay interrupted");
      lastFeedback = now;
      peer = source;
      robotSeq = feedback.header.seqno;
      if (!anchored) {
        rotation =
            (q * cutterToTcp.conjugate() * plan->poses.front().q.conjugate())
                .normalized();
        translation = p - rotation * plan->poses.front().p;
        start = now;
        anchored = true;
        V low = V::Constant(std::numeric_limits<double>::infinity()),
          high = -low;
        // Conservative transformed bounds in constant time; all geometry work
        // and full CSV/serialization checks were finished before the socket.
        for (int k = 0; k < 8; ++k) {
          V corner;
          for (int a = 0; a < 3; ++a)
            corner[a] = (k & (1 << a)) ? plan->high[a] : plan->low[a];
          corner = rotation * corner + translation;
          low = low.cwiseMin(corner);
          high = high.cwiseMax(corner);
        }
        need(low.cwiseAbs().maxCoeff() <= 100000 &&
                 high.cwiseAbs().maxCoeff() <= 100000,
             "Transformed path bounds exceed coordinate limit");
        std::lock_guard<std::mutex> lock(mutex_);
        anchored_ = true;
        rotation_ = rotation;
        translation_ = translation;
        baseLow_ = low;
        baseHigh_ = high;
        state_ = "running";
        message_ =
            "Streaming XYZ + quaternion to the confirmed virtual controller";
      }
      double t = age(start);
      if (stop_ && !holding) {
        holding = true;
        hold = {p, q, t, 0};
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = "holding";
        message_ = "Holding measured TCP; stop EGM in RobotStudio. This is not "
                   "an emergency stop";
      }
      Pose target = holding ? hold : sample(plan->poses, t);
      if (!holding) {
        target.p = rotation * target.p + translation;
        target.q = (rotation * target.q * cutterToTcp).normalized();
        if (t >= plan->poses.back().t) {
          if ((target.p - p).norm() <= .1 &&
              target.q.angularDistance(q) <= .00872665) {
            if (settledSince < 0)
              settledSince = t;
            settled = t - settledSince >= .2;
          } else {
            settledSince = -1;
            settled = false;
          }
          need(
              t <= plan->poses.back().t + 12,
              "Timed EGM session did not end; check runSeconds and final pose");
        }
      }
      auto bytes = encode(target, seq++, uint32_t(t * 1000));
      need(sendto(udp.s, reinterpret_cast<const char *>(bytes.data()),
                  int(bytes.size()), 0, reinterpret_cast<sockaddr *>(&peer),
                  sizeof peer) == int(bytes.size()),
           "UDP target send failed");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        sent_ = seq;
        elapsed_ = t;
      }
    }
    finish("stopped", "Application closing; verify virtual controller stopped");
  } catch (const std::exception &e) {
    finish("error", e.what());
  }
}
} // namespace milling::egm
