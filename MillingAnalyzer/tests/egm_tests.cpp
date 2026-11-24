#include "egm3.pb.h"
#include "egm_session.h"
#include "pb_decode.h"
#include <cmath>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
using namespace milling::egm;
namespace {
int checks = 0;
void check(bool ok, const char *why) {
  ++checks;
  if (!ok)
    throw std::runtime_error(why);
}
const std::string header =
    "stage,operation,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,qy,qz,time_"
    "s,motion,collision,sweep_collision\n";
const std::string valid =
    header + "1,1,0,0,0,0,1,0,0,0,0,0,0,0\n1,1,1,1,0,0,1,0,0,0,1,4,0,0\n";
void reject(std::string text, const std::string &why) {
  try {
    preflight(text);
  } catch (const std::exception &e) {
    check(std::string(e.what()).find(why) != std::string::npos, e.what());
    return;
  }
  throw std::runtime_error("Accepted invalid CSV: " + why);
}
std::string row(const std::string &second) {
  return header + "1,1,0,0,0,0,1,0,0,0,0,0,0,0\n" + second + "\n";
}
} // namespace
int main() {
  try {
    auto plan = preflight(valid);
    check(plan.commands == 251 && plan.poses.size() == 2 &&
              plan.poses.back().motion == 4,
          "whole program/plunge preflight");
    auto middle = sample(plan.poses, .5);
    check((middle.p - V(.5, 0, 0)).norm() < 1e-12, "linear interpolation");
    auto first = encode(plan.poses.front(), 0, 0),
         same = encode(plan.poses.front(), 0, 0);
    check(first == same, "deterministic encoding");
    EgmSensor packet = EgmSensor_init_zero;
    auto input = pb_istream_from_buffer(first.data(), first.size());
    check(pb_decode(&input, EgmSensor_fields, &packet), "decode");
    auto &c = packet.planned.cartesian;
    check(c.pos.has_x && c.pos.has_y && c.pos.has_z && c.pos.x == 0 &&
              c.pos.y == 0 && c.pos.z == 0,
          "explicit zero XYZ");
    check(c.has_orient && !c.has_euler && c.orient.has_u0 && c.orient.has_u1 &&
              c.orient.has_u2 && c.orient.has_u3 && c.orient.u0 == 1 &&
              c.orient.u1 == 0 && c.orient.u2 == 0 && c.orient.u3 == 0,
          "quaternion order and presence");
    check(packet.header.has_seqno && packet.header.has_tm &&
              packet.header.seqno == 0 && packet.header.tm == 0,
          "zero seq/time presence");
    auto crlf = valid;
    for (size_t p = 0; (p = crlf.find('\n', p)) != std::string::npos; p += 2)
      crlf.insert(p, "\r");
    check(preflight(crlf).commands == 251, "CRLF supported");
    auto antipodal = preflight(row("1,1,1,1,0,0,-1,0,0,0,1,0,0,0"));
    check(sample(antipodal.poses, .5).q.angularDistance(Q::Identity()) < 1e-12,
          "sign-continuous SLERP");
    auto rotated = plan.poses;
    rotated.back().q = Q(Eigen::AngleAxisd(.3, V::UnitY()));
    check(std::abs(sample(rotated, .5).q.angularDistance(Q::Identity()) - .15) <
              1e-12,
          "orientation SLERP");
    reject("", "Empty");
    reject("\xef\xbb\xbf" + valid, "Expected");
    reject(valid + "\n", "Empty");
    reject(row("1,1,1,1,0,0,1,0,0,0,1,4,0,0,"), "incomplete");
    reject(row("1,1,1,nan,0,0,1,0,0,0,1,0,0,0"), "finite");
    reject(row("1,1,1,inf,0,0,1,0,0,0,1,0,0,0"), "finite");
    reject(row("1,1,1,1,0,0,2,0,0,0,1,0,0,0"), "quaternion");
    reject(row("1,1,1,1,0,0,1,0,0,0,0,0,0,0"), "Timeline");
    reject(row("1,1,1,1,0,0,1,0,0,0,241,0,0,0"), "Duration");
    reject(row("1,1,1,1,0,0,1,0,0,0,11,0,0,0"), "Timeline");
    reject(row("1,1,1,11,0,0,1,0,0,0,1,0,0,0"), "jump");
    reject(row("1,1,1,1,0,0,1,0,0,0,.001,0,0,0"), "Speed");
    reject(row("1,1,1,0,0,0,.9987502603949663,0,.0499791692706783,0,.01,0,0,0"),
           "Speed");
    reject(row("1,1,1,1,0,0,1,0,0,0,1,5,0,0"), "motion");
    reject(row("1,1,1,1,0,0,1,0,0,0,1,0,1,0"), "Collision");
    reject(row("1,1,1,1,0,0,1,0,0,0,1,0,0,1"), "Collision");
    reject(row("1,1,2,1,0,0,1,0,0,0,1,0,0,0"), "IDs");
    reject(row("3,1,1,1,0,0,1,0,0,0,1,0,0,0"), "stage");
    const std::string face =
        "face_id,pass_id,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,qy,qz,"
        "nx,ny,nz,time_s,collision,sweep_collision,motion\n";
    check(preflight(face + "8,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0\n8,0,1,1,0,0,1,0,"
                           "0,0,0,0,1,1,0,0,3\n")
                  .scope == 8,
          "whole-face contract");
    reject(face + "8,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0\n", "complete face");
    reject(face + "8,0,0,0,0,0,1,0,0,0,0,0,2,0,0,0,0\n", "normal");
    struct Comma : std::numpunct<char> {
      char do_decimal_point() const override { return ','; }
    };
    auto original = std::locale();
    std::locale::global(std::locale(original, new Comma));
    check(preflight(valid).commands == 251, "locale-independent decimals");
    std::locale::global(original);
    Session session;
    session.prepare(valid);
    check(!session.active(), "preparation opens no session");
    auto before = session.status();
    try {
      session.prepare("bad");
    } catch (const std::exception &) {
    }
    check(before == session.status(),
          "failed preflight preserves prepared job");
    session.invalidate();
    check(session.status().find("\"state\":\"idle\"") != std::string::npos,
          "invalidates old prepared path");
    try {
      session.csv(1);
      throw std::logic_error("stale accepted");
    } catch (const std::runtime_error &) {
    }
    std::cout
        << checks
        << " EGM preflight/serialization checks passed; no sockets opened\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
