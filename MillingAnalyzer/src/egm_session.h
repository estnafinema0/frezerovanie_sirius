#pragma once
#include <Eigen/Geometry>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Separate RW6 virtual-controller pose adapter. No dependency on EgmClient,
// FrezaEgm interpolation, RAPID tooldata or the working XYZ transport.
namespace milling::egm {
using V = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
struct Pose {
  V p;
  Q q;
  double t;
  int motion;
};
struct Plan {
  std::vector<Pose> poses;
  std::string csv;
  V low, high;
  unsigned commands = 0;
  int scope = 0;
};
Plan preflight(
    const std::string &csv); // Entire CSV + 4 ms protobuf roundtrip; no socket.
Pose sample(const std::vector<Pose> &, double t);
std::vector<unsigned char> encode(const Pose &, uint32_t seq, uint32_t tm);
struct Settings {
  std::string bindIp = "127.0.0.1", peerIp = "127.0.0.1";
  int port = 6510;
  bool virtualConfirmed = false, anchorConfirmed = false;
};
class Session {
public:
  ~Session();
  void prepare(const std::string &csv);
  void invalidate();
  void start(unsigned id, const Settings &);
  void stop(); // Cancel waiting, or hold measured TCP until RAPID stops EGM.
  bool active() const;
  std::string status() const;
  std::string csv(unsigned id) const;

private:
  void run(std::shared_ptr<const Plan>, Settings);
  void finish(const char *state, const std::string &message);
  mutable std::mutex mutex_;
  std::shared_ptr<const Plan> plan_;
  unsigned id_ = 0;
  bool active_ = false, anchored_ = false;
  std::string state_ = "idle",
              message_ = "Prepare a calculated trajectory first";
  Settings settings_;
  unsigned sent_ = 0;
  double elapsed_ = 0;
  V translation_ = V::Zero(), baseLow_ = V::Zero(), baseHigh_ = V::Zero();
  Q rotation_ = Q::Identity();
  std::atomic<bool> stop_{false}, shutdown_{false};
  std::thread worker_;
};
} // namespace milling::egm
