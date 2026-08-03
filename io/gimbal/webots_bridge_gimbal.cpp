#include "webots_bridge_gimbal.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

#include "../bridge_protocol.hpp"

namespace hfut::io {

namespace {
std::string resolveBridgeDir(const std::string& bridge_dir) {
  if (!bridge_dir.empty()) return bridge_dir;
  const char* env = std::getenv("WEBOTS_ROS_FREE_BRIDGE_DIR");
  if (env != nullptr && env[0] != '\0') return env;
  return bridge::kDefaultBridgeDir;
}
}  // namespace

WebotsBridgeGimbal::WebotsBridgeGimbal(const std::string& bridge_dir) {
  std::string dir = resolveBridgeDir(bridge_dir);
  if (!dir.empty() && dir.back() == '/') dir.pop_back();
  command_path_ = dir + "/" + bridge::kCommandFile;
  tmp_path_ = command_path_ + ".tmp";
}

bool WebotsBridgeGimbal::send(const GimbalCommand& command, double sim_time_s) {
  bridge::CommandPacket pkt{};
  std::memcpy(pkt.magic, bridge::kCommandMagic, 8);
  pkt.version = bridge::kProtocolVersion;
  pkt.seq = ++seq_;
  pkt.sim_time_s = sim_time_s;

  pkt.yaw = command.yaw;
  pkt.yaw_diff = command.yaw_diff;
  pkt.yaw_vel = command.yaw_vel;
  pkt.yaw_acc = command.yaw_acc;
  pkt.pitch = command.pitch;
  pkt.pitch_diff = command.pitch_diff;
  pkt.pitch_vel = command.pitch_vel;
  pkt.pitch_acc = command.pitch_acc;
  pkt.distance = command.distance;
  pkt.fire_advice = command.fire_advice ? 1 : 0;
  pkt.mode = static_cast<int8_t>(command.mode);

  {
    std::ofstream out(tmp_path_, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out.write(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
    if (!out) return false;
  }
  return std::rename(tmp_path_.c_str(), command_path_.c_str()) == 0;
}

}  // namespace hfut::io
