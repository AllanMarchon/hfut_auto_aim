// Writes gimbal_command.bin consumed by the Webots ros_free_camera_bridge
// controller. Writes to a .tmp sibling then rename(2)s over the target so the
// controller never reads a torn record.
#ifndef HFUT_AUTO_AIM_WEBOTS_BRIDGE_GIMBAL_HPP
#define HFUT_AUTO_AIM_WEBOTS_BRIDGE_GIMBAL_HPP

#include <cstdint>
#include <string>

#include "hfut_auto_aim/gimbal_command.hpp"

namespace hfut::io {

class WebotsBridgeGimbal {
 public:
  explicit WebotsBridgeGimbal(const std::string& bridge_dir = "");

  // Serializes `command` (angles already in radians) and atomically publishes
  // it. `sim_time_s` ties the command to the frame it was computed from.
  // Returns false on a filesystem error.
  bool send(const GimbalCommand& command, double sim_time_s);

 private:
  std::string command_path_;
  std::string tmp_path_;
  uint64_t seq_ = 0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_WEBOTS_BRIDGE_GIMBAL_HPP
