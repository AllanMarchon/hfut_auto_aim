// ROS-free shim for rm_interfaces/msg/GimbalCmd. Field- and constant-compatible
// with the .msg. NOTE: the ported controller fills angles in DEGREES here (the
// ROS convention); the io boundary (Pipeline -> GimbalCommand) converts to the
// radians the bridge wire uses.
#ifndef HFUT_COMPAT_RM_INTERFACES_GIMBAL_CMD_HPP
#define HFUT_COMPAT_RM_INTERFACES_GIMBAL_CMD_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <std_msgs/msg/header.hpp>

namespace rm_interfaces {
namespace msg {

struct GimbalCmd {
  using SharedPtr = std::shared_ptr<GimbalCmd>;
  using ConstSharedPtr = std::shared_ptr<const GimbalCmd>;

  std_msgs::msg::Header header;
  double yaw = 0.0;
  double yaw_diff = 0.0;
  double yaw_v = 0.0;
  double yaw_a = 0.0;
  double pitch = 0.0;
  double pitch_diff = 0.0;
  double pitch_v = 0.0;
  double pitch_a = 0.0;

  // mode
  static constexpr int8_t MODE_BLIND_CAMERA_RESULT = -2;
  static constexpr int8_t MODE_NO_VALID_MEASUREMENT = -1;
  static constexpr int8_t MODE_UNKNOWN = 0;
  static constexpr int8_t MODE_NORMAL_MEASUREMENT = 1;

  double distance = 0.0;
  bool fire_advice = false;
  std::string target_id;
  int8_t mode = MODE_UNKNOWN;
};

}  // namespace msg
}  // namespace rm_interfaces

#endif  // HFUT_COMPAT_RM_INTERFACES_GIMBAL_CMD_HPP
