// SP25 实车主链输出给串口层的云台命令。内部角度统一使用弧度。
#ifndef HFUT_AUTO_AIM_GIMBAL_COMMAND_HPP
#define HFUT_AUTO_AIM_GIMBAL_COMMAND_HPP

#include <cstdint>
#include <string>

namespace hfut {

enum class GimbalMode : int8_t {
  blind_camera_result = -2,
  no_valid_measurement = -1,
  unknown = 0,
  normal_measurement = 1,
};

struct GimbalCommand {
  double yaw = 0.0;
  double yaw_diff = 0.0;
  double yaw_vel = 0.0;
  double yaw_acc = 0.0;
  double pitch = 0.0;
  double pitch_diff = 0.0;
  double pitch_vel = 0.0;
  double pitch_acc = 0.0;
  double distance = 0.0;  // 米
  bool fire_advice = false;
  std::string target_id;
  GimbalMode mode = GimbalMode::unknown;
};

}  // namespace hfut

#endif  // HFUT_AUTO_AIM_GIMBAL_COMMAND_HPP
