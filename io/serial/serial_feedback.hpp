#ifndef HFUT_AUTO_AIM_SERIAL_FEEDBACK_HPP
#define HFUT_AUTO_AIM_SERIAL_FEEDBACK_HPP

#include <cstdint>

namespace hfut::io {

struct SerialFeedback {
  bool received = false;
  std::uint8_t mode = 0;
  double bullet_speed = 23.0;
  double roll_rad = 0.0;
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double chassis_vx = 0.0;
  double chassis_vy = 0.0;
  double chassis_wz = 0.0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_SERIAL_FEEDBACK_HPP
