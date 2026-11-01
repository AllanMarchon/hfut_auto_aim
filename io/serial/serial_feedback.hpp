#ifndef HFUT_AUTO_AIM_SERIAL_FEEDBACK_HPP
#define HFUT_AUTO_AIM_SERIAL_FEEDBACK_HPP

#include <cstdint>
#include <optional>

namespace hfut::io {

struct SerialFeedback {
  bool received = false;
  std::uint8_t mode = 0;
  double bullet_speed = 23.0;  // 电控反馈弹速，单位 m/s；无效时由主程序回退到配置值。
  double roll_rad = 0.0;
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  std::optional<double> fire_delay_s;  // 下位机必发的发弹延迟，毫秒转秒保存；无有效反馈时无值。
  // empty：新版反馈未提供底盘速度，不从串口包读取这些字段。
  double chassis_vx = 0.0;
  double chassis_vy = 0.0;
  double chassis_wz = 0.0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_SERIAL_FEEDBACK_HPP
