#ifndef HFUT_BUFF_GIMBAL_COMPAT_HPP
#define HFUT_BUFF_GIMBAL_COMPAT_HPP

#include <optional>

namespace io
{
// 供 buff 规划器使用的最小云台状态视图；真实 HFUT 串口反馈在
// standard.cpp 边界处转换后再传入。
struct GimbalState
{
  float yaw = 0.0F;
  float yaw_vel = 0.0F;
  float pitch = 0.0F;
  float pitch_vel = 0.0F;
  float bullet_speed = 0.0F;
  std::optional<double> fire_delay_s;  // 最新有效发弹延迟，单位秒；不参与姿态历史插值。
};
}  // namespace io

#endif  // HFUT_BUFF_GIMBAL_COMPAT_HPP
