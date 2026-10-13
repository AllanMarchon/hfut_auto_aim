#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

#include "buff_target.hpp"
#include "buff_type.hpp"
#include "io/command.hpp"

namespace auto_buff
{
// 打符角度及其导数；角度为弧度，速度为弧度每秒，加速度为弧度每秒平方。
struct AimMotionCommand
{
  io::Command command{};
  double yaw_velocity = 0.0;
  double yaw_acceleration = 0.0;
  double pitch_velocity = 0.0;
  double pitch_acceleration = 0.0;
  bool motion_valid = false;
};

// Aimer 的两个状态语义必须分开：control 表示瞄准角是否通过换叶片保护，
// shoot 表示本帧是否满足开火条件，最终对齐门控接受后再提交间隔。
struct FireDecision
{
  bool control = false;
  bool shoot = false;
};

struct FireAdviceProfile
{
  double max_angle_step_rad = 5.0 * 3.14159265358979323846 / 180.0;
  int stable_frames = 2;
  double min_interval_s = 0.7;
};

class Aimer
{
public:
  Aimer(const std::string & config_path);

  // 静止目标直接用当前 PnP 世界坐标解弹道，不预测符叶旋转，也不提供开火建议。
  io::Command aimPoint(const Eigen::Vector3d & aim_point_world, double bullet_speed);

  io::Command aim(
    Target & target, std::chrono::steady_clock::time_point & timestamp, double bullet_speed,
    bool to_now = true);

  AimMotionCommand aimWithMotion(
    Target & target, Target & past_target, Target & future_target,
    std::chrono::steady_clock::time_point & timestamp, double bullet_speed,
    bool to_now = true, bool enable_motion_feedforward = true);

  // MPC 模式只复用 Aimer 的换叶片/开火状态机，不再重复计算瞄准角和运动前馈。
  FireDecision fireAdvice(
    PowerRune_type type, double yaw, double pitch,
    std::chrono::steady_clock::time_point now);
  void commitFireAdvice(bool accepted, std::chrono::steady_clock::time_point now);
  void resetFireState();

  double last_distance() const { return last_distance_; }

  double angle;      ///
  double t_gap = 0;  ///

private:
  SmallTarget target_;
  double yaw_offset_;
  double pitch_offset_;

  FireAdviceProfile small_fire_profile_;
  FireAdviceProfile big_fire_profile_;
  double predict_time_;
  double motion_difference_time_ = 0.12;
  double motion_max_yaw_velocity_rad_s_ = 120.0 * 3.14159265358979323846 / 180.0;
  double motion_max_pitch_velocity_rad_s_ = 90.0 * 3.14159265358979323846 / 180.0;
  double motion_max_yaw_acceleration_rad_s2_ = 50.0;
  double motion_max_pitch_acceleration_rad_s2_ = 80.0;
  bool motion_feedforward_enabled_ = true;

  int mistake_count_ = 0;
  bool switch_fanblade_ = false;

  double last_yaw_ = 0;
  double last_pitch_ = 0;
  double last_distance_ = 0;

  std::chrono::steady_clock::time_point last_fire_t_;
  bool has_fired_ = false;
  PowerRune_type fire_mode_ = SMALL;
  bool has_fire_sample_ = false;
  int fire_stable_frames_ = 0;
  double fire_last_yaw_ = 0.0;
  double fire_last_pitch_ = 0.0;
  bool pending_fire_advice_ = false;

  bool updateControlState(double yaw, double pitch);
  bool evaluateFireAdvice(
    PowerRune_type type, double yaw, double pitch,
    std::chrono::steady_clock::time_point now);

  bool get_send_angle(
    auto_buff::Target & target, const double predict_time, const double bullet_speed,
    const bool to_now, double & yaw, double & pitch);
};
}  // namespace auto_buff
#endif  // AUTO_AIM__AIMER_HPP
