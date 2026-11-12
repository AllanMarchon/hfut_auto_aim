#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <string>

#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/armor_avoidance_optimizer.hpp"
#include "tasks/auto_aim/planner/anti_spin_policy.hpp"
#include "tasks/auto_aim/spin_speed_profile.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct AimReference
{
  Eigen::Vector2d yaw_pitch{Eigen::Vector2d::Zero()};
  bool angle_window_valid{false};
};

struct Plan
{
  bool control;
  bool fire;
  // 高速反小陀螺模式是否生效。该模式的 yaw/pitch 是守候角，不是正在运动的装甲板角度。
  bool anti_spin_active{false};
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float distance{-1.0F};
  bool target_spin_speed_valid{false};
  double target_spin_speed_rad_s{0.0};
  double fire_alignment_hold_s{0.0};
  // 高速反小陀螺可独立放宽最终反馈角门限；负值表示沿用普通门限。
  double fire_gate_yaw_tolerance_override_rad{-1.0};
  double fire_gate_pitch_tolerance_override_rad{-1.0};
};

class Planner
{
  public:
  Eigen::Vector4d debug_xyza = Eigen::Vector4d::Zero();
  Planner(const std::string & config_path, const std::string & planner_profile = "mpc_planner");

  // 使用外部提供的打符参考轨迹复用共享 yaw/pitch MPC。
  Plan plan(const Trajectory & trajectory, double yaw0, double distance);
  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed, double fire_delay_s = 0.0);
  void setArmorAvoidanceConfig(const ArmorAvoidanceConfig & config);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double coming_angle_rad_{55.0 / 57.3};
  double leaving_angle_rad_{20.0 / 57.3};
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  SpinSpeedProfile ordinary_spin_profile_;
  bool anti_spin_enabled_{true};
  double anti_spin_exit_hysteresis_rad_s_{1.0};
  double anti_spin_guard_stable_time_s_{0.04};
  double anti_spin_front_face_tolerance_rad_{12.0 / 57.3};
  double anti_spin_fire_yaw_tolerance_rad_{-1.0};
  double anti_spin_fire_pitch_tolerance_rad_{-1.0};
  AntiSpinModeSelector anti_spin_mode_selector_;
  std::string planner_profile_;
  ArmorAvoidanceConfig armor_avoidance_config_;
  ArmorAvoidanceResult armor_avoidance_result_;
  int aim_lock_id_{-1};
  bool trajectory_angle_window_valid_{true};

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;
  bool warned_solver_status_{false};

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  AimReference aim(const Target & target, double bullet_speed);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
  bool updateAntiSpinState(const Target & target);
  Plan planAntiSpin(Target target, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
