#include "buff_aimer.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

#include <algorithm>
#include <stdexcept>

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  fire_gap_time_ = yaml["fire_gap_time"].as<double>();
  predict_time_ = yaml["predict_time"].as<double>();
  const auto motion_feedforward = yaml["motion_feedforward"];
  if (motion_feedforward) {
    motion_feedforward_enabled_ =
      motion_feedforward["enable"].as<bool>(motion_feedforward_enabled_);
    motion_difference_time_ =
      motion_feedforward["difference_time"].as<double>(motion_difference_time_);
    motion_max_yaw_velocity_rad_s_ =
      motion_feedforward["max_yaw_velocity"].as<double>(120.0) / 57.3;
    motion_max_pitch_velocity_rad_s_ =
      motion_feedforward["max_pitch_velocity"].as<double>(90.0) / 57.3;
    motion_max_yaw_acceleration_rad_s2_ =
      motion_feedforward["max_yaw_acceleration"].as<double>(motion_max_yaw_acceleration_rad_s2_);
    motion_max_pitch_acceleration_rad_s2_ =
      motion_feedforward["max_pitch_acceleration"].as<double>(motion_max_pitch_acceleration_rad_s2_);
  }
  if (!std::isfinite(motion_difference_time_) || motion_difference_time_ <= 0.0 ||
      !std::isfinite(motion_max_yaw_velocity_rad_s_) || motion_max_yaw_velocity_rad_s_ <= 0.0 ||
      !std::isfinite(motion_max_pitch_velocity_rad_s_) || motion_max_pitch_velocity_rad_s_ <= 0.0 ||
      !std::isfinite(motion_max_yaw_acceleration_rad_s2_) ||
      motion_max_yaw_acceleration_rad_s2_ <= 0.0 ||
      !std::isfinite(motion_max_pitch_acceleration_rad_s2_) ||
      motion_max_pitch_acceleration_rad_s2_ <= 0.0) {
    throw std::invalid_argument("打符运动前馈的差分间隔、速度和加速度上限必须是有限正数");
  }

  last_fire_t_ = std::chrono::steady_clock::now();
}

io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  io::Command command = {false, false, 0, 0};
  if (target.is_unsolve()) return command;

  // 如果子弹速度小于10，将其设为24
  if (bullet_speed < 10) bullet_speed = 24;

  auto now = std::chrono::steady_clock::now();

  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;

  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    command.yaw = yaw;
    command.pitch = -pitch;  //世界坐标系下的pitch向上为负
    command.horizon_distance = last_distance_;
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      command.control = true;
    } else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;
      mistake_count_++;
      command.control = false;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      command.control = true;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  }

  if (switch_fanblade_) {
    command.shoot = false;
    last_fire_t_ = now;
  } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    command.shoot = true;
    last_fire_t_ = now;
  }

  return command;
}

AimMotionCommand Aimer::aimWithMotion(
  auto_buff::Target & target, auto_buff::Target & past_target,
  auto_buff::Target & future_target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  AimMotionCommand result;
  if (target.is_unsolve()) return result;

  result.command = aim(target, timestamp, bullet_speed, to_now);
  if (!motion_feedforward_enabled_ || !result.command.control) return result;
  const double center_distance = last_distance_;
  const double center_angle = angle;

  if (bullet_speed < 10.0) bullet_speed = 24.0;
  const auto now = std::chrono::steady_clock::now();
  const double detect_now_gap = tools::delta_time(now, timestamp);
  const double future = to_now ? detect_now_gap + predict_time_ : 0.1 + predict_time_;
  const double dt = std::min(motion_difference_time_, future);
  if (!std::isfinite(dt) || dt <= 1e-4) return result;

  double past_yaw = 0.0;
  double past_pitch = 0.0;
  double future_yaw = 0.0;
  double future_pitch = 0.0;
  const bool past_valid =
    get_send_angle(past_target, future - dt, bullet_speed, to_now, past_yaw, past_pitch);
  last_distance_ = center_distance;
  angle = center_angle;
  const bool future_valid =
    get_send_angle(future_target, future + dt, bullet_speed, to_now, future_yaw, future_pitch);
  last_distance_ = center_distance;
  angle = center_angle;
  if (!past_valid || !future_valid) {
    return result;
  }

  const double past_command_pitch = -past_pitch;
  const double future_command_pitch = -future_pitch;
  const double yaw_before = tools::limit_rad(result.command.yaw - past_yaw);
  const double yaw_after = tools::limit_rad(future_yaw - result.command.yaw);
  const double pitch_before = result.command.pitch - past_command_pitch;
  const double pitch_after = future_command_pitch - result.command.pitch;

  result.yaw_velocity = tools::limit_rad(future_yaw - past_yaw) / (2.0 * dt);
  result.yaw_acceleration = (yaw_after - yaw_before) / (dt * dt);
  result.pitch_velocity = (future_command_pitch - past_command_pitch) / (2.0 * dt);
  result.pitch_acceleration = (pitch_after - pitch_before) / (dt * dt);
  result.yaw_velocity = std::clamp(
    result.yaw_velocity, -motion_max_yaw_velocity_rad_s_, motion_max_yaw_velocity_rad_s_);
  result.pitch_velocity = std::clamp(
    result.pitch_velocity, -motion_max_pitch_velocity_rad_s_, motion_max_pitch_velocity_rad_s_);
  result.yaw_acceleration = std::clamp(
    result.yaw_acceleration, -motion_max_yaw_acceleration_rad_s2_,
    motion_max_yaw_acceleration_rad_s2_);
  result.pitch_acceleration = std::clamp(
    result.pitch_acceleration, -motion_max_pitch_acceleration_rad_s2_,
    motion_max_pitch_acceleration_rad_s2_);
  result.motion_valid = std::isfinite(result.yaw_velocity) &&
                        std::isfinite(result.yaw_acceleration) &&
                        std::isfinite(result.pitch_velocity) &&
                        std::isfinite(result.pitch_acceleration);
  if (!result.motion_valid) {
    result.yaw_velocity = 0.0;
    result.yaw_acceleration = 0.0;
    result.pitch_velocity = 0.0;
    result.pitch_acceleration = 0.0;
  }
  return result;
}

bool Aimer::get_send_angle(
  auto_buff::Target & target, const double predict_time, const double bullet_speed,
  const bool to_now, double & yaw, double & pitch)
{
  // 考虑detecor所消耗的时间，此外假设aimer的用时可忽略不计
  // 如果 to_now 为 true，则根据当前时间和时间戳预测目标位置,deltatime = 现在时间减去当时照片时间，加上0.1
  target.predict(predict_time);
  // std::cout << "gap: " << detect_now_gap << std::endl;
  angle = target.ekf_x()[5];

  // 计算目标点的空间坐标
  auto aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  double d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  double h = aim_in_world[2];

  // 创建弹道对象
  tools::Trajectory trajectory0(bullet_speed, d, h);
  if (trajectory0.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    last_distance_ = 0;
    return false;
  }

  // 根据第一个弹道飞行时间预测目标位置
  target.predict(trajectory0.fly_time);
  angle = target.ekf_x()[5];

  // 计算新的目标点的空间坐标
  aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  h = aim_in_world[2];
  tools::Trajectory trajectory1(bullet_speed, d, h);
  if (trajectory1.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    last_distance_ = 0;
    return false;
  }

  // 计算时间误差
  auto time_error = trajectory1.fly_time - trajectory0.fly_time;
  if (std::abs(time_error) > 0.01) {  // 如果时间误差过大，返回未命中结果
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    return false;
  }

  // 计算偏航角和俯仰角，并返回命中结果
  yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
  pitch = trajectory1.pitch + pitch_offset_;
  last_distance_ = d;
  return true;
};

}  // namespace auto_buff
