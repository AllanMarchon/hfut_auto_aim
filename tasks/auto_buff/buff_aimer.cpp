#include "buff_aimer.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  predict_time_ = yaml["predict_time"].as<double>();
  const double legacy_fire_gap = yaml["fire_gap_time"].as<double>(0.7);
  const auto fire_advice = yaml["fire_advice"];
  const auto load_fire_profile = [&](const char * key) {
    FireAdviceProfile profile;
    profile.min_interval_s = legacy_fire_gap;
    const auto node = fire_advice[key];
    if (node) {
      profile.max_angle_step_rad =
        node["max_angle_step_deg"].as<double>(5.0) * 3.14159265358979323846 / 180.0;
      profile.stable_frames = node["stable_frames"].as<int>(2);
      profile.min_interval_s = node["min_interval_s"].as<double>(legacy_fire_gap);
    }
    if (!std::isfinite(profile.max_angle_step_rad) || profile.max_angle_step_rad <= 0.0 ||
        profile.max_angle_step_rad > 3.14159265358979323846 || profile.stable_frames < 1 ||
        !std::isfinite(profile.min_interval_s) || profile.min_interval_s < 0.0) {
      throw std::invalid_argument(
        std::string("buff.fire_advice.") + key +
        " 参数无效：max_angle_step_deg 必须在 (0, 180]，stable_frames >= 1，min_interval_s >= 0");
    }
    return profile;
  };
  small_fire_profile_ = load_fire_profile("smallbuff");
  big_fire_profile_ = load_fire_profile("bigbuff");
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
}

io::Command Aimer::aimPoint(const Eigen::Vector3d & aim_point_world, double bullet_speed)
{
  io::Command command{false, false, 0.0, 0.0};
  if (!aim_point_world.allFinite()) return command;
  if (!std::isfinite(bullet_speed) || bullet_speed < 10.0) bullet_speed = 24.0;

  const double horizontal_distance = std::hypot(aim_point_world.x(), aim_point_world.y());
  if (!std::isfinite(horizontal_distance) || horizontal_distance <= 1e-6) return command;

  tools::Trajectory trajectory(bullet_speed, horizontal_distance, aim_point_world.z());
  if (trajectory.unsolvable || !std::isfinite(trajectory.pitch)) return command;

  command.yaw = std::atan2(aim_point_world.y(), aim_point_world.x()) + yaw_offset_;
  command.pitch = -(trajectory.pitch + pitch_offset_);
  command.horizon_distance = horizontal_distance;
  command.control = std::isfinite(command.yaw) && std::isfinite(command.pitch);
  if (!command.control) return io::Command{false, false, 0.0, 0.0};

  last_distance_ = horizontal_distance;
  return command;
}

io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  io::Command command = {false, false, 0, 0};
  if (target.is_unsolve()) {
    resetFireState();
    return command;
  }

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
    // control 是角度/换叶片保护状态，不能被开火间隔状态覆盖。
    command.control = updateControlState(yaw, pitch);
    const auto type = dynamic_cast<BigTarget *>(&target) != nullptr ? BIG : SMALL;
    const bool fire_ready = evaluateFireAdvice(type, yaw, pitch, now);
    command.shoot = command.control && fire_ready;
  } else {
    resetFireState();
  }

  return command;
}

FireDecision Aimer::fireAdvice(
  PowerRune_type type, double yaw, double pitch,
  std::chrono::steady_clock::time_point now)
{
  FireDecision result;
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    resetFireState();
    return result;
  }

  result.control = updateControlState(yaw, pitch);
  const bool fire_ready = evaluateFireAdvice(type, yaw, pitch, now);
  result.shoot = result.control && fire_ready;
  return result;
}

bool Aimer::updateControlState(double yaw, double pitch)
{
  if (mistake_count_ > 3) {
    switch_fanblade_ = true;
    mistake_count_ = 0;
    last_yaw_ = yaw;
    last_pitch_ = pitch;
    return true;
  } else if (std::abs(tools::limit_rad(last_yaw_ - yaw)) > 5 / 57.3 ||
             std::abs(last_pitch_ - pitch) > 5 / 57.3) {
    switch_fanblade_ = true;
    mistake_count_++;
    last_yaw_ = yaw;
    last_pitch_ = pitch;
    return false;
  } else {
    switch_fanblade_ = false;
    mistake_count_ = 0;
    last_yaw_ = yaw;
    last_pitch_ = pitch;
    return true;
  }
}

bool Aimer::evaluateFireAdvice(
  PowerRune_type type, double yaw, double pitch,
  std::chrono::steady_clock::time_point now)
{
  const auto & profile = type == BIG ? big_fire_profile_ : small_fire_profile_;
  if (!has_fire_sample_ || type != fire_mode_) {
    fire_mode_ = type;
    has_fire_sample_ = true;
    fire_stable_frames_ = 1;
    fire_last_yaw_ = yaw;
    fire_last_pitch_ = pitch;
    pending_fire_advice_ = false;
  } else {
    const double yaw_step = std::abs(tools::limit_rad(yaw - fire_last_yaw_));
    const double pitch_step = std::abs(pitch - fire_last_pitch_);
    if (yaw_step <= profile.max_angle_step_rad && pitch_step <= profile.max_angle_step_rad) {
      ++fire_stable_frames_;
    } else {
      fire_stable_frames_ = 1;
    }
    fire_last_yaw_ = yaw;
    fire_last_pitch_ = pitch;
  }

  const bool stable = fire_stable_frames_ >= profile.stable_frames;
  const bool interval_elapsed =
    !has_fired_ || tools::delta_time(now, last_fire_t_) >= profile.min_interval_s;
  pending_fire_advice_ = stable && interval_elapsed;
  return pending_fire_advice_;
}

void Aimer::commitFireAdvice(
  bool accepted, std::chrono::steady_clock::time_point now)
{
  if (pending_fire_advice_ && accepted) {
    last_fire_t_ = now;
    has_fired_ = true;
  }
  pending_fire_advice_ = false;
}

void Aimer::resetFireState()
{
  mistake_count_ = 0;
  switch_fanblade_ = true;
  has_fire_sample_ = false;
  fire_stable_frames_ = 0;
  pending_fire_advice_ = false;
}

AimMotionCommand Aimer::aimWithMotion(
  auto_buff::Target & target, auto_buff::Target & past_target,
  auto_buff::Target & future_target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now, bool enable_motion_feedforward)
{
  AimMotionCommand result;
  if (target.is_unsolve()) return result;

  result.command = aim(target, timestamp, bullet_speed, to_now);
  if (!motion_feedforward_enabled_ || !enable_motion_feedforward || !result.command.control)
    return result;
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
  const auto solve = [&](auto & prediction_target) {
    prediction_target.predict(predict_time);
    angle = prediction_target.ekf_x()[5];

    auto aim_in_world =
      prediction_target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
    double d = std::hypot(aim_in_world[0], aim_in_world[1]);
    double h = aim_in_world[2];
    tools::Trajectory trajectory0(bullet_speed, d, h);
    if (trajectory0.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
      last_distance_ = 0;
      return false;
    }

    prediction_target.predict(trajectory0.fly_time);
    angle = prediction_target.ekf_x()[5];
    aim_in_world = prediction_target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
    d = std::hypot(aim_in_world[0], aim_in_world[1]);
    h = aim_in_world[2];
    tools::Trajectory trajectory1(bullet_speed, d, h);
    if (trajectory1.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
      last_distance_ = 0;
      return false;
    }

    const double time_error = trajectory1.fly_time - trajectory0.fly_time;
    if (std::abs(time_error) > 0.01) {
      tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
      return false;
    }

    yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
    pitch = trajectory1.pitch + pitch_offset_;
    last_distance_ = d;
    return true;
  };

  // Legacy Aimer takes a target by reference; keep its BigTarget forecast off the live tracker.
  if (auto * big_target = dynamic_cast<auto_buff::BigTarget *>(&target)) {
    auto prediction_target = *big_target;
    return solve(prediction_target);
  }
  return solve(target);
};

}  // namespace auto_buff
