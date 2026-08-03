#include "gestalt_idle_scanner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hfut::io {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

bool positiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool nonNegativeFinite(double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

GestaltIdleScanner::GestaltIdleScanner(GestaltIdleScanConfig config)
    : config_(config) {
  if (!positiveFinite(config_.yaw_rate_deg_s) ||
      !positiveFinite(config_.pitch_rate_deg_s) ||
      !positiveFinite(config_.pitch_limit_deg) || config_.pitch_limit_deg >= 90.0 ||
      !nonNegativeFinite(config_.activation_delay_s) ||
      !positiveFinite(config_.max_step_s)) {
    throw std::invalid_argument("invalid Gestalt idle scan configuration");
  }
}

void GestaltIdleScanner::reset() {
  scanning_ = false;
  pitch_direction_ = -1;
  target_yaw_rad_ = 0.0;
  target_pitch_rad_ = 0.0;
  inactive_since_.reset();
  last_step_ = Clock::time_point{};
}

void GestaltIdleScanner::writeCommand(
    double feedback_yaw_rad, double feedback_pitch_rad,
    double yaw_velocity_rad_s, double pitch_velocity_rad_s,
    GimbalCommand& command) const {
  command = GimbalCommand{};
  command.yaw = target_yaw_rad_;
  command.pitch = target_pitch_rad_;
  command.yaw_diff = std::remainder(target_yaw_rad_ - feedback_yaw_rad, 2.0 * kPi);
  command.pitch_diff = target_pitch_rad_ - feedback_pitch_rad;
  command.yaw_vel = yaw_velocity_rad_s;
  command.pitch_vel = pitch_velocity_rad_s;
  command.fire_advice = false;
  command.mode = GimbalMode::normal_measurement;
}

bool GestaltIdleScanner::update(
    bool auto_aim_active, bool visual_contact,
    double feedback_yaw_rad, double feedback_pitch_rad,
    Clock::time_point now, GimbalCommand& command) {
  if (!config_.enabled || auto_aim_active ||
      !std::isfinite(feedback_yaw_rad) || !std::isfinite(feedback_pitch_rad)) {
    reset();
    return false;
  }

  if (visual_contact) {
    target_yaw_rad_ = feedback_yaw_rad;
    target_pitch_rad_ = feedback_pitch_rad;
    scanning_ = false;
    inactive_since_.reset();
    last_step_ = now;
    writeCommand(feedback_yaw_rad, feedback_pitch_rad, 0.0, 0.0, command);
    return true;
  }

  if (!inactive_since_.has_value()) {
    inactive_since_ = now;
  }
  const double inactive_time_s =
      std::chrono::duration<double>(now - *inactive_since_).count();
  if (inactive_time_s < config_.activation_delay_s) {
    scanning_ = false;
    target_yaw_rad_ = feedback_yaw_rad;
    target_pitch_rad_ = feedback_pitch_rad;
    last_step_ = now;
    return false;
  }

  const double pitch_limit_rad = config_.pitch_limit_deg * kDegToRad;
  if (!scanning_) {
    target_yaw_rad_ = feedback_yaw_rad;
    target_pitch_rad_ = feedback_pitch_rad;
    pitch_direction_ = target_pitch_rad_ > pitch_limit_rad ? -1 :
        (target_pitch_rad_ < -pitch_limit_rad ? 1 : -1);
    last_step_ = now;
    scanning_ = true;
  }

  const double dt = std::clamp(
      std::chrono::duration<double>(now - last_step_).count(), 0.0,
      config_.max_step_s);
  last_step_ = now;

  const double yaw_rate_rad_s = config_.yaw_rate_deg_s * kDegToRad;
  const double pitch_rate_rad_s = config_.pitch_rate_deg_s * kDegToRad;
  target_yaw_rad_ = std::remainder(target_yaw_rad_ + yaw_rate_rad_s * dt, 2.0 * kPi);

  const bool pitch_was_outside = std::abs(target_pitch_rad_) > pitch_limit_rad;
  if (target_pitch_rad_ > pitch_limit_rad) {
    pitch_direction_ = -1;
  } else if (target_pitch_rad_ < -pitch_limit_rad) {
    pitch_direction_ = 1;
  }
  target_pitch_rad_ += pitch_direction_ * pitch_rate_rad_s * dt;

  if (!pitch_was_outside) {
    if (target_pitch_rad_ >= pitch_limit_rad) {
      target_pitch_rad_ = pitch_limit_rad;
      pitch_direction_ = -1;
    } else if (target_pitch_rad_ <= -pitch_limit_rad) {
      target_pitch_rad_ = -pitch_limit_rad;
      pitch_direction_ = 1;
    }
  }

  writeCommand(
      feedback_yaw_rad, feedback_pitch_rad, yaw_rate_rad_s,
      pitch_direction_ * pitch_rate_rad_s, command);
  return true;
}

}  // namespace hfut::io
