#include "climber_rune_aimer.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "../rune_predictor.hpp"
#include "tools/ballistic_solver.hpp"

namespace auto_buff_v2
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

// RunePredictor 负责前向预测；Climber 的 MPC 还会在估计时刻之前采样。
RuneEstimate rewind_to(const RuneEstimate & target, Timestamp time)
{
  auto state = target;
  const double dt = std::chrono::duration<double>(time - target.timestamp).count();
  if (state.sine_valid) {
    state.sine_phase += state.sine_omega * dt;
    state.sine_t += dt;
    if (std::abs(state.sine_omega) > 1e-12) {
      state.rotation_angle += state.sine_v * dt + state.sine_a / state.sine_omega *
        (std::cos(target.sine_phase) - std::cos(state.sine_phase));
    } else {
      state.rotation_angle += state.rotation_speed * dt;
    }
    state.rotation_speed = state.sine_v + state.sine_a * std::sin(state.sine_phase);
  } else {
    state.rotation_angle += state.rotation_speed * dt;
  }
  state.timestamp = time;
  return state;
}
}  // namespace

ClimberRuneAimer::Config ClimberRuneAimer::Config::load(const YAML::Node & node)
{
  Config result;
  if (node) {
    result.fire_gap_time = node["fire_gap_time"].as<double>(result.fire_gap_time);
    result.predict_time = node["predict_time"].as<double>(result.predict_time);
  }
  if (!std::isfinite(result.fire_gap_time) || result.fire_gap_time < 0 ||
      !std::isfinite(result.predict_time) || result.predict_time <= 0) {
    throw std::invalid_argument(
      "buff_v2.aimer.climber requires fire_gap_time >= 0 and predict_time > 0");
  }
  return result;
}

ClimberRuneAimer::ClimberRuneAimer(BuffConfig::Planner config, Config climber_config)
: config_(std::move(config)), climber_config_(climber_config),
  ballistic_solver_(tools::make_ballistic_solver(
    config_.ballistic.type,
    {config_.ballistic.njust_air_resistance, config_.ballistic.climber_air_resistance_k}))
{
}

std::optional<AimSolution> ClimberRuneAimer::aim_at(
  const RuneEstimate & target, Timestamp prediction_time, double speed) const
{
  if (speed < 10 || speed > 28) speed = 24;
  const auto state =
    prediction_time < target.timestamp ? rewind_to(target, prediction_time) : target;
  const auto point = RunePredictor{}.aimpoint_at(state, prediction_time);
  if (!point) return std::nullopt;
  const double distance = std::hypot(point->x(), point->y());
  const auto bullet = ballistic_solver_->solve(speed, distance, point->z());
  if (!bullet) return std::nullopt;
  return AimSolution{
    {std::atan2(point->y(), point->x()) + config_.yaw_offset,
     -bullet->pitch - config_.pitch_offset},
    bullet->fly_time, *point};
}

std::optional<RuneAim> ClimberRuneAimer::aim(
  std::uint64_t generation, const std::optional<RuneEstimate> & target, double speed,
  const io::GimbalState & gimbal, Timestamp now)
{
  if (!target) {
    generation_.reset();
    return std::nullopt;
  }
  if (generation_ != generation) {
    generation_ = generation;
    last_fire_ = now;
    last_angles_.setZero();
    mistake_count_ = 0;
    switch_fanblade_ = false;
    first_in_aimer_ = true;
  }
  if (speed < 10 || speed > 28) speed = 24;
  predicted_ = target->predicted;
  const auto future = offset_time(now, climber_config_.predict_time);
  const auto first = aim_at(*target, future, speed);
  if (!first) {
    first_in_aimer_ = true;
    return std::nullopt;
  }
  const auto prediction_time = offset_time(future, first->fly_time);
  const auto second = aim_at(*target, prediction_time, speed);
  if (!second || std::abs(second->fly_time - first->fly_time) > 0.03) {
    first_in_aimer_ = true;
    return std::nullopt;
  }

  auto_aim::Plan result;
  result.yaw = second->angles.x();
  result.pitch = second->angles.y();
  const bool changed = (second->angles - last_angles_).cwiseAbs().maxCoeff() > 5 / 57.3;
  if (mistake_count_ > 3) {
    switch_fanblade_ = true;
    mistake_count_ = 0;
    result.control = true;
    first_in_aimer_ = true;
  } else if (changed) {
    switch_fanblade_ = true;
    ++mistake_count_;
    result.control = true;
    first_in_aimer_ = true;
  } else {
    switch_fanblade_ = false;
    mistake_count_ = 0;
    result.control = true;
  }
  last_angles_ = second->angles;
  if (switch_fanblade_) last_fire_ = now;

  if (result.control) {
    if (first_in_aimer_) {
      first_in_aimer_ = false;
    } else {
      // Climber 在前向和反向求解之间恢复原始估计。
      const double dt = climber_config_.predict_time;
      const auto previous_time = offset_time(target->timestamp, -dt);
      const auto previous_first = aim_at(*target, previous_time, speed);
      if (!previous_first) {
        first_in_aimer_ = true;
        return std::nullopt;
      }
      const auto previous = aim_at(
        *target, offset_time(previous_time, previous_first->fly_time), speed);
      if (!previous || std::abs(previous->fly_time - previous_first->fly_time) > 0.03) {
        first_in_aimer_ = true;
        return std::nullopt;
      }
      result.yaw_vel =
        std::remainder(second->angles.x() - previous->angles.x(), 2 * kPi) / (2 * dt);
      result.yaw_acc = (std::remainder(second->angles.x() - gimbal.yaw, 2 * kPi) -
                        std::remainder(gimbal.yaw - previous->angles.x(), 2 * kPi)) / (dt * dt);
      result.pitch_vel =
        std::remainder(second->angles.y() - previous->angles.y(), 2 * kPi) / (2 * dt);
      result.pitch_acc = (second->angles.y() - 2 * gimbal.pitch + previous->angles.y()) / (dt * dt);
    }
  }
  return RuneAim{result, *second, prediction_time, speed};
}

bool ClimberRuneAimer::fire_advice(
  const Eigen::Vector3d &, const Eigen::Vector3d &, const auto_aim::Plan & plan,
  const io::GimbalState &, Timestamp now)
{
  if (!plan.control || !generation_ || switch_fanblade_) return false;
  if (std::chrono::duration<double>(now - last_fire_).count() <= climber_config_.fire_gap_time)
    return false;
  last_fire_ = now;
  return !predicted_;
}
}  // namespace auto_buff_v2
