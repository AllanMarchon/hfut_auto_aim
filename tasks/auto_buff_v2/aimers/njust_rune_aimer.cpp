#include "njust_rune_aimer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "../rune_predictor.hpp"
#include "tools/ballistic_solver.hpp"

namespace auto_buff_v2
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

NjustRuneAimer::Config NjustRuneAimer::Config::load(const YAML::Node & node)
{
  Config result;
  if (node) {
    result.shoot_delay = node["shoot_delay"].as<double>(result.shoot_delay);
    result.rune_idle_duration = node["rune_idle_duration"].as<double>(result.rune_idle_duration);
    result.rune_shoot_duration = node["rune_shoot_duration"].as<double>(result.rune_shoot_duration);
    result.yaw_tolerance = node["yaw_tolerance"].as<double>(result.yaw_tolerance);
    result.pitch_tolerance = node["pitch_tolerance"].as<double>(result.pitch_tolerance);
  }
  return result;
}

NjustRuneAimer::NjustRuneAimer(BuffConfig::Planner config, Config njust_config)
: config_(std::move(config)),
  njust_config_(njust_config),
  ballistic_solver_(tools::make_ballistic_solver(
    config_.ballistic.type,
    {config_.ballistic.njust_air_resistance, config_.ballistic.climber_air_resistance_k}))
{
}

std::optional<RuneAim> NjustRuneAimer::aim(
  std::uint64_t generation, const std::optional<RuneEstimate> & target, double speed,
  const io::GimbalState &, Timestamp now)
{
  auto_aim::Plan result;
  if (!target) {
    attack_start_.reset();
    attack_generation_.reset();
    return std::nullopt;
  }
  if (attack_generation_ != generation) {
    attack_start_ = now;
    attack_generation_ = generation;
  }
  if (speed < config_.bullet_speed_min || speed > config_.bullet_speed_max)
    speed = config_.bullet_speed_default;
  const double distance = target->center.norm();
  double fly_time = distance / speed;
  AimSolution solution;
  // RMCS 开火控制：每次都基于原始估计重新迭代，最多求解五次。
  Timestamp prediction_time;
  for (int i = 0; i < 5; ++i) {
    prediction_time = offset_time(now, njust_config_.shoot_delay + fly_time);
    const auto next = aim_at(*target, prediction_time, speed);
    if (!next) return std::nullopt;
    solution = *next;
    const double previous = fly_time;
    fly_time = next->fly_time;
    if (std::abs(fly_time - previous) < 0.001) break;
  }
  const auto before = aim_at(*target, offset_time(prediction_time, -0.01), speed);
  const auto after = aim_at(*target, offset_time(prediction_time, 0.01), speed);
  if (!before || !after) return std::nullopt;
  result.control = true;
  result.yaw = solution.angles.x();
  result.pitch = solution.angles.y();
  result.yaw_vel = std::remainder(after->angles.x() - before->angles.x(), 2 * kPi) / 0.02;
  result.pitch_vel = (after->angles.y() - before->angles.y()) / 0.02;
  result.yaw_acc = (std::remainder(after->angles.x() - solution.angles.x(), 2 * kPi) -
                   std::remainder(solution.angles.x() - before->angles.x(), 2 * kPi)) / 0.0001;
  result.pitch_acc = (after->angles.y() - 2 * solution.angles.y() + before->angles.y()) / 0.0001;
  return RuneAim{result, solution, prediction_time, speed};
}

std::optional<AimSolution> NjustRuneAimer::aim_at(
  const RuneEstimate & state, Timestamp prediction_time, double speed) const
{
  const auto point = RunePredictor{}.aimpoint_at(state, std::max(prediction_time, state.timestamp));
  if (!point) return std::nullopt;
  const double distance = std::hypot(point->x(), point->y());
  const auto bullet = ballistic_solver_->solve(speed, distance, point->z());
  if (!bullet) return std::nullopt;
  return AimSolution{
    {std::remainder(std::atan2(point->y(), point->x()) + config_.yaw_offset, 2 * kPi),
     -bullet->pitch - config_.pitch_offset},
    bullet->fly_time, *point};
}

bool NjustRuneAimer::fire_advice(
  const Eigen::Vector3d & rune_center, const Eigen::Vector3d & aimpoint,
  const auto_aim::Plan & plan, const io::GimbalState & gimbal, Timestamp now)
{
  if (!plan.control || !attack_start_) return false;
  const double cycle = njust_config_.rune_idle_duration + njust_config_.rune_shoot_duration;
  const double phase =
    std::fmod(std::chrono::duration<double>(now - *attack_start_).count(), cycle);
  const bool shoot_phase = phase >= njust_config_.rune_idle_duration;
  const double yaw_error = std::remainder(gimbal.yaw - plan.yaw, 2 * kPi);
  const double pitch_error = gimbal.pitch - plan.pitch;
  const double xy = std::hypot(aimpoint.x(), aimpoint.y());
  if (xy < 0.1) return false;
  const double ratio = std::min(xy, 5.0) / xy;
  const Eigen::Vector3d scaled = aimpoint * ratio;
  const double scaled_xy = std::hypot(scaled.x(), scaled.y());
  const double center_yaw = std::atan2(rune_center.y(), rune_center.x());
  const double target_yaw = std::atan2(aimpoint.y(), aimpoint.x());
  const double blade_scale =
    std::max(0.0, std::cos(std::remainder(target_yaw - center_yaw, 2 * kPi)));
  const double yaw_upper = std::atan2(njust_config_.yaw_tolerance * blade_scale, scaled_xy);
  const double yaw_lower = -yaw_upper;
  const double pitch_center = std::atan2(scaled.z(), scaled_xy);
  const double pitch_lower =
    std::atan2(scaled.z() - njust_config_.pitch_tolerance * blade_scale, scaled_xy) - pitch_center;
  const double pitch_upper =
    std::atan2(scaled.z() + njust_config_.pitch_tolerance * blade_scale, scaled_xy) - pitch_center;
  return shoot_phase && yaw_error >= yaw_lower && yaw_error <= yaw_upper &&
         pitch_error >= pitch_lower && pitch_error <= pitch_upper;
}
}  // namespace auto_buff_v2
