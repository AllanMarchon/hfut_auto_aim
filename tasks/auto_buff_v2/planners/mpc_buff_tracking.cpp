#include "mpc_buff_tracking.hpp"

#include <array>
#include <cmath>

#include "../aimers/rune_aimer_factory.hpp"

namespace auto_buff_v2
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

std::optional<auto_aim::Trajectory> make_reference_trajectory(
  const RuneEstimate & state, Timestamp center_time, double speed, double yaw0,
  const RuneAimer & aimer)
{
  std::array<AimSolution, auto_aim::HORIZON + 2> samples;
  for (int i = 0; i <= auto_aim::HORIZON + 1; ++i) {
    const auto time = offset_time(
      center_time, (static_cast<double>(i - 1 - auto_aim::HALF_HORIZON)) * auto_aim::DT);
    const auto solution = aimer.aim_at(state, time, speed);
    if (!solution) return std::nullopt;
    samples[i] = *solution;
  }

  auto_aim::Trajectory result;
  for (int i = 0; i < auto_aim::HORIZON; ++i) {
    const auto & before = samples[i];
    const auto & center = samples[i + 1];
    const auto & after = samples[i + 2];
    result.col(i) << std::remainder(center.angles.x() - yaw0, 2 * kPi),
      std::remainder(after.angles.x() - before.angles.x(), 2 * kPi) / (2 * auto_aim::DT),
      center.angles.y(), (after.angles.y() - before.angles.y()) / (2 * auto_aim::DT);
  }
  return result;
}
}  // namespace

MpcBuffTracking::MpcBuffTracking(Config config)
: aimer_(make_rune_aimer(config))
{
}

std::optional<MpcBuffTrackingRequest> MpcBuffTracking::prepare(
  std::uint64_t target_generation, const std::optional<RuneEstimate> & target,
  double bullet_speed, Timestamp now, const io::GimbalState & gimbal)
{
  const auto aimed = aimer_->aim(target_generation, target, bullet_speed, gimbal, now);
  if (!aimed || !aimed->plan.control) return std::nullopt;
  const auto trajectory = make_reference_trajectory(
    *target, aimed->prediction_time, aimed->bullet_speed, aimed->solution.angles.x(), *aimer_);
  if (!trajectory) return std::nullopt;

  MpcBuffTrackingRequest request;
  request.trajectory = *trajectory;
  request.yaw0 = aimed->solution.angles.x();
  request.distance = std::hypot(target->center.x(), target->center.y());
  request.fly_time = aimed->solution.fly_time;
  request.rune_center = target->center;
  request.aimpoint = aimed->solution.point;
  return request;
}

bool MpcBuffTracking::fire_advice(
  const MpcBuffTrackingRequest & request, const auto_aim::Plan & plan,
  const io::GimbalState & gimbal, Timestamp now)
{
  return aimer_->fire_advice(request.rune_center, request.aimpoint, plan, gimbal, now);
}
}  // namespace auto_buff_v2
