#ifndef MAX_ENTROPY_TRACKER_UTILS_TRACKER_MOTION_GUARD_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_TRACKER_MOTION_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <initializer_list>

#include <Eigen/Core>

#include "max_entropy_tracker/core/config.hpp"

namespace fyt::auto_aim {

/// Result of guarding the linear motion states (velocity / acceleration).
///
/// `scale` reports the multiplicative factor actually applied to each state:
/// 1.0 = untouched, (0, 1) = shrunk by clamping or decay, 0.0 = zeroed.
/// Callers that also maintain the filter covariance must mirror the same
/// factor on the matching covariance rows/columns; a scale of exactly 0.0
/// means the state was pinned to zero and its variance must be rebuilt at a
/// nominal level (see syncGuardedCovariance) instead of collapsed, otherwise
/// the filter can never move the state away from zero again.
struct GuardedLinearMotion {
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double velocity_scale = 1.0;
  double acceleration_scale = 1.0;
};

/// Result of guarding the whole-vehicle yaw rate. Same scale contract as
/// GuardedLinearMotion.
struct GuardedYawRate {
  double value = 0.0;
  double scale = 1.0;
};

inline GuardedLinearMotion guardLinearMotion(
    Eigen::Vector3d velocity, Eigen::Vector3d acceleration,
    bool observation_missing, double dt,
    const TrackerMotionGuardParameters& config) {
  GuardedLinearMotion result;
  if (!velocity.allFinite() || !acceleration.allFinite()) {
    result.velocity_scale = 0.0;
    result.acceleration_scale = 0.0;
    return result;
  }
  result.velocity = velocity;
  result.acceleration = acceleration;
  if (!config.enabled) return result;

  auto clamp_norm = [](Eigen::Vector3d& value, double max_norm) {
    const double norm = value.norm();
    if (std::isfinite(max_norm) && max_norm > 0.0 && norm > max_norm) {
      const double scale = max_norm / norm;
      value *= scale;
      return scale;
    }
    return 1.0;
  };

  result.velocity_scale *=
      clamp_norm(result.velocity, config.max_linear_speed_mps);
  result.acceleration_scale *=
      clamp_norm(result.acceleration, config.max_linear_acceleration_mps2);

  if (observation_missing) {
    const double bounded_dt = std::clamp(dt, 0.0, 0.5);
    const double half_life = std::max(
        config.temp_lost_velocity_half_life_s, 1e-3);
    const double decay = std::exp(-std::log(2.0) * bounded_dt / half_life);
    // A stale CA acceleration can recreate the false translation immediately.
    result.acceleration.setZero();
    result.acceleration_scale = 0.0;
    if (decay < 1e-3) {
      // Decayed below numerical relevance: pin to zero so the covariance is
      // rebuilt rather than driven to an unrecoverable denormal.
      result.velocity.setZero();
      result.velocity_scale = 0.0;
    } else {
      result.velocity *= decay;
      result.velocity_scale *= decay;
    }
  }

  if (result.velocity.norm() <
      std::max(config.stationary_speed_deadband_mps, 0.0)) {
    result.velocity.setZero();
    result.velocity_scale = 0.0;
    result.acceleration.setZero();
    result.acceleration_scale = 0.0;
  }
  return result;
}

/// Guards the whole-vehicle yaw rate. There is intentionally no
/// missing-observation decay: rotation persists, so a spinning target keeps
/// its rotational prediction instead of turning into straight-line drift.
inline GuardedYawRate guardYawRate(
    double yaw_rate, const TrackerMotionGuardParameters& config) {
  GuardedYawRate result;
  if (!std::isfinite(yaw_rate)) {
    result.scale = 0.0;
    return result;
  }
  result.value = yaw_rate;
  if (!config.enabled) return result;

  const double max_rate = config.max_yaw_rate_rad_s;
  const double magnitude = std::abs(result.value);
  if (std::isfinite(max_rate) && max_rate > 0.0 && magnitude > max_rate) {
    result.value = std::copysign(max_rate, result.value);
    result.scale = max_rate / magnitude;
  }
  if (std::abs(result.value) <
      std::max(config.yaw_rate_deadband_rad_s, 0.0)) {
    result.value = 0.0;
    result.scale = 0.0;
  }
  return result;
}

/// Mirrors a guard action onto the filter covariance. For each guarded state
/// index: 0 < scale < 1 shrinks the row/column (equivalent to the congruent
/// scaling D·P·D, which preserves positive semi-definiteness); scale == 0
/// zeroes the row/column and rebuilds the diagonal at reset_std^2 so the
/// state stays observable and can leave the guard pin later.
inline void syncGuardedCovariance(
    Eigen::MatrixXd& covariance, std::initializer_list<int> state_indices,
    double applied_scale, double reset_std) {
  if (applied_scale >= 1.0) return;
  const double reset_variance = reset_std * reset_std;
  for (const int index : state_indices) {
    if (index < 0 || index >= covariance.rows()) continue;
    if (applied_scale <= 0.0) {
      covariance.row(index).setZero();
      covariance.col(index).setZero();
      covariance(index, index) = reset_variance;
    } else {
      covariance.row(index) *= applied_scale;
      covariance.col(index) *= applied_scale;
    }
  }
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_TRACKER_MOTION_GUARD_HPP_
