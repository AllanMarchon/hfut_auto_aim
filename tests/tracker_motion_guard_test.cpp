#include "max_entropy_tracker/utils/tracker_motion_guard.hpp"
#include "max_entropy_tracker/utils/tracking_output_policy.hpp"

#include <cmath>
#include <cstdio>

namespace {

bool near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool near(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs,
          double tolerance = 1e-9) {
  return (lhs - rhs).norm() <= tolerance;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "%s\n", message);
  return code;
}

}  // namespace

int main() {
  fyt::auto_aim::TrackerMotionGuardParameters config;
  const Eigen::Vector3d velocity(6.0, 0.0, 0.0);
  const Eigen::Vector3d acceleration(0.0, 12.0, 0.0);

  // Disabled guard: pass-through, scales stay 1.
  auto result = fyt::auto_aim::guardLinearMotion(
      velocity, acceleration, false, 0.01, config);
  if (!near(result.velocity, velocity) ||
      !near(result.acceleration, acceleration) ||
      !near(result.velocity_scale, 1.0) ||
      !near(result.acceleration_scale, 1.0)) {
    return fail("disabled motion guard changed valid state", 1);
  }

  // Clamp: norms capped, scale reports the shrink factor.
  config.enabled = true;
  config.max_linear_speed_mps = 3.0;
  config.max_linear_acceleration_mps2 = 8.0;
  result = fyt::auto_aim::guardLinearMotion(
      velocity, acceleration, false, 0.01, config);
  if (!near(result.velocity.norm(), 3.0) ||
      !near(result.acceleration.norm(), 8.0) ||
      !near(result.velocity_scale, 0.5) ||
      !near(result.acceleration_scale, 8.0 / 12.0)) {
    return fail("motion guard did not cap implausible motion", 2);
  }

  // Stationary deadband: both zeroed, scales report 0.
  config.stationary_speed_deadband_mps = 0.15;
  result = fyt::auto_aim::guardLinearMotion(
      Eigen::Vector3d(0.1, 0.0, 0.0), acceleration, false, 0.01, config);
  if (!result.velocity.isZero() || !result.acceleration.isZero() ||
      !near(result.velocity_scale, 0.0) ||
      !near(result.acceleration_scale, 0.0)) {
    return fail("stationary deadband did not suppress jitter", 3);
  }

  // Missing observation: velocity half-life decay, acceleration zeroed.
  config.temp_lost_velocity_half_life_s = 0.12;
  result = fyt::auto_aim::guardLinearMotion(
      Eigen::Vector3d(2.0, 0.0, 0.0), acceleration, true, 0.12, config);
  if (!near(result.velocity.x(), 1.0) ||
      !result.acceleration.isZero() ||
      !near(result.velocity_scale, 0.5) ||
      !near(result.acceleration_scale, 0.0)) {
    return fail("missing-observation velocity decay is incorrect", 4);
  }

  // Decay below numerical relevance pins velocity to zero (scale 0) instead
  // of leaving a denormal that would collapse the covariance.
  result = fyt::auto_aim::guardLinearMotion(
      Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d::Zero(), true, 0.5,
      config);
  if (!result.velocity.isZero() || !near(result.velocity_scale, 0.0)) {
    return fail("long-missing decay did not pin velocity to zero", 5);
  }

  // Non-finite input is rejected with scales 0.
  result = fyt::auto_aim::guardLinearMotion(
      Eigen::Vector3d::Constant(NAN), acceleration, false, 0.01, config);
  if (!result.velocity.isZero() || !result.acceleration.isZero() ||
      !near(result.velocity_scale, 0.0) ||
      !near(result.acceleration_scale, 0.0)) {
    return fail("non-finite motion was not rejected", 6);
  }

  // Yaw-rate guard: disabled pass-through.
  config.enabled = false;
  auto yaw_result = fyt::auto_aim::guardYawRate(7.0, config);
  if (!near(yaw_result.value, 7.0) || !near(yaw_result.scale, 1.0)) {
    return fail("disabled yaw-rate guard changed valid state", 7);
  }

  // Yaw-rate guard: clamp reports scale, deadband zeroes, non-finite rejected.
  config.enabled = true;
  config.max_yaw_rate_rad_s = 15.0;
  config.yaw_rate_deadband_rad_s = 0.05;
  yaw_result = fyt::auto_aim::guardYawRate(-20.0, config);
  if (!near(yaw_result.value, -15.0) || !near(yaw_result.scale, 0.75)) {
    return fail("yaw-rate clamp is incorrect", 8);
  }
  yaw_result = fyt::auto_aim::guardYawRate(0.03, config);
  if (!near(yaw_result.value, 0.0) || !near(yaw_result.scale, 0.0)) {
    return fail("yaw-rate deadband did not suppress jitter", 9);
  }
  yaw_result = fyt::auto_aim::guardYawRate(NAN, config);
  if (!near(yaw_result.value, 0.0) || !near(yaw_result.scale, 0.0)) {
    return fail("non-finite yaw rate was not rejected", 10);
  }

  // Covariance sync: shrink applies D·P·D per guarded index.
  Eigen::MatrixXd covariance(4, 4);
  // States 0..3: X, VX, Y, VY (guard VX/VY with scale 0.5).
  covariance << 1.0, 0.10, 0.00, 0.20,
                0.10, 2.0, 0.30, 0.40,
                0.00, 0.30, 1.5, 0.50,
                0.20, 0.40, 0.50, 2.5;
  const Eigen::MatrixXd original = covariance;
  fyt::auto_aim::syncGuardedCovariance(covariance, {1, 3}, 0.5, 0.5);
  bool shrink_ok =
      near(covariance(1, 1), original(1, 1) * 0.25) &&
      near(covariance(3, 3), original(3, 3) * 0.25) &&
      near(covariance(1, 3), original(1, 3) * 0.25) &&
      near(covariance(0, 1), original(0, 1) * 0.5) &&
      near(covariance(2, 3), original(2, 3) * 0.5) &&
      near(covariance(0, 0), original(0, 0)) &&
      near(covariance(2, 2), original(2, 2)) &&
      near(covariance(0, 2), original(0, 2));
  if (!shrink_ok) {
    return fail("covariance shrink did not apply congruent scaling", 11);
  }

  // Covariance sync: zero scale rebuilds the diagonal at reset_std^2 and
  // clears cross-correlations without touching unrelated entries.
  covariance = original;
  fyt::auto_aim::syncGuardedCovariance(covariance, {1, 3}, 0.0, 0.5);
  bool reset_ok =
      near(covariance(1, 1), 0.25) && near(covariance(3, 3), 0.25) &&
      near(covariance(0, 1), 0.0) && near(covariance(1, 0), 0.0) &&
      near(covariance(1, 2), 0.0) && near(covariance(2, 1), 0.0) &&
      near(covariance(1, 3), 0.0) && near(covariance(3, 1), 0.0) &&
      near(covariance(2, 3), 0.0) && near(covariance(3, 2), 0.0) &&
      near(covariance(0, 0), original(0, 0)) &&
      near(covariance(2, 2), original(2, 2)) &&
      near(covariance(0, 2), original(0, 2));
  if (!reset_ok) {
    return fail("covariance reset did not rebuild the diagonal", 12);
  }

  // TEMP_LOST output freshness window.
  if (!fyt::auto_aim::temporaryPredictionIsFresh(0.15, 0.15) ||
      fyt::auto_aim::temporaryPredictionIsFresh(0.151, 0.15) ||
      fyt::auto_aim::temporaryPredictionIsFresh(std::nullopt, 0.15)) {
    return fail("temporary prediction output timeout is incorrect", 13);
  }
  return 0;
}
