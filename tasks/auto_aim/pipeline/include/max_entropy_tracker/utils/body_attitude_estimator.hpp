#ifndef MAX_ENTROPY_TRACKER_UTILS_BODY_ATTITUDE_ESTIMATOR_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_BODY_ATTITUDE_ESTIMATOR_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fyt::auto_aim {

/// Whole-vehicle attitude (pitch/roll) estimate derived from plate poses.
struct BodyAttitudeEstimate {
  bool valid = false;
  bool trusted_for_geometry = false;
  // These Euler components are relative to the tracker's yaw phase. For a
  // symmetric four-panel target that phase is only observable modulo pi/2;
  // compare the world up/orientation after phase alignment, not raw signs.
  double pitch_rad = 0.0;
  double roll_rad = 0.0;
  Eigen::Vector3d up{0.0, 0.0, 1.0};  // smoothed chassis up direction (world)
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

/// Estimates the chassis up direction from armor plate orientations and
/// converts it (plus the tracked yaw) into body pitch/roll.
///
/// Per plate, the chassis up is recovered from the plate's up axis
/// (R_plate * z_obj) rotated by the armor mount pitch about the plate's
/// width axis. Single-plate attitude is noisy (~±8 deg on typical PnP), so
/// the up direction is EMA-smoothed; treat the readout as a slow chassis
/// signal (slope, brake rocking), not a per-frame measurement.
class BodyAttitudeEstimator {
 public:
  BodyAttitudeEstimator(double mount_pitch_rad, double ema_alpha)
      : mount_pitch_rad_(mount_pitch_rad), ema_alpha_(ema_alpha) {}

  /// Feed one plate orientation in the world frame.
  /// R_plate columns: X = plate normal, Y = plate width, Z = plate height.
  void addPlateOrientation(
      const Eigen::Matrix3d &R_plate_world,
      bool trusted_for_geometry = false) {
    addPlateOrientation(
        R_plate_world, mount_pitch_rad_, trusted_for_geometry);
  }

  void addPlateOrientation(
      const Eigen::Matrix3d &R_plate_world,
      double mount_pitch_rad,
      bool trusted_for_geometry) {
    const Eigen::Vector3d width_axis = R_plate_world.col(1).normalized();
    const Eigen::Vector3d plate_up = R_plate_world.col(2).normalized();
    // Self-calibrating de-tilt sign: the mount angle between plate-up and
    // chassis-up is fixed, but its sign flips with the plate-frame convention
    // of the producer (offset profile vs PnP measurement), and hardcoding it
    // per platform proved brittle (+15deg -> 28.5deg, +7deg -> 20.5deg,
    // -15deg -> 3.6deg median chassis-up error on webots PnP). De-tilting by
    // ±|mount_pitch| and keeping the more vertical candidate removes the
    // sign dependency; it stays correct while the true chassis tilt is
    // below |mount_pitch| (15deg), which covers normal robot poses.
    const double mp = std::abs(mount_pitch_rad);
    const Eigen::Vector3d candidate_pos =
        (Eigen::AngleAxisd(mp, width_axis) * plate_up).normalized();
    const Eigen::Vector3d candidate_neg =
        (Eigen::AngleAxisd(-mp, width_axis) * plate_up).normalized();
    const Eigen::Vector3d chassis_up =
        (candidate_pos.z() >= candidate_neg.z()) ? candidate_pos : candidate_neg;
    trusted_for_geometry_ = trusted_for_geometry_ || trusted_for_geometry;
    if (trusted_for_geometry) {
      // Calibrated/direct plate frames are already a full-state measurement.
      // Preserve their current layout attitude instead of phase-lagging it
      // through the slow PnP diagnostic filter.
      up_ = chassis_up;
      up_valid_ = true;
      return;
    }
    if (!up_valid_) {
      up_ = chassis_up;
      up_valid_ = true;
      return;
    }
    Eigen::Vector3d aligned = chassis_up;
    if (aligned.dot(up_) < 0.0) aligned = -aligned;
    up_ = ((1.0 - ema_alpha_) * up_ + ema_alpha_ * aligned).normalized();
  }

  /// Current estimate; body_yaw is the tracked whole-vehicle yaw (world).
  BodyAttitudeEstimate estimate(double body_yaw) const {
    BodyAttitudeEstimate out;
    if (!up_valid_) return out;
    out.valid = true;
    out.trusted_for_geometry = trusted_for_geometry_;
    out.up = up_;
    const Eigen::Vector3d up_in_yaw_frame =
        Eigen::AngleAxisd(-body_yaw, Eigen::Vector3d::UnitZ()) * up_;
    out.roll_rad = std::atan2(
        -up_in_yaw_frame.y(),
        std::hypot(up_in_yaw_frame.x(), up_in_yaw_frame.z()));
    out.pitch_rad =
        std::atan2(up_in_yaw_frame.x(), up_in_yaw_frame.z());
    out.orientation =
        Eigen::AngleAxisd(body_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(out.pitch_rad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(out.roll_rad, Eigen::Vector3d::UnitX());
    out.orientation.normalize();
    return out;
  }

  bool valid() const { return up_valid_; }
  void reset() {
    up_valid_ = false;
    trusted_for_geometry_ = false;
  }

 private:
  double mount_pitch_rad_;
  double ema_alpha_;
  Eigen::Vector3d up_{0.0, 0.0, 1.0};
  bool up_valid_ = false;
  bool trusted_for_geometry_ = false;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_BODY_ATTITUDE_ESTIMATOR_HPP_
