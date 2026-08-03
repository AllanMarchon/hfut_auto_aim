// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_ADAPTIVE_MEASUREMENT_NOISE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_ADAPTIVE_MEASUREMENT_NOISE_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::vehicle {

// Shared facing/range-adaptive measurement covariance model, factored out of
// VehicleUkfBackendV1 so the InEKF backend can use the exact same R model
// (via V1StyleNoiseModel). These functions encode the validated vision
// lessons: PnP noise grows with range, yaw observations degrade at distance
// and near edge-on/frontal views, and far-range yaw can flip by +-pi.

// Range-adaptive position noise: PnP lateral error grows ~linearly with range
// (pixel subtense), depth error ~quadratically (depth-from-apparent-size).
// The configured sigmas are tuned at ~2m; without this scaling every update
// beyond ~4m gets NIS/sanity-rejected and the track flaps between drop and
// re-acquire (seen on fast-receding targets at 4-6m: all_gate_fail storms
// with detection boxes clearly visible).
inline void distance_noise_factors(const ObservationData &obs, double &lateral,
                                   double &depth) {
  const double range = std::hypot(obs.x, obs.y);
  lateral = std::clamp(range / 2.0, 1.0, 3.0);
  depth = lateral * lateral;
}

// Yaw observations degrade catastrophically at range in vision: beyond ~3m
// single-plate PnP yaw is off by +-100-150deg on the webots mesh (corner
// jitter at ~40px apparent width), which would otherwise gate out every
// frame. Inflate R_yaw quadratically with range so the filter tracks on
// position alone at distance and the yaw state simply dead-reckons.
inline double yaw_range_noise_factor(const ObservationData &obs) {
  const double range = std::hypot(obs.x, obs.y);
  const double f = std::clamp(range / 3.0, 1.0, 5.0);
  return f * f;
}

// At range, single-plate PnP yaw intermittently flips by +-pi (detector
// artifact at small apparent size): the plate position fits but the measured
// yaw is 180deg out, so every panel hypothesis fails chi2_yaw and the track
// drops despite clear detections. Fold near-pi innovations back: the flip is
// physically impossible for an outward-facing armor, while a genuine >150deg
// model lag is far less likely at these ranges. Only applied beyond 3m —
// close-range pi-scale innovations are real mis-tracks and must keep failing.
inline double fold_flipped_yaw(double innov_yaw, double range_m) {
  if (range_m > 3.0 && std::abs(innov_yaw) > 2.6) {
    innov_yaw -= std::copysign(M_PI, innov_yaw);
  }
  return innov_yaw;
}

inline Eigen::Matrix4d measurement_covariance(
    const ObservationData &obs, double sigma_pos_xy, double sigma_pos_z,
    double sigma_yaw) {
  double lat, dep;
  distance_noise_factors(obs, lat, dep);
  const double sp = sigma_pos_xy * lat;
  const double sz = sigma_pos_z * dep;
  const Eigen::Matrix4d fallback = Eigen::Vector4d(
      sp * sp, sp * sp, sz * sz, sigma_yaw * sigma_yaw).asDiagonal();

  if (!obs.ba_pnp.has_value()) return fallback;
  const auto &meta = obs.ba_pnp.value();
  if (!meta.valid || !meta.cov_valid || !meta.frame_aligned ||
      !meta.cov_xyz_yaw.allFinite()) {
    return fallback;
  }

  const Eigen::Matrix4d symmetric =
      0.5 * (meta.cov_xyz_yaw + meta.cov_xyz_yaw.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(symmetric);
  if (solver.info() != Eigen::Success) return fallback;

  Eigen::Vector4d eigenvalues = solver.eigenvalues().cwiseMax(1e-12);
  Eigen::Matrix4d ba_cov = solver.eigenvectors() * eigenvalues.asDiagonal() *
                           solver.eigenvectors().transpose();
  // PnP covariance can be overconfident, especially for yaw near a frontal
  // view. Keep the configured model noise as a floor on every component so a
  // tiny per-frame covariance cannot collapse the posterior before a panel
  // transition.
  for (int i = 0; i < 4; ++i) {
    const double floor_var = i < 2 ? sp * sp : (i == 2 ? sz * sz : sigma_yaw * sigma_yaw);
    ba_cov(i, i) = std::max(ba_cov(i, i), floor_var);
  }
  return ba_cov;
}

// Facing-adaptive yaw noise factor (sp_vision_25 target.cpp:191): yaw
// observations from edge-on plates are far less trustworthy, so scale the yaw
// variance up with the facing angle — front-on 1x, edge-on ~1.94x. Keeps
// static PnP yaw jitter from spinning up a false whole-vehicle rotation.
inline double facing_yaw_noise_factor(const ObservationData &obs) {
  const double view_dir = std::atan2(obs.y, obs.x) + M_PI;
  const double facing =
      std::min(std::abs(normalize_angle(obs.yaw - view_dir)), M_PI / 2.0);
  const bool planar_pnp = obs.ba_pnp.has_value() &&
      obs.ba_pnp->valid && obs.ba_pnp->pose_estimate_mode != 0;
  if (planar_pnp) {
    // Planar yaw is weak near a frontal view (little perspective cue), most
    // informative at moderate obliquity, then degrades again as the plate
    // collapses edge-on. Keep direct/calibrated pose channels unchanged.
    const double frontal = 1.0 + 1.5 * std::pow(std::cos(facing), 2.0);
    const double edge = std::clamp(
        (facing - M_PI / 3.0) / (M_PI / 6.0), 0.0, 1.0);
    return frontal * (1.0 + 2.0 * edge * edge);
  }
  return std::log(facing + 1.0) + 1.0;
}

inline Eigen::Matrix4d facing_adaptive_yaw_covariance(
    const ObservationData &obs, double sigma_pos_xy, double sigma_pos_z,
    double sigma_yaw) {
  Eigen::Matrix4d R =
      measurement_covariance(obs, sigma_pos_xy, sigma_pos_z, sigma_yaw);
  const double factor = facing_yaw_noise_factor(obs);
  const double range_factor = yaw_range_noise_factor(obs);
  R(3, 3) *= factor * factor * range_factor * range_factor;
  return R;
}

inline Eigen::Matrix<double, 8, 8> dual_measurement_covariance(
    const ObservationData &obs0, const ObservationData &obs1,
    double sigma_pos_xy, double sigma_pos_z, double sigma_yaw,
    double scale) {
  Eigen::Matrix<double, 8, 8> covariance =
      Eigen::Matrix<double, 8, 8>::Zero();
  covariance.block<4, 4>(0, 0) =
      facing_adaptive_yaw_covariance(obs0, sigma_pos_xy, sigma_pos_z, sigma_yaw);
  covariance.block<4, 4>(4, 4) =
      facing_adaptive_yaw_covariance(obs1, sigma_pos_xy, sigma_pos_z, sigma_yaw);
  return covariance * std::max(scale, 0.0);
}

}  // namespace fyt::auto_aim::vehicle

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_ADAPTIVE_MEASUREMENT_NOISE_HPP_
