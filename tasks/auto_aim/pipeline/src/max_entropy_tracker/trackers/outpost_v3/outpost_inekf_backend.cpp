// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v3/outpost_inekf_backend.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "max_entropy_tracker/utils/angle_utils.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"

namespace fyt::auto_aim::outpost_v3 {

using Idx = OutpostStateIndex;

// ═══════════════════════════════════════════════════════════════════
// Static state layout
// ═══════════════════════════════════════════════════════════════════

StateLayout OutpostInEKFBackend::s_layout_{};
bool OutpostInEKFBackend::s_layout_initialized_ = false;

// Shared across all instances on purpose — see the member declarations.
Eigen::Vector3d OutpostInEKFBackend::anchor_ema_ = Eigen::Vector3d::Zero();
Eigen::Vector3d OutpostInEKFBackend::anchor_init_ = Eigen::Vector3d::Zero();
std::deque<Eigen::Vector3d> OutpostInEKFBackend::anchor_window_{};
double OutpostInEKFBackend::last_cross_time_ = -1.0;
std::deque<double> OutpostInEKFBackend::cross_intervals_{};
double OutpostInEKFBackend::timing_rate_mag_ = 0.0;

StateLayout OutpostStateIndex::build_layout() {
  StateLayout layout;
  layout.register_state("X", X);
  layout.register_state("Y", Y);
  layout.register_state("Z", Z);
  layout.register_state("VX", VX);
  layout.register_state("VY", VY);
  layout.register_state("VZ", VZ);
  layout.register_state("AX", AX);
  layout.register_state("AY", AY);
  layout.register_state("AZ", AZ);
  layout.register_state("YAW", YAW);
  layout.register_state("YAW_RATE", YAW_RATE);
  layout.register_state("YAW_ACC", YAW_ACC);
  // Provide R1/R2/DZA stubs so DynamicStateIndex accessors don't throw
  layout.register_state("R1", kDim);
  layout.register_state("R2", kDim);
  layout.register_state("DZA", kDim);
  layout.freeze();
  return layout;
}

DynamicStateIndex OutpostStateIndex::make_idx() {
  static StateLayout layout = build_layout();
  return DynamicStateIndex(layout);
}

// ═══════════════════════════════════════════════════════════════════
// Lie group operations (SO(2))
// ═══════════════════════════════════════════════════════════════════

double OutpostInEKFBackend::left_jacobian_SO2(double dpsi) {
  if (std::abs(dpsi) < 1e-8) return 1.0;
  return std::sin(dpsi) / dpsi;
}

double OutpostInEKFBackend::right_jacobian_SO2(double dpsi) {
  return left_jacobian_SO2(dpsi);  // SO(2) is abelian
}

// ═══════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════

OutpostInEKFBackend::OutpostInEKFBackend(const OutpostV3Config &cfg,
                                         double dt)
    : cfg_(cfg), dt_(dt), geom_(cfg.geometry), state_idx_(s_layout_) {
  if (!s_layout_initialized_) {
    s_layout_ = OutpostStateIndex::build_layout();
    s_layout_initialized_ = true;
  }
  x_ = Eigen::VectorXd::Zero(Idx::kDim);
  const auto &ip = cfg_.initial_P;
  P_ = Eigen::MatrixXd::Identity(Idx::kDim, Idx::kDim);
  P_(Idx::X, Idx::X) = ip.pos;
  P_(Idx::Y, Idx::Y) = ip.pos;
  P_(Idx::Z, Idx::Z) = ip.pos;
  P_(Idx::VX, Idx::VX) = ip.vel;
  P_(Idx::VY, Idx::VY) = ip.vel;
  P_(Idx::VZ, Idx::VZ) = ip.vel;
  P_(Idx::AX, Idx::AX) = ip.acc;
  P_(Idx::AY, Idx::AY) = ip.acc;
  P_(Idx::AZ, Idx::AZ) = ip.acc;
  P_(Idx::YAW, Idx::YAW) = ip.yaw;
  P_(Idx::YAW_RATE, Idx::YAW_RATE) = ip.yaw_rate;
  P_(Idx::YAW_ACC, Idx::YAW_ACC) = ip.yaw_acc;
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

// ═══════════════════════════════════════════════════════════════════
// Reset
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::reset(const ObservationData &obs, int panel_id,
                                double /*r1*/, double /*r2*/,
                                double /*dza*/) {
  // r1/r2/dza are ignored — structure starts from the configured prior and
  // adapts online within +/-5% (see commit()).
  geom_ = cfg_.geometry;
  current_panel_id_ = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  x_ = initialize_state(obs, current_panel_id_);
  anchor_ema_ = Eigen::Vector3d(x_(Idx::X), x_(Idx::Y), x_(Idx::Z));
  anchor_init_ = anchor_ema_;
  // The candidate window SURVIVES reset: it is filter-independent (built
  // from observation radial yaw, see commit) and the outpost is static, so
  // history stays valid across re-acquisitions. Clearing it starved the
  // median — flip-flopping tracks reset every few frames, the window never
  // reached 16 samples, and every reset re-anchored on a single noisy
  // draw. With enough history the median is a better anchor than the new
  // draw, so prefer it.
  if (anchor_window_.size() >= 16) {
    anchor_ema_ = per_axis_median(anchor_window_);
    anchor_init_ = anchor_ema_;
    x_(Idx::X) = anchor_ema_.x();
    x_(Idx::Y) = anchor_ema_.y();
    x_(Idx::Z) = anchor_ema_.z();
  }
  k_ = current_panel_id_;
  last_k_ = current_panel_id_;
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
  last_innov_yaw_ = 0.0;
  last_nis_ = -1.0;
  last_update_type_ = 0;
  last_armor_angle_time_ = -1.0;
  armor_angle_ema_ = 0.0;
  armor_angle_magnitude_ema_ = 0.0;
  armor_angle_valid_streak_ = 0;
  last_cross_time_ = -1.0;
  // cross_intervals_ intentionally survives reset: plate crossing timing is
  // filter-independent and the outpost spins uniformly, so history stays
  // valid across re-acquisitions (same argument as the anchor window).
  initialized_ = true;
}

// ═══════════════════════════════════════════════════════════════════
// Predict
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::predict(double dt) {
  if (!initialized_) return;
  last_update_type_ = 0;
  last_nis_ = -1.0;

  const double dt2 = dt * dt;

  // Static anchor: the outpost is a rigid static rotor. Single-plate 3D
  // observations make (center, yaw, panel) jointly ambiguous — a free
  // center absorbs rotation and forms mirror/wrong-panel fixed points
  // (measured: vyaw sign flipping, center circling at ~0.5m). Freeze the
  // center at the reset anchor (initialize_state is panel-independent and
  // verified accurate to a few cm) and let yaw/panel do all the fitting.
  // ── Mean propagation (nominal dynamics) ──
  Eigen::VectorXd x_pred = x_;
  // yaw += yaw_rate·dt + 0.5·yaw_acc·dt²
  x_pred(Idx::YAW) = normalize_angle(
      x_(Idx::YAW) + x_(Idx::YAW_RATE) * dt +
      0.5 * x_(Idx::YAW_ACC) * dt2);
  // yaw_rate += yaw_acc·dt
  x_pred(Idx::YAW_RATE) += x_(Idx::YAW_ACC) * dt;
  // Center is frozen at the anchor; translation states pinned to zero.
  for (const int index : {Idx::VX, Idx::VY, Idx::VZ,
                          Idx::AX, Idx::AY, Idx::AZ}) {
    x_pred(index) = 0.0;
  }

  // ── Error-state transition F ──
  Eigen::Matrix<double, Idx::kDim, Idx::kDim> F =
      Eigen::Matrix<double, Idx::kDim, Idx::kDim>::Identity();
  // Yaw ← yaw_rate
  F(Idx::YAW, Idx::YAW_RATE) = dt;
  // Yaw ← yaw_acc
  F(Idx::YAW, Idx::YAW_ACC) = 0.5 * dt2;
  // Yaw_rate ← yaw_acc
  F(Idx::YAW_RATE, Idx::YAW_ACC) = dt;
  // Translation error states are frozen with the anchor (see above).
  for (const int index : {Idx::X, Idx::VX, Idx::AX,
                          Idx::Y, Idx::VY, Idx::AY,
                          Idx::Z, Idx::VZ, Idx::AZ}) {
    F.row(index).setZero();
    F.col(index).setZero();
  }

  // ── Covariance propagation: P = F·P·Fᵀ + Q ──
  auto Q = build_process_Q(dt);

  // Rotation-witness-driven translation damping: while the measured plate
  // bearing shows sustained rotation, the center must NOT be allowed to
  // absorb the plates' circular motion as translation (the "center orbits
  // instead of yaw rotates" trap — seen as the estimated center wandering
  // on a ~0.5m circle with vyaw pinned at 0). Pin the translation process
  // noise down so rotation is forced into the yaw channel. A static or
  // slowly drifting outpost keeps the full translation bandwidth.
  const double rotation_level =
      std::clamp(std::abs(armor_angle_ema_) / 2.0, 0.0, 1.0);
  const double translation_scale = 1.0 - 0.95 * rotation_level;
  if (translation_scale < 0.999) {
    const double factor = std::sqrt(translation_scale);
    for (const int index : {Idx::X, Idx::VX, Idx::AX,
                            Idx::Y, Idx::VY, Idx::AY,
                            Idx::Z, Idx::VZ, Idx::AZ}) {
      Q.row(index) *= factor;
      Q.col(index) *= factor;
    }
  }

  P_ = F * P_ * F.transpose() + Q;

  x_ = x_pred;
  P_ = ensure_positive_definite(P_);

  // Steady-state yaw-rate pull toward the bearing witness. Committed
  // edge-on records bias the filter's rate (measured: vyaw oscillating
  // -2.4..-5.5 around -3.2 vs true -2.51 rad/s, the dominant remaining
  // aim error via over-led prediction). The pull target is the
  // frontal-crossing TIMING rate when available (unbiased: the record yaw
  // bias vanishes at the crossing), otherwise the finite-difference EMA
  // (biased a few % fast by the bias trend across each passage).
  if (armor_angle_valid_streak_ >= 5) {
    const double coherence = std::abs(armor_angle_ema_) /
        std::max(armor_angle_magnitude_ema_, 1e-6);
    if (coherence > 0.85 && std::abs(armor_angle_ema_) > 1.0) {
      double target = armor_angle_ema_;
      if (timing_rate_mag_ > 0.5) {
        target = std::copysign(timing_rate_mag_, armor_angle_ema_);
      }
      x_(Idx::YAW_RATE) += 0.15 * (target - x_(Idx::YAW_RATE));
      // The spin is uniform: any residual yaw_acc acts as a constant rate
      // leak fighting the pull (measured: acc ~-0.4 rad/s² holding the
      // rate 3.5% fast against the exact timing witness). Decay it out.
      x_(Idx::YAW_ACC) *= 0.90;
    }
  }

  // Filter-independent anchor maintenance (see noteAnchorObservation).
  applyAnchorMedian();
}

// ═══════════════════════════════════════════════════════════════════
// PredictContext
// ═══════════════════════════════════════════════════════════════════

vehicle::PredictContext OutpostInEKFBackend::buildPredictContext() const {
  vehicle::PredictContext ctx;
  ctx.x_prior = x_;
  ctx.P_prior = P_;
  ctx.k_prior = k_;
  ctx.last_k_prior = last_k_;
  ctx.hybrid_prior.panel_id = current_panel_id_;
  ctx.hybrid_prior.phase_index = current_panel_id_;
  return ctx;
}

// ═══════════════════════════════════════════════════════════════════
// Observation model:  z_pred = p + Rz(yaw) · r_i
// ═══════════════════════════════════════════════════════════════════

Eigen::Vector3d OutpostInEKFBackend::obs_model(
    const Eigen::VectorXd &x, int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  const double yaw = x(Idx::YAW);
  const double armor_yaw = yaw + geom_.panel_angles[pid];
  const double cos_a = std::cos(armor_yaw);
  const double sin_a = std::sin(armor_yaw);

  Eigen::Vector3d z;
  z(0) = x(Idx::X) + geom_.radius * cos_a;
  z(1) = x(Idx::Y) + geom_.radius * sin_a;
  z(2) = x(Idx::Z) + geom_.z_offsets[pid];
  return z;
}

// ═══════════════════════════════════════════════════════════════════
// World-frame Jacobian:  H_world = ∂z_pred/∂x  (3 × 12)
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix<double, 3, Idx::kDim>
OutpostInEKFBackend::obs_jacobian_world(
    const Eigen::VectorXd &x, int panel_id) const {
  (void)x;
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  const double yaw = x(Idx::YAW);
  const double armor_yaw = yaw + geom_.panel_angles[pid];
  const double cos_a = std::cos(armor_yaw);
  const double sin_a = std::sin(armor_yaw);

  Eigen::Matrix<double, 3, Idx::kDim> H =
      Eigen::Matrix<double, 3, Idx::kDim>::Zero();

  // Row 0: ∂x_obs/∂state
  H(0, Idx::X) = 1.0;
  H(0, Idx::YAW) = -geom_.radius * sin_a;

  // Row 1: ∂y_obs/∂state
  H(1, Idx::Y) = 1.0;
  H(1, Idx::YAW) = geom_.radius * cos_a;

  // Row 2: ∂z_obs/∂state
  H(2, Idx::Z) = 1.0;

  return H;
}

// ═══════════════════════════════════════════════════════════════════
// Body-frame innovation:  ν = Rz(-yaw) · (z_obs − z_pred)
// ═══════════════════════════════════════════════════════════════════

Eigen::Vector3d OutpostInEKFBackend::compute_body_frame_innovation(
    const Eigen::Vector3d &z_obs, const Eigen::Vector3d &z_pred,
    double center_yaw) const {
  const double dx = z_obs(0) - z_pred(0);
  const double dy = z_obs(1) - z_pred(1);
  const double cos_cy = std::cos(center_yaw);
  const double sin_cy = std::sin(center_yaw);

  Eigen::Vector3d innov;
  innov(0) =  cos_cy * dx + sin_cy * dy;
  innov(1) = -sin_cy * dx + cos_cy * dy;
  innov(2) = z_obs(2) - z_pred(2);
  return innov;
}

// ═══════════════════════════════════════════════════════════════════
// Body-frame H:  H_body = Ad_Rz(-yaw) · H_world
// Key simplification: body-frame YAW derivatives depend only on
// panel phase φᵢ, not center_yaw.
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix<double, 3, Idx::kDim>
OutpostInEKFBackend::compute_body_frame_H(
    const Eigen::Matrix<double, 3, Idx::kDim> &H_world,
    double center_yaw, int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  const double cos_cy = std::cos(center_yaw);
  const double sin_cy = std::sin(center_yaw);
  const double phi = geom_.panel_angles[pid];
  const double cos_phi = std::cos(phi);
  const double sin_phi = std::sin(phi);

  // Start from world-frame H, rotate position rows (0,1)
  Eigen::Matrix<double, 3, Idx::kDim> H_body = H_world;

  // Row 0: cos(cy)·H(0,:) + sin(cy)·H(1,:)
  H_body.row(0) = cos_cy * H_world.row(0) + sin_cy * H_world.row(1);
  // Row 1: -sin(cy)·H(0,:) + cos(cy)·H(1,:)
  H_body.row(1) = -sin_cy * H_world.row(0) + cos_cy * H_world.row(1);

  // Replace YAW derivatives with simplified body-frame form
  // that depends only on panel phase φ, not center_yaw.
  H_body(0, Idx::YAW) = -geom_.radius * sin_phi;
  H_body(1, Idx::YAW) =  geom_.radius * cos_phi;

  return H_body;
}

// ═══════════════════════════════════════════════════════════════════
// Body-frame R:  R_body = Ad_Rz(-yaw) · R_world · Ad_Rz(-yaw)ᵀ
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix3d OutpostInEKFBackend::rotate_R_to_body_frame(
    const Eigen::Matrix3d &R_world, double center_yaw) const {
  const double cos_cy = std::cos(center_yaw);
  const double sin_cy = std::sin(center_yaw);

  // Rz^T = [cos,  sin]   on the [x,y] subspace
  //        [-sin, cos]
  Eigen::Matrix3d R_body = R_world;

  // Rotate rows 0,1 by Rz^T
  for (int col = 0; col < 3; ++col) {
    const double vx = R_world(0, col);
    const double vy = R_world(1, col);
    R_body(0, col) =  cos_cy * vx + sin_cy * vy;
    R_body(1, col) = -sin_cy * vx + cos_cy * vy;
  }
  // Rotate cols 0,1 by Rz
  for (int row = 0; row < 3; ++row) {
    const double vx = R_body(row, 0);
    const double vy = R_body(row, 1);
    R_body(row, 0) = cos_cy * vx + sin_cy * vy;
    R_body(row, 1) = -sin_cy * vx + cos_cy * vy;
  }

  return R_body;
}

// ═══════════════════════════════════════════════════════════════════
// Build observation noise R
// ═══════════════════════════════════════════════════════════════════

Eigen::Matrix3d OutpostInEKFBackend::build_observation_R(
    const ObservationData &obs) const {
  const double sxy2 = cfg_.observation_noise.sigma_pos_xy *
                      cfg_.observation_noise.sigma_pos_xy;
  const double sz2 = cfg_.observation_noise.sigma_pos_z *
                     cfg_.observation_noise.sigma_pos_z;
  // The direct-pose publisher writes the honest per-plate noise (including
  // view-angle-dependent systematic bias magnitude) into the observation's
  // covariance metadata — at 5m it can exceed 0.2m, while the config floor
  // is 0.02m. Trusting the floor would NIS-reject every biased plate and
  // deadlock the tracker in AMBIGUOUS mode. The bias lies along the camera
  // ray, so the record noise is projected through the observation's
  // elevation: xy gets cos(elev), z gets sin(elev) — z observations stay
  // sharp enough to discriminate the staggered panel heights (0.102m).
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = sxy2;
  R(1, 1) = sxy2;
  R(2, 2) = sz2;
  if (obs.ba_pnp.has_value() && obs.ba_pnp->cov_valid &&
      obs.ba_pnp->cov_xyz_yaw.allFinite()) {
    const double horiz = std::hypot(obs.x, obs.y);
    const double cos_e =
        horiz > 1e-6 ? std::clamp(horiz / std::sqrt(horiz * horiz + obs.z * obs.z), 0.0, 1.0) : 1.0;
    const double sin_e = std::sqrt(std::max(0.0, 1.0 - cos_e * cos_e));
    const double p_xy = obs.ba_pnp->cov_xyz_yaw(0, 0) * cos_e * cos_e;
    const double p_z = obs.ba_pnp->cov_xyz_yaw(2, 2) * sin_e * sin_e;
    R(0, 0) = std::max(R(0, 0), p_xy);
    R(1, 1) = std::max(R(1, 1), p_xy);
    R(2, 2) = std::max(R(2, 2), p_z);
  }
  return R;
}

// ═══════════════════════════════════════════════════════════════════
// evaluateSingle  —  body-frame invariant innovation, 3D obs
// ═══════════════════════════════════════════════════════════════════

vehicle::MeasurementEval OutpostInEKFBackend::evaluateSingle(
    const vehicle::PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;

  // Build 3D observation vector
  Eigen::Vector3d z_obs(obs.x, obs.y, obs.z);

  // World-frame prediction
  Eigen::Vector3d z_pred = obs_model(ctx.x_prior, pid);

  // World-frame Jacobian
  auto H_world = obs_jacobian_world(ctx.x_prior, pid);

  // Body-frame innovation
  const double center_yaw = ctx.x_prior(Idx::YAW);
  Eigen::Vector3d innov = compute_body_frame_innovation(
      z_obs, z_pred, center_yaw);

  // Body-frame H
  auto H_body = compute_body_frame_H(H_world, center_yaw, pid);

  // Body-frame R
  Eigen::Matrix3d R_world = build_observation_R(obs);
  Eigen::Matrix3d R_body = rotate_R_to_body_frame(R_world, center_yaw);

  // Innovation covariance: S = H·P·Hᵀ + R  (all body-frame)
  Eigen::Matrix3d S = H_body * ctx.P_prior * H_body.transpose() + R_body;

  vehicle::MeasurementEval eval;
  Eigen::LLT<Eigen::Matrix3d> llt(S);
  if (llt.info() != Eigen::Success) {
    eval.valid = false;
    eval.reject_reason = "S_not_spd";
    return eval;
  }

  auto solved = llt.solve(innov);
  double nis = innov.dot(solved);

  double logdet = 0.0;
  const auto &L = llt.matrixL();
  for (int i = 0; i < 3; ++i) logdet += std::log(L(i, i));
  logdet *= 2.0;
  constexpr int m = 3;  // obs dim
  double log_likelihood =
      -0.5 * (nis + logdet + m * std::log(2.0 * M_PI));

  // Per-component chi2
  double chi2_pos = 0.0;
  for (int i = 0; i < 3; ++i) {
    chi2_pos += (innov(i) * innov(i)) / std::max(S(i, i), 1e-10);
  }

  const auto &gt = cfg_.gate;
  bool gate_pass = true;
  if (nis > gt.single_total_nis) gate_pass = false;
  if (chi2_pos > gt.single_pos_chi2) gate_pass = false;

  eval.valid = true;
  eval.gate_pass = gate_pass;
  eval.nis = nis;
  eval.mahalanobis = std::sqrt(nis);
  eval.log_likelihood = log_likelihood;
  eval.score = log_likelihood;
  eval.chi2_pos = chi2_pos;
  eval.chi2_yaw = 0.0;
  eval.innovation = innov;
  eval.S = S;
  eval.z_pred = z_pred;
  eval.z_obs = z_obs;
  eval.obs_yaw_radial = obs.yaw;
  if (obs.ba_pnp.has_value() && obs.ba_pnp->cov_valid &&
      obs.ba_pnp->cov_xyz_yaw.allFinite()) {
    eval.obs_pos_std_m =
        std::sqrt(std::max(0.0, obs.ba_pnp->cov_xyz_yaw(0, 0)));
  }

  if (!gate_pass) {
    std::ostringstream oss;
    oss << "gate_fail:nis=" << nis << ",chi2_pos=" << chi2_pos;
    eval.reject_reason = oss.str();
  }
  return eval;
}

// ═══════════════════════════════════════════════════════════════════
// evaluateDual  —  stub, Phase 1 does not implement dual
// ═══════════════════════════════════════════════════════════════════

vehicle::MeasurementEval OutpostInEKFBackend::evaluateDual(
    const vehicle::PredictContext & /*ctx*/,
    const ObservationData & /*obs0*/,
    const ObservationData & /*obs1*/,
    int /*panel_id_0*/, int /*panel_id_1*/) const {
  vehicle::MeasurementEval eval;
  eval.valid = false;
  eval.reject_reason = "dual_not_implemented";
  return eval;
}

// ═══════════════════════════════════════════════════════════════════
// tryUpdateSingle
// ═══════════════════════════════════════════════════════════════════

vehicle::UkfTrial OutpostInEKFBackend::tryUpdateSingle(
    const vehicle::PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  vehicle::UkfTrial trial;
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;

  trial.hypothesis.kind = vehicle::HypothesisKind::Single;
  trial.hypothesis.assignments[0] = {0, pid};
  trial.hypothesis.assignment_count = 1;

  vehicle::MeasurementEval eval = evaluateSingle(ctx, obs, pid);
  trial.eval = eval;
  if (!eval.valid) {
    trial.reject_reason = eval.reject_reason;
    return trial;
  }

  // Recompute body-frame quantities
  const double center_yaw = ctx.x_prior(Idx::YAW);
  auto H_world = obs_jacobian_world(ctx.x_prior, pid);
  auto H_body = compute_body_frame_H(H_world, center_yaw, pid);

  Eigen::Vector3d innov = eval.innovation;
  Eigen::Matrix3d R_world = build_observation_R(obs);
  Eigen::Matrix3d R_body = rotate_R_to_body_frame(R_world, center_yaw);
  Eigen::Matrix3d S = eval.S;

  // Kalman gain: K = P·Hᵀ·S⁻¹  (12 × 3)
  Eigen::Matrix<double, Idx::kDim, 3> K =
      ctx.P_prior * H_body.transpose() * S.inverse();

  // Static anchor (see predict): the center is frozen at the reset anchor,
  // so position corrections must not move it or its velocity/acceleration;
  // yaw and its rates absorb the innovation instead.
  for (const int index : {Idx::X, Idx::VX, Idx::AX,
                          Idx::Y, Idx::VY, Idx::AY,
                          Idx::Z, Idx::VZ, Idx::AZ}) {
    K.row(index).setZero();
  }

  // When the bearing witness is authoritative, freeze the yaw-RATE channel
  // as well: single-plate innovations yank the rate by +-0.5 rad/s every
  // commit (measured: state oscillating -1.6..-3.5 while the clean witness
  // sits at -2.44 vs true -2.51), and that oscillation directly becomes
  // prediction phase error. The rate then comes solely from the witness
  // pull in predict(); the yaw PHASE row stays Kalman-updated so commits
  // still lock the layout phase. Without a fed witness (e.g. vision
  // records with unknown covariance) the filter keeps its normal rate
  // update.
  {
    const double coherence = std::abs(armor_angle_ema_) /
        std::max(armor_angle_magnitude_ema_, 1e-6);
    const bool witness_authoritative = armor_angle_valid_streak_ >= 5 &&
        coherence > 0.85 && std::abs(armor_angle_ema_) > 1.0;
    if (witness_authoritative) {
      K.row(Idx::YAW_RATE).setZero();
      K.row(Idx::YAW_ACC).setZero();
    }
  }

  // Error-state correction, then retract to nominal state
  Eigen::VectorXd dx = K * innov;
  Eigen::VectorXd x_post = retract_left_invariant(ctx.x_prior, dx);

  // Joseph form covariance
  Eigen::Matrix<double, Idx::kDim, Idx::kDim> I_KH =
      Eigen::Matrix<double, Idx::kDim, Idx::kDim>::Identity() -
      K * H_body;
  Eigen::MatrixXd P_post =
      I_KH * ctx.P_prior * I_KH.transpose() +
      K * R_body * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = pid;
  trial.last_k_post = ctx.k_prior;
  trial.hybrid_post.panel_id = pid;
  trial.hybrid_post.phase_index = pid;

  trial.reconstruction_pos_error =
      compute_reconstruction_error(x_post, obs, pid);

  // Clamp oversized center corrections instead of rejecting the update
  // outright (same convention as the vehicle backend): when the witness
  // nudge or a re-acquisition produces a >max_center_jump correction, a
  // hard reject discards the very observation that heals the track. A
  // clamped step converges in 2-3 frames without any teleport.
  {
    const double limit = std::max(1e-3, cfg_.posterior_sanity.max_center_jump);
    Eigen::Vector3d delta(x_post(Idx::X) - ctx.x_prior(Idx::X),
                          x_post(Idx::Y) - ctx.x_prior(Idx::Y),
                          x_post(Idx::Z) - ctx.x_prior(Idx::Z));
    const double jump = delta.norm();
    if (jump > limit) {
      delta *= limit / jump;
      trial.x_post(Idx::X) = ctx.x_prior(Idx::X) + delta.x();
      trial.x_post(Idx::Y) = ctx.x_prior(Idx::Y) + delta.y();
      trial.x_post(Idx::Z) = ctx.x_prior(Idx::Z) + delta.z();
      trial.center_jump_clamped = true;
      x_post = trial.x_post;
    }
  }

  trial.posterior_sanity_pass =
      check_posterior_sanity(ctx.x_prior, x_post, P_post);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = "posterior_sanity_fail";
    trial.success = false;
  }

  return trial;
}

// ═══════════════════════════════════════════════════════════════════
// tryUpdateDual  —  stub
// ═══════════════════════════════════════════════════════════════════

vehicle::UkfTrial OutpostInEKFBackend::tryUpdateDual(
    const vehicle::PredictContext & /*ctx*/,
    const ObservationData & /*obs0*/,
    const ObservationData & /*obs1*/,
    int /*panel_id_0*/, int /*panel_id_1*/) const {
  vehicle::UkfTrial trial;
  trial.reject_reason = "dual_not_implemented";
  return trial;
}

// ═══════════════════════════════════════════════════════════════════
// commit
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::commit(const vehicle::UkfTrial &trial) {
  if (!trial.success) return;

  x_ = trial.x_post;
  P_ = trial.P_post;
  k_ = trial.k_post;
  last_k_ = trial.last_k_post;

  if (trial.hypothesis.kind == vehicle::HypothesisKind::Single) {
    current_panel_id_ = trial.hypothesis.assignments[0].panel_id;
  }

  // Bounded online structure adaptation: per-panel height offsets are
  // PRIORS, not constants — field geometry may deviate ~5%. Learn them from
  // committed observations with a slow EMA clamped to +/-5% of the
  // configured prior. The RADIUS is intentionally NOT learned online:
  // est_r = |obs - center| couples it to the frozen anchor, forming a
  // positive feedback loop (measured as a 3cm -> 26cm center walk over one
  // run). A true radius deviation is instead absorbed by the anchor as a
  // <=5%-of-r center bias along the line of sight, harmless for aiming.
  //
  // The anchor itself is maintained OUTSIDE the filter — see
  // noteAnchorObservation/predict. Committed innovations carry no center
  // signal (single-plate ambiguity: the filter re-fits yaw around whatever
  // center it is given), so nothing center-related happens here.
  if (trial.hypothesis.kind == vehicle::HypothesisKind::Single) {
    const int pid = ((current_panel_id_ % kNumPanels) + kNumPanels) % kNumPanels;
    const auto &z_obs = trial.eval.z_obs;
    const double est_dz = z_obs(2) - x_(Idx::Z);
    constexpr double kAlpha = 0.05;  // ~20-frame time constant
    const double prior_dz = cfg_.geometry.z_offsets[pid];
    geom_.z_offsets[pid] += kAlpha * (est_dz - geom_.z_offsets[pid]);
    const double tol_dz = 0.05 * cfg_.geometry.radius;
    geom_.z_offsets[pid] = std::clamp(
        geom_.z_offsets[pid], prior_dz - tol_dz, prior_dz + tol_dz);

    // Phase anchor: the observation's radial yaw is EXTERNAL to the filter
    // (direct record / normal+pi in vision) and unbiased for near-frontal
    // plates, so center_yaw = radial_yaw - panel_angle is a filter-
    // independent phase reference. With the rate pinned by the witness,
    // the phase still settles with a steady-state lag (~0.4 rad, measured
    // as the dominant impact offset); bleed it off with a small per-commit
    // pull. Gated TWICE: to trustworthy near-frontal records (biased
    // records would pull the phase toward their bias), and to witness-
    // authoritative periods (during ambiguity wrong panel assignments
    // would amplify the churn with +-120° candidates).
    const double coherence = std::abs(armor_angle_ema_) /
        std::max(armor_angle_magnitude_ema_, 1e-6);
    const bool witness_authoritative = armor_angle_valid_streak_ >= 5 &&
        coherence > 0.85 && std::abs(armor_angle_ema_) > 1.0;
    constexpr double kMaxAnchorRecordStd = 0.06;
    if (witness_authoritative &&
        trial.eval.obs_pos_std_m > 0.0 &&
        trial.eval.obs_pos_std_m < kMaxAnchorRecordStd) {
      const double cand_phase = normalize_angle(
          trial.eval.obs_yaw_radial - geom_.panel_angles[pid]);
      const double dphi = angle_difference(cand_phase, x_(Idx::YAW));
      constexpr double kMaxPhasePull = 0.5;
      if (std::abs(dphi) < kMaxPhasePull) {
        x_(Idx::YAW) = normalize_angle(x_(Idx::YAW) + 0.05 * dphi);
      }
    }
  }

  last_nis_ = trial.eval.nis;
  if (trial.eval.innovation.size() >= 3) {
    last_innov_xyz_ = trial.eval.innovation.head<3>();
  }
  last_innov_yaw_ = 0.0;
  last_update_type_ =
      (trial.hypothesis.kind == vehicle::HypothesisKind::Single) ? 1 : 2;

  P_ = ensure_positive_definite(P_);
}

// ═══════════════════════════════════════════════════════════════════
// snapshot
// ═══════════════════════════════════════════════════════════════════

vehicle::BackendSnapshot OutpostInEKFBackend::snapshot() const {
  vehicle::BackendSnapshot snap;
  snap.x = x_;
  snap.P = P_;
  snap.k = k_;
  snap.last_k = last_k_;
  snap.current_panel_id = current_panel_id_;
  snap.hybrid.panel_id = current_panel_id_;
  snap.hybrid.phase_index = current_panel_id_;
  snap.last_nis = last_nis_;
  snap.last_innov_xyz = last_innov_xyz_;
  snap.last_innov_yaw = last_innov_yaw_;
  snap.last_update_type = last_update_type_;
  return snap;
}

// ═══════════════════════════════════════════════════════════════════
// SpinFilterInterface accessors
// ═══════════════════════════════════════════════════════════════════

Eigen::Vector3d OutpostInEKFBackend::get_center_position() const {
  return Eigen::Vector3d(x_(Idx::X), x_(Idx::Y), x_(Idx::Z));
}

std::pair<double, double> OutpostInEKFBackend::get_radii() const {
  return {geom_.radius, geom_.radius};
}

double OutpostInEKFBackend::get_yaw() const {
  return normalize_angle(x_(Idx::YAW));
}

double OutpostInEKFBackend::get_raw_yaw() const { return x_(Idx::YAW); }

// ═══════════════════════════════════════════════════════════════════
// Posterior sanity checks
// ═══════════════════════════════════════════════════════════════════

bool OutpostInEKFBackend::check_posterior_sanity(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &x_post,
    const Eigen::MatrixXd &P_post) const {
  const auto &ps = cfg_.posterior_sanity;
  const bool debug = std::getenv("HFUT_DEBUG_OUTPOST") != nullptr;

  // Center jump
  Eigen::Vector3d prior_center(x_prior(Idx::X), x_prior(Idx::Y),
                                x_prior(Idx::Z));
  Eigen::Vector3d post_center(x_post(Idx::X), x_post(Idx::Y),
                               x_post(Idx::Z));
  const double center_jump = (post_center - prior_center).norm();
  if (center_jump > ps.max_center_jump) {
    if (debug)
      std::fprintf(stderr, "[outpost] sanity fail: center_jump=%.3f>%.2f\n",
                   center_jump, ps.max_center_jump);
    return false;
  }

  // Yaw jump
  double yaw_jump =
      std::abs(angle_difference(x_post(Idx::YAW), x_prior(Idx::YAW)));
  if (yaw_jump > ps.max_yaw_jump) {
    if (debug)
      std::fprintf(stderr, "[outpost] sanity fail: yaw_jump=%.3f>%.2f\n",
                   yaw_jump, ps.max_yaw_jump);
    return false;
  }

  // Yaw rate / yaw acc physical limits
  if (std::abs(x_post(Idx::YAW_RATE)) > ps.max_yaw_rate) {
    if (debug)
      std::fprintf(stderr, "[outpost] sanity fail: yaw_rate=%.2f>%.1f\n",
                   x_post(Idx::YAW_RATE), ps.max_yaw_rate);
    return false;
  }
  if (std::abs(x_post(Idx::YAW_ACC)) > ps.max_yaw_acc) {
    if (debug)
      std::fprintf(stderr, "[outpost] sanity fail: yaw_acc=%.2f>%.1f\n",
                   x_post(Idx::YAW_ACC), ps.max_yaw_acc);
    return false;
  }

  // Covariance sanity
  if (!P_post.allFinite()) return false;
  for (int i = 0; i < Idx::kDim; ++i) {
    if (P_post(i, i) <= 0.0) return false;
  }

  return true;
}

// ═══════════════════════════════════════════════════════════════════
// Reconstruction error
// ═══════════════════════════════════════════════════════════════════

double OutpostInEKFBackend::compute_reconstruction_error(
    const Eigen::VectorXd &x_post, const ObservationData &obs,
    int panel_id) const {
  Eigen::Vector3d z_rebuild = obs_model(x_post, panel_id);
  Eigen::Vector3d pos_obs(obs.x, obs.y, obs.z);
  return (pos_obs - z_rebuild).norm();
}

// ═══════════════════════════════════════════════════════════════════
// Left-invariant retraction:  X̂⁺ = Exp(δξ̂) ∘ X̂
// ═══════════════════════════════════════════════════════════════════

Eigen::VectorXd OutpostInEKFBackend::retract_left_invariant(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &dx) const {
  Eigen::VectorXd x_post = x_prior;

  // Position (additive, left-invariant for ℝⁿ)
  x_post(Idx::X) += dx(Idx::X);
  x_post(Idx::Y) += dx(Idx::Y);
  x_post(Idx::Z) += dx(Idx::Z);

  // Velocity
  x_post(Idx::VX) += dx(Idx::VX);
  x_post(Idx::VY) += dx(Idx::VY);
  x_post(Idx::VZ) += dx(Idx::VZ);

  // Acceleration
  x_post(Idx::AX) += dx(Idx::AX);
  x_post(Idx::AY) += dx(Idx::AY);
  x_post(Idx::AZ) += dx(Idx::AZ);

  // Rotation: Exp(δψ) · R̂ ⇔ yaw⁺ = normalize(yaw + δψ)
  x_post(Idx::YAW) = normalize_angle(x_prior(Idx::YAW) + dx(Idx::YAW));

  // Yaw rate / yaw acc
  x_post(Idx::YAW_RATE) += dx(Idx::YAW_RATE);
  x_post(Idx::YAW_ACC) += dx(Idx::YAW_ACC);

  return x_post;
}

// ═══════════════════════════════════════════════════════════════════
// State initialization from observation
// ═══════════════════════════════════════════════════════════════════

Eigen::VectorXd OutpostInEKFBackend::initialize_state(
    const ObservationData &obs, int panel_id) const {
  const int pid = ((panel_id % kNumPanels) + kNumPanels) % kNumPanels;
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(Idx::kDim);

  // Direct armor_pose records carry the TRUE radial yaw (center → armor) in
  // ObservationData.yaw (pipeline.cpp:387). An earlier version subtracted pi
  // here assuming the vision-PnP normal convention, which mirrored the
  // initial center by 2·radius and poisoned every downstream fit.
  const double armor_yaw = normalize_angle(obs.yaw);
  const double center_yaw =
      normalize_angle(armor_yaw - geom_.panel_angles[pid]);

  // Back-project from armor position to center position
  x0(Idx::X) = obs.x - geom_.radius * std::cos(armor_yaw);
  x0(Idx::Y) = obs.y - geom_.radius * std::sin(armor_yaw);
  x0(Idx::Z) = obs.z - geom_.z_offsets[pid];
  x0(Idx::YAW) = center_yaw;

  if (std::getenv("HFUT_DEBUG_OUTPOST") != nullptr) {
    std::fprintf(stderr,
                 "[outpost] init panel=%d obs=(%.3f,%.3f,%.3f) obs_yaw=%.3f "
                 "armor_yaw=%.3f -> center=(%.3f,%.3f,%.3f) yaw=%.3f\n",
                 pid, obs.x, obs.y, obs.z, obs.yaw, armor_yaw,
                 x0(Idx::X), x0(Idx::Y), x0(Idx::Z), center_yaw);
  }

  return x0;
}

// ═══════════════════════════════════════════════════════════════════
// Process noise Q (discrete white-noise jerk / CA model)
// ═══════════════════════════════════════════════════════════════════

Eigen::Vector3d OutpostInEKFBackend::per_axis_median(
    const std::deque<Eigen::Vector3d> &window) {
  Eigen::Vector3d med = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    values.reserve(window.size());
    for (const auto &c : window) values.push_back(c(axis));
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    med(axis) = values[mid];
  }
  return med;
}

// ═══════════════════════════════════════════════════════════════════
// Filter-independent anchor estimation
//
// The frozen center anchor (see predict) is only as good as the single
// observation that seeded it — measured as run-to-run hit-rate swings
// (26.9% vs 34.9% with identical code). Maintaining it INSIDE the filter
// fails twice over: committed innovations carry no center signal
// (single-plate ambiguity: the filter re-fits yaw around whatever center
// it is given), and requiring commits starves the estimator exactly when
// the anchor is bad (bad anchor -> NIS rejections -> no commits).
//
// So candidates are computed from RAW observations, bypassing the filter:
//   xy: obs_xy - r_prior * e(obs.yaw)   (radial yaw is external to the
//                                        filter manifold; pipeline.cpp
//                                        provides it in both modes)
//   z : obs.z                            (median plate height == middle
//                                        plate == center height by config)
// Only nearly-frontal records enter: the direct publisher inflates the
// record noise std with the view-angle systematic bias magnitude (range
// pull up to ~10% of range, yaw pull toward the camera bearing — both
// pointing AWAY from the camera), so the std is a direct proxy for bias.
// The anchor follows the per-axis window MEDIAN, which never references
// the current anchor and therefore cannot deadlock.
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::noteAnchorObservation(const ObservationData &obs) {
  double record_std = 0.0;
  if (obs.ba_pnp.has_value() && obs.ba_pnp->cov_valid &&
      obs.ba_pnp->cov_xyz_yaw.allFinite()) {
    record_std = std::sqrt(std::max(0.0, obs.ba_pnp->cov_xyz_yaw(0, 0)));
  }
  // std==0 means "unknown" (vision PnP) and is accepted, preserving the
  // previous behaviour there. The record std also absorbs the frontal
  // range bias (effectivePositionStd = hypot(noise, rangeBias)): at
  // std=0.06 the residual range pull can still reach ~6cm and the window
  // median settles ~2.2cm downrange of truth (measured); 0.03 caps the
  // per-record bias at ~3cm while still filling the window in seconds.
  constexpr double kMaxAnchorRecordStd = 0.03;
  if (record_std > kMaxAnchorRecordStd) return;

  const double r_prior = cfg_.geometry.radius;
  anchor_window_.emplace_back(obs.x - r_prior * std::cos(obs.yaw),
                              obs.y - r_prior * std::sin(obs.yaw), obs.z);
  constexpr size_t kAnchorWindow = 64;
  if (anchor_window_.size() > kAnchorWindow) anchor_window_.pop_front();
}

void OutpostInEKFBackend::applyAnchorMedian() {
  if (!initialized_ || anchor_window_.size() < 16) return;
  const Eigen::Vector3d anchor_target = per_axis_median(anchor_window_);
  constexpr double kAlphaAnchor = 0.05;
  anchor_ema_ += kAlphaAnchor * (anchor_target - anchor_ema_);
  constexpr double kMaxAnchorDrift = 0.5;
  const Eigen::Vector3d drift = anchor_ema_ - anchor_init_;
  if (drift.norm() > kMaxAnchorDrift) {
    anchor_ema_ = anchor_init_ + drift * (kMaxAnchorDrift / drift.norm());
  }
  x_(Idx::X) = anchor_ema_.x();
  x_(Idx::Y) = anchor_ema_.y();
  x_(Idx::Z) = anchor_ema_.z();
  if (std::getenv("HFUT_DEBUG_OUTPOST") != nullptr) {
    std::fprintf(stderr,
                 "[outpost] anchor median=(%.3f,%.3f,%.3f) anchor=(%.3f,%.3f,%.3f) n=%zu witness=%.3f timing=%.3f state_rate=%.3f\n",
                 anchor_target.x(), anchor_target.y(), anchor_target.z(),
                 anchor_ema_.x(), anchor_ema_.y(), anchor_ema_.z(),
                 anchor_window_.size(), armor_angle_ema_, timing_rate_mag_,
                 x_(Idx::YAW_RATE));
  }
}

Eigen::Matrix<double, Idx::kDim, Idx::kDim>
OutpostInEKFBackend::build_process_Q(double dt) const {
  Eigen::Matrix<double, Idx::kDim, Idx::kDim> Q =
      Eigen::Matrix<double, Idx::kDim, Idx::kDim>::Zero();

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;

  const double q_a = cfg_.process_noise.acc * cfg_.process_noise.acc;
  const double q_alpha =
      cfg_.process_noise.yaw_acc * cfg_.process_noise.yaw_acc;

  // CA block: [p, v, a] for each axis (X, Y, Z)
  auto fill_ca_block = [&](int p_idx, int v_idx, int a_idx, double q) {
    Q(p_idx, p_idx) = q * dt5 / 20.0;
    Q(p_idx, v_idx) = q * dt4 / 8.0;
    Q(v_idx, p_idx) = Q(p_idx, v_idx);
    Q(p_idx, a_idx) = q * dt3 / 6.0;
    Q(a_idx, p_idx) = Q(p_idx, a_idx);
    Q(v_idx, v_idx) = q * dt3 / 3.0;
    Q(v_idx, a_idx) = q * dt2 / 2.0;
    Q(a_idx, v_idx) = Q(v_idx, a_idx);
    Q(a_idx, a_idx) = q * dt;
  };

  fill_ca_block(Idx::X, Idx::VX, Idx::AX, q_a);
  fill_ca_block(Idx::Y, Idx::VY, Idx::AY, q_a);
  fill_ca_block(Idx::Z, Idx::VZ, Idx::AZ, q_a);

  // Yaw CA block: [yaw, yaw_rate, yaw_acc]
  fill_ca_block(Idx::YAW, Idx::YAW_RATE, Idx::YAW_ACC, q_alpha);

  return Q;
}

// ═══════════════════════════════════════════════════════════════════
// Rotation witness: model-free armor-bearing finite difference that
// breaks the spin-direction (mirror) trap. Ported from the vehicle
// backend's noteArmorAngle semantics.
// ═══════════════════════════════════════════════════════════════════

void OutpostInEKFBackend::noteArmorAngle(double angle_rad, double timestamp) {
  if (!initialized_) return;
  if (last_armor_angle_time_ > 0.0 && timestamp > last_armor_angle_time_) {
    const double dt = timestamp - last_armor_angle_time_;
    const double delta = angle_difference(angle_rad, last_armor_angle_);
    // The feed is frontal-gated (outpost_tracker_v3): samples within one
    // frontal passage are a few frames apart (|delta| <= vyaw*0.15s), while
    // passage-to-passage jumps are +-120deg +/- the window width. Both a
    // loose delta gate AND a dt cap are needed — a 0.6-0.7rad jump across a
    // passage gap sneaks past a 0.7 delta gate as a huge wrong-sign rate
    // (clamped to +-15) and drags the witness magnitude down (measured:
    // witness settling at 1.85 vs true 2.51 rad/s).
    if (std::abs(delta) < 0.35 && dt > 1e-3 && dt < 0.2) {
      const double inst = std::clamp(delta / dt, -15.0, 15.0);
      constexpr double kAlpha = 0.15;
      if (armor_angle_valid_streak_ < 5) {
        const double n = armor_angle_valid_streak_ + 1.0;
        armor_angle_ema_ += (inst - armor_angle_ema_) / n;
        armor_angle_magnitude_ema_ +=
            (std::abs(inst) - armor_angle_magnitude_ema_) / n;
      } else {
        armor_angle_ema_ =
            (1.0 - kAlpha) * armor_angle_ema_ + kAlpha * inst;
        armor_angle_magnitude_ema_ =
            (1.0 - kAlpha) * armor_angle_magnitude_ema_ +
            kAlpha * std::abs(inst);
      }
      ++armor_angle_valid_streak_;

      // Frontal-crossing timing: interpolate the moment the plate bearing
      // crosses the camera bearing (the record yaw bias is exactly zero
      // there, unlike anywhere else in the passage). Crossings of
      // successive plates are 1/3 rotation apart, so |vyaw| =
      // (2pi/3)/interval. Consecutive samples here are guaranteed
      // same-passage by the gates above.
      const double cam_bearing =
          std::atan2(-anchor_ema_.y(), -anchor_ema_.x());
      const double prev_rel =
          angle_difference(last_armor_angle_, cam_bearing);
      const double cur_rel = angle_difference(angle_rad, cam_bearing);
      if (prev_rel * cur_rel < 0.0) {
        const double frac =
            std::abs(prev_rel) / (std::abs(prev_rel) + std::abs(cur_rel));
        const double t_cross = last_armor_angle_time_ + frac * dt;
        if (last_cross_time_ > 0.0) {
          const double interval = t_cross - last_cross_time_;
          // 1/3 rotation at any plausible spin: 0.4s..2.0s.
          if (interval > 0.4 && interval < 2.0) {
            cross_intervals_.push_back(interval);
            if (cross_intervals_.size() > 8) cross_intervals_.pop_front();
            std::vector<double> sorted(cross_intervals_.begin(),
                                       cross_intervals_.end());
            const size_t mid = sorted.size() / 2;
            std::nth_element(sorted.begin(), sorted.begin() + mid,
                             sorted.end());
            timing_rate_mag_ = (2.0 * M_PI / 3.0) / sorted[mid];
          }
        }
        last_cross_time_ = t_cross;
      }

      // Mirror-trap sign conflicts are handled by the tracker with a full
      // re-anchor (a vyaw nudge cannot fix the mirrored phase — it just
      // makes the filter fight itself). Here we only handle the "no rate
      // yet" case: seed the rate toward the witness so the filter starts
      // tracking rotation early.
      const double state_rate = x_(Idx::YAW_RATE);
      const double coherence = std::abs(armor_angle_ema_) /
          std::max(armor_angle_magnitude_ema_, 1e-6);
      const bool missing_rate = std::abs(state_rate) < 0.5 &&
          std::abs(armor_angle_ema_) > 1.0;
      if (armor_angle_valid_streak_ >= 5 && coherence > 0.70 &&
          missing_rate) {
        const double clamped = std::clamp(armor_angle_ema_, -15.0, 15.0);
        x_(Idx::YAW_RATE) = 0.3 * state_rate + 0.7 * clamped;
        P_(Idx::YAW_RATE, Idx::YAW_RATE) =
            std::max(P_(Idx::YAW_RATE, Idx::YAW_RATE), 1.0);
      }
    } else {
      // A +-120deg plate switch invalidates the finite difference — skip
      // the sample, but KEEP the streak/EMA: they were built exclusively
      // from same-passage pairs and stay valid across the gap. Resetting
      // the streak flapped the witness authority off for ~5 frames every
      // 0.83s passage switch, during which biased commits yanked the rate
      // and periodically cascaded into TEMP_LOST loops (measured: with
      // reset, state_rate spikes to 3.3-4.4 and 200+ transitions/run;
      // without, track stable and vyaw pinned at the true 2.513).
    }
  }
  last_armor_angle_ = angle_rad;
  last_armor_angle_time_ = timestamp;
}

}  // namespace fyt::auto_aim::outpost_v3
