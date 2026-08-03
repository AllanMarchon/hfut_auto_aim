// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/vehicle/backends/vehicle_ukf_backend_v1.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>

#include "max_entropy_tracker/trackers/vehicle/adaptive_measurement_noise.hpp"
#include "max_entropy_tracker/utils/angle_utils.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"
#include "max_entropy_tracker/utils/sigma_points.hpp"

namespace fyt::auto_aim::vehicle {

namespace {

std::shared_ptr<CompositeProcessModel> create_v1_process_model(
    const UnifiedConfig &cfg) {
  TranslationConfig tc;
  tc.cv_process_noise_vel = cfg.motion.cv_process_noise_vel;
  tc.ca_process_noise_acc = cfg.motion.ca_process_noise_acc;
  tc.singer_alpha = cfg.motion.singer_alpha;
  tc.singer_sigma = cfg.motion.singer_sigma;

  RotationConfig rc;
  rc.cv_process_noise_rate = cfg.spin.spin_process_noise_delta_rate;
  rc.ca_process_noise_acc = cfg.spin.spin_process_noise_delta_acc;

  StructuralConfig sc;
  sc.process_noise_r = cfg.motion.process_noise_r;
  sc.process_noise_dz = cfg.motion.process_noise_dz;

  return create_default_process_model(cfg.motion.translation_model,
                                      RotationModel::CV, tc, rc, sc, 3);
}

double weighted_angle_mean(const Eigen::MatrixXd &samples,
                           const Eigen::VectorXd &weights, int col) {
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (int i = 0; i < samples.rows(); ++i) {
    sin_sum += weights(i) * std::sin(samples(i, col));
    cos_sum += weights(i) * std::cos(samples(i, col));
  }
  return std::atan2(sin_sum, cos_sum);
}

void scale_vertical_covariance(Eigen::MatrixXd &covariance,
                               const DynamicStateIndex &idx, double scale,
                               bool include_position) {
  const double bounded_scale = std::clamp(scale, 1e-6, 1.0);
  const double factor = std::sqrt(bounded_scale);
  std::vector<int> indices{idx.VZ()};
  if (include_position) indices.push_back(idx.Z());
  if (idx.has("AZ")) indices.push_back(idx.AZ());
  for (const int index : indices) {
    covariance.row(index) *= factor;
    covariance.col(index) *= factor;
  }
}

}  // namespace

VehicleUkfBackendV1::VehicleUkfBackendV1(const UnifiedConfig &config, double dt)
    : config_(config),
      dt_(dt),
      process_model_(create_v1_process_model(config)),
      state_idx_(process_model_->layout()) {
  int n = process_model_->state_dim();
  x_ = Eigen::VectorXd::Zero(n);
  P_ = Eigen::MatrixXd::Identity(n, n) * 100.0;
  Q_ = process_model_->build_Q(dt_);
  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
}

void VehicleUkfBackendV1::reset(const ObservationData &obs, int panel_id,
                                double r1, double r2, double dza) {
  current_panel_id_ = ((panel_id % 4) + 4) % 4;
  auto idx = state_idx_;

  double use_r = (current_panel_id_ % 2 == 0) ? r1 : r2;
  double panel_angle = current_panel_id_ * (M_PI / 2.0);
  double center_x = obs.x - use_r * std::cos(obs.yaw);
  double center_y = obs.y - use_r * std::sin(obs.yaw);
  double center_yaw = normalize_angle(obs.yaw - panel_angle);
  k_ = 0;
  last_k_ = 0;

  x_ = Eigen::VectorXd::Zero(process_model_->state_dim());
  x_(idx.X()) = center_x;
  x_(idx.VX()) = 0.0;
  x_(idx.Y()) = center_y;
  x_(idx.VY()) = 0.0;
  const double z_offset = (current_panel_id_ % 2 == 0) ? -dza : dza;
  x_(idx.Z()) = obs.z - z_offset;
  x_(idx.VZ()) = 0.0;
  x_(idx.YAW()) = center_yaw;
  x_(idx.YAW_RATE()) = 0.0;
  x_(idx.R1()) = r1;
  x_(idx.R2()) = r2;
  x_(idx.DZA()) = dza;

  P_ = process_model_->get_initial_covariance();
  scale_vertical_covariance(
      P_, idx, config_.vehicle_tracker.ukf_v1.vertical_dynamics_scale, false);
  Q_ = process_model_->build_Q(dt_);

  last_innov_xyz_ = Eigen::VectorXd::Zero(3);
  last_innov_yaw_ = 0.0;
  last_nis_ = -1.0;
  last_update_type_ = 0;
  // Witness histories depend on sample continuity; drop the first delta
  // after a reset instead of differencing across it.
  last_obs_time_ = -1.0;
  last_armor_angle_time_ = -1.0;
  armor_angle_ema_ = 0.0;
  armor_angle_magnitude_ema_ = 0.0;
  armor_angle_samples_ = 0;
  armor_angle_valid_streak_ = 0;
  dual_height_evidence_.clear();

  initialized_ = true;
}

void VehicleUkfBackendV1::predict(double dt) {
  if (!initialized_) return;
  last_update_type_ = 0;
  last_nis_ = -1.0;

  Q_ = process_model_->build_Q(dt);
  scale_vertical_covariance(
      Q_, state_idx_, config_.vehicle_tracker.ukf_v1.vertical_dynamics_scale,
      true);

  // Adaptive rotation process noise driven by the MEASURED yaw rate (EMA of
  // finite-differenced center-yaw observations): quiet yaw channel while the
  // target measures near-stationary so static PnP yaw jitter cannot spin up
  // a phantom rotation; full bandwidth once real rotation is measured. A
  // constant delta_rate cannot serve both cases, and state-variance gating
  // never settles because observation jitter keeps the covariance inflated.
  // Bandwidth driver: prefer the model-free armor-bearing witness
  // (armor_angle_ema_) — it is panel-agnostic, so it stays clean both at rest
  // (center-yaw EMA oscillates +-2 rad/s there and would hold the yaw channel
  // open, feeding the phantom-spin tails) and inside the symmetric-plate trap
  // (where the center-yaw EMA is absorbed by the assignment walk). Fall back
  // to the center-yaw EMA while the witness is still collecting its first
  // samples.
  const double omega_meas = (armor_angle_samples_ > 30)
      ? std::abs(armor_angle_ema_)
      : std::abs(omega_meas_ema_);
  // Warmup holds full rotation bandwidth past the fixed 25-frame window while
  // the measured yaw rate is clearly non-zero: if early frames were corrupted
  // (loop-rate dip, panel confusion) the EMA needs longer to build, and
  // dropping to the quiet-target floor at frame 25 can lock a spinning target
  // into a permanent omega~=0 misread (seen in matrix L4 runs: 60s at
  // omega 0.3 vs truth 8.0 with commits flowing, so no reject-streak ever
  // fires the reset). Bounded at 150 frames so a jittery static target still
  // settles onto the quiet floor afterwards.
  const bool warmup_full = (committed_updates_ < 25) ||
      (committed_updates_ < 150 && omega_meas > 1.5);
  const double rotation_bandwidth = warmup_full
      ? 1.0
      : std::clamp(0.3 + 0.5 * omega_meas, 0.3, 1.0);
  if (rotation_bandwidth < 0.999) {
    for (const int index : {state_idx_.YAW(), state_idx_.YAW_RATE()}) {
      Q_.row(index) *= rotation_bandwidth;
      Q_.col(index) *= rotation_bandwidth;
    }
  }

  if (!sigma_gen_) {
    sigma_gen_ = std::make_unique<SigmaPointGenerator>(
        process_model_->state_dim(), config_.ukf.alpha, config_.ukf.beta,
        config_.ukf.kappa);
  }

  Eigen::MatrixXd P_safe = ensure_positive_definite(P_, 1e-6);
  Eigen::MatrixXd sigma_pts = sigma_gen_->generate(x_, P_safe);
  int n_sigma = sigma_pts.rows();
  int n = process_model_->state_dim();

  Eigen::MatrixXd sigma_pred(n_sigma, n);
  for (int i = 0; i < n_sigma; ++i) {
    sigma_pred.row(i) =
        process_model_->predict(sigma_pts.row(i).transpose(), dt);
  }

  auto Wm = sigma_gen_->Wm();
  auto Wc = sigma_gen_->Wc();

  x_ = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n_sigma; ++i) {
    x_ += Wm(i) * sigma_pred.row(i).transpose();
  }

  int yaw_index = state_idx_.YAW();
  x_(yaw_index) = weighted_angle_mean(sigma_pred, Wm, yaw_index);
  Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::VectorXd diff = sigma_pred.row(i).transpose() - x_;
    diff(yaw_index) =
        angle_difference(sigma_pred(i, yaw_index), x_(yaw_index));
    P_pred += Wc(i) * diff * diff.transpose();
  }
  P_pred += Q_;
  P_ = P_pred;
  apply_state_constraints();
  P_ = ensure_positive_definite(P_);
}

PredictContext VehicleUkfBackendV1::buildPredictContext() const {
  PredictContext ctx;
  ctx.x_prior = x_;
  ctx.P_prior = P_;
  ctx.k_prior = k_;
  ctx.last_k_prior = last_k_;
  return ctx;
}

std::string VehicleUkfBackendV1::r_type_for_panel(int panel_id) {
  return (panel_id % 2 == 0) ? "r1" : "r2";
}

std::string VehicleUkfBackendV1::armor_layer_for_panel(int panel_id) {
  return (panel_id % 2 == 0) ? "lower" : "upper";
}

Eigen::Vector4d VehicleUkfBackendV1::obs_model_single(
    const Eigen::VectorXd &x, int k, int panel_id) const {
  (void)k;
  auto idx = state_idx_;
  double x_c = x(idx.X());
  double y_c = x(idx.Y());
  double z_mean = x(idx.Z());
  double d_za = x(idx.DZA());

  double radius = (panel_id % 2 == 0) ? x(idx.R1()) : x(idx.R2());
  double center_yaw = normalize_angle(x(idx.YAW()));
  double panel_angle = panel_id * (M_PI / 2.0);
  double armor_yaw = normalize_angle(center_yaw + panel_angle);

  double x_obs = x_c + radius * std::cos(armor_yaw);
  double y_obs = y_c + radius * std::sin(armor_yaw);
  double z_offset = (panel_id % 2 == 0) ? -d_za : d_za;
  double z_obs = z_mean + z_offset;

  Eigen::Vector4d z;
  z << x_obs, y_obs, z_obs, center_yaw;
  return z;
}

void VehicleUkfBackendV1::generate_sigma_points(
    const Eigen::VectorXd &x, const Eigen::MatrixXd &P,
    Eigen::MatrixXd &out_sigma_pts,
    Eigen::VectorXd &out_Wm, Eigen::VectorXd &out_Wc) const {
  if (!sigma_gen_) {
    sigma_gen_ = std::make_unique<SigmaPointGenerator>(
        process_model_->state_dim(), config_.ukf.alpha, config_.ukf.beta,
        config_.ukf.kappa);
  }
  Eigen::MatrixXd P_safe = ensure_positive_definite(P, 1e-6);
  out_sigma_pts = sigma_gen_->generate(x, P_safe);
  out_Wm = sigma_gen_->Wm();
  out_Wc = sigma_gen_->Wc();
}

MeasurementEval VehicleUkfBackendV1::evaluateSingle(
    const PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  MeasurementEval eval;
  const int p = ((panel_id % 4) + 4) % 4;

  double panel_angle = p * (M_PI / 2.0);
  double center_yaw_obs = normalize_angle(obs.yaw - panel_angle);
  Eigen::Vector4d z_obs;
  z_obs << obs.x, obs.y, obs.z, center_yaw_obs;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd z_pred_pts(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred_pts.row(i) =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p)
            .transpose();
  }

  Eigen::Vector4d z_pred = Eigen::Vector4d::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);

  // Innovation with yaw wrap
  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = fold_flipped_yaw(angle_difference(z_obs(3), z_pred(3)),
                                  std::hypot(obs.x, obs.y));

  // Gate with the same covariance model used by the committed update. Using
  // a different R here makes a hypothesis pass one stage and then leave an
  // overconfident posterior that rejects the next panel transition.
  const auto &v1 = config_.vehicle_tracker.ukf_v1;
  Eigen::Matrix4d R = facing_adaptive_yaw_covariance(
      obs, v1.sigma_pos_xy, v1.sigma_pos_z, v1.sigma_yaw);

  // S = Pzz + R
  Eigen::MatrixXd diff_z(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
  }

  Eigen::Matrix4d Pzz = R;
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
  }

  Eigen::Matrix4d S = Pzz;

  // NIS via LLT
  Eigen::LLT<Eigen::Matrix4d> llt(S);
  if (llt.info() != Eigen::Success) {
    eval.valid = false;
    eval.reject_reason = "S_not_spd";
    return eval;
  }
  auto solved = llt.solve(innov);
  double nis = innov.dot(solved);

  double logdet = 0.0;
  const auto &L = llt.matrixL();
  for (int i = 0; i < 4; ++i) {
    logdet += std::log(L(i, i));
  }
  logdet *= 2.0;
  double log_likelihood =
      -0.5 * (nis + logdet + 4.0 * std::log(2.0 * M_PI));

  // Per-component chi2
  double chi2_yaw = (innov(3) * innov(3)) / S(3, 3);
  Eigen::Vector3d innov_pos = innov.head<3>();
  Eigen::Matrix3d S_pos = S.topLeftCorner<3, 3>();
  double chi2_pos =
      innov_pos.transpose() * S_pos.inverse() * innov_pos;

  // Gate check using configurable thresholds
  const auto &gt = config_.vehicle_tracker.ukf_v1.gate;
  bool gate_pass = true;
  if (nis > gt.single_total_nis) gate_pass = false;
  if (chi2_pos > gt.single_pos_chi2) gate_pass = false;
  if (chi2_yaw > gt.single_yaw_chi2) gate_pass = false;

  eval.valid = true;
  eval.gate_pass = gate_pass;
  eval.nis = nis;
  eval.mahalanobis = std::sqrt(nis);
  eval.log_likelihood = log_likelihood;
  eval.score = log_likelihood;
  eval.chi2_pos = chi2_pos;
  eval.chi2_yaw = chi2_yaw;
  eval.innovation = innov;
  eval.S = S;
  eval.z_pred = z_pred;
  eval.z_obs = z_obs;

  if (!gate_pass) {
    std::ostringstream oss;
    oss << "gate_fail:nis=" << nis << ",chi2_pos=" << chi2_pos
        << ",chi2_yaw=" << chi2_yaw;
    eval.reject_reason = oss.str();
    if (debug_omega_ema_ && (gate_fail_prints_++ % 30) == 0) {
      std::fprintf(stderr,
                   "[gate_fail] k=%d nis=%.1f pos=%.1f yaw=%.1f "
                   "innov=(%+.2f,%+.2f,%+.2f,%+.2f) Spos=%.3f Syaw=%.4f "
                   "obs=(%.2f,%.2f,%.2f)\n",
                   p, nis, chi2_pos, chi2_yaw, innov(0), innov(1), innov(2),
                   innov(3), S(0, 0), S(3, 3), obs.x, obs.y, obs.z);
    }
  }

  return eval;
}

MeasurementEval VehicleUkfBackendV1::evaluateDual(
    const PredictContext &ctx, const ObservationData &obs0,
    const ObservationData &obs1, int panel_id_0, int panel_id_1) const {
  MeasurementEval eval;
  const int p0 = ((panel_id_0 % 4) + 4) % 4;
  const int p1 = ((panel_id_1 % 4) + 4) % 4;

  double center_yaw_obs0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double center_yaw_obs1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  Eigen::Matrix<double, 8, 1> z_obs;
  z_obs << obs0.x, obs0.y, obs0.z, center_yaw_obs0,
           obs1.x, obs1.y, obs1.z, center_yaw_obs1;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();

  Eigen::MatrixXd z_pred_pts(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d z0 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p0);
    Eigen::Vector4d z1 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p1);
    z_pred_pts.row(i) << z0.transpose(), z1.transpose();
  }

  Eigen::Matrix<double, 8, 1> z_pred = Eigen::Matrix<double, 8, 1>::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);
  z_pred(7) = weighted_angle_mean(z_pred_pts, Wm, 7);

  Eigen::Matrix<double, 8, 1> innov = z_obs - z_pred;
  innov(3) = fold_flipped_yaw(angle_difference(z_obs(3), z_pred(3)),
                                  std::hypot(obs0.x, obs0.y));
  innov(7) = fold_flipped_yaw(angle_difference(z_obs(7), z_pred(7)),
                                  std::hypot(obs1.x, obs1.y));

  const auto &v1 = config_.vehicle_tracker.ukf_v1;
  Eigen::Matrix<double, 8, 8> R = dual_measurement_covariance(
      obs0, obs1, v1.sigma_pos_xy, v1.sigma_pos_z, v1.sigma_yaw,
      v1.dual_raw_R_scale);

  Eigen::MatrixXd diff_z(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
    diff_z(i, 7) = angle_difference(z_pred_pts(i, 7), z_pred(7));
  }

  Eigen::Matrix<double, 8, 8> Pzz = R;
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Matrix<double, 8, 1> dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
  }
  Eigen::Matrix<double, 8, 8> S = Pzz;

  Eigen::LLT<Eigen::Matrix<double, 8, 8>> llt(S);
  if (llt.info() != Eigen::Success) {
    eval.valid = false;
    eval.reject_reason = "S_not_spd";
    return eval;
  }
  auto solved = llt.solve(innov);
  double nis = innov.dot(solved);

  double logdet = 0.0;
  const auto &L = llt.matrixL();
  for (int i = 0; i < 8; ++i) {
    logdet += std::log(L(i, i));
  }
  logdet *= 2.0;
  double log_likelihood =
      -0.5 * (nis + logdet + 8.0 * std::log(2.0 * M_PI));

  double chi2_yaw0 = (innov(3) * innov(3)) / S(3, 3);
  double chi2_yaw1 = (innov(7) * innov(7)) / S(7, 7);
  double chi2_yaw = std::max(chi2_yaw0, chi2_yaw1);

  Eigen::Vector3d innov_pos0 = innov.segment<3>(0);
  Eigen::Vector3d innov_pos1 = innov.segment<3>(4);
  Eigen::Matrix3d S_pos0 = S.block<3, 3>(0, 0);
  Eigen::Matrix3d S_pos1 = S.block<3, 3>(4, 4);
  double chi2_pos0 =
      innov_pos0.transpose() * S_pos0.inverse() * innov_pos0;
  double chi2_pos1 =
      innov_pos1.transpose() * S_pos1.inverse() * innov_pos1;
  double chi2_pos = std::max(chi2_pos0, chi2_pos1);

  const auto &gt = config_.vehicle_tracker.ukf_v1.gate;
  bool gate_pass = true;
  if (nis > gt.dual_total_nis) gate_pass = false;
  if (chi2_pos0 > gt.dual_each_pos_chi2 || chi2_pos1 > gt.dual_each_pos_chi2) gate_pass = false;
  if (chi2_yaw0 > gt.dual_each_yaw_chi2 || chi2_yaw1 > gt.dual_each_yaw_chi2) gate_pass = false;

  eval.valid = true;
  eval.gate_pass = gate_pass;
  eval.nis = nis;
  eval.mahalanobis = std::sqrt(nis);
  eval.log_likelihood = log_likelihood;
  eval.score = log_likelihood;
  eval.chi2_pos = chi2_pos;
  eval.chi2_yaw = chi2_yaw;
  eval.innovation = innov;
  eval.S = S;
  eval.z_pred = z_pred;
  eval.z_obs = z_obs;

  if (!gate_pass) {
    std::ostringstream oss;
    oss << "gate_fail:nis=" << nis << ",chi2_pos=" << chi2_pos
        << ",chi2_yaw=" << chi2_yaw;
    eval.reject_reason = oss.str();
  }

  return eval;
}

UkfTrial VehicleUkfBackendV1::tryUpdateSingle(
    const PredictContext &ctx, const ObservationData &obs,
    int panel_id) const {
  UkfTrial trial;
  const int p = ((panel_id % 4) + 4) % 4;

  trial.hypothesis.kind = HypothesisKind::Single;
  trial.hypothesis.assignments[0] = {0, p};
  trial.hypothesis.assignment_count = 1;

  MeasurementEval eval = evaluateSingle(ctx, obs, p);
  trial.eval = eval;
  if (!eval.valid) {
    trial.reject_reason = eval.reject_reason;
    return trial;
  }

  double panel_angle = p * (M_PI / 2.0);
  double center_yaw_obs = normalize_angle(obs.yaw - panel_angle);
  Eigen::Vector4d z_obs;
  z_obs << obs.x, obs.y, obs.z, center_yaw_obs;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();
  int n = process_model_->state_dim();

  Eigen::MatrixXd z_pred_pts(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred_pts.row(i) =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p)
            .transpose();
  }

  Eigen::Vector4d z_pred = Eigen::Vector4d::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);

  Eigen::Vector4d innov = z_obs - z_pred;
  innov(3) = fold_flipped_yaw(angle_difference(z_obs(3), z_pred(3)),
                                  std::hypot(obs.x, obs.y));

  const auto &v1 = config_.vehicle_tracker.ukf_v1;
  double sp = v1.sigma_pos_xy;
  double sz = v1.sigma_pos_z;
  double sy = v1.sigma_yaw;
  Eigen::Matrix4d R = facing_adaptive_yaw_covariance(obs, sp, sz, sy);

  Eigen::MatrixXd diff_z(n_sigma, 4);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
  }

  Eigen::Matrix4d Pzz = R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n, 4);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
    Eigen::VectorXd dx = sigma_pts.row(i).transpose() - ctx.x_prior;
    Pxz += Wc(i) * dx * dz.transpose();
  }

  Eigen::Matrix4d S = Pzz;
  Eigen::LLT<Eigen::Matrix4d> llt(S);
  if (llt.info() != Eigen::Success) {
    trial.reject_reason = "S_not_spd_in_tryUpdate";
    return trial;
  }

  Eigen::MatrixXd Pzz_inv = S.inverse();
  Eigen::MatrixXd K = Pxz * Pzz_inv;

  // Conservative structural gain (single obs: freeze r1/r2/dza)
  auto idx = state_idx_;
  // Single update structural gain from config (default 0 = frozen)
  const auto &su = config_.vehicle_tracker.ukf_v1.single_update;
  K.row(idx.R1()) *= su.structural_gain_r;
  K.row(idx.R2()) *= su.structural_gain_r;
  K.row(idx.DZA()) *= su.structural_gain_dza;

  Eigen::VectorXd x_post = ctx.x_prior + K * innov;
  x_post(idx.YAW()) = normalize_angle(x_post(idx.YAW()));
  Eigen::MatrixXd P_post = ctx.P_prior - K * S * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = 0;
  trial.last_k_post = 0;

  // Update the measured yaw-rate EMA (drives adaptive rotation noise): the
  // panel-relative center yaw is continuous across real panel switches, and
  // instantaneous values are clamped to bound wrong-panel outliers. Values
  // beyond |12| rad/s are dropped entirely instead of clamped: a late panel
  // switch makes center-yaw jump backward by ~90deg within one frame, which
  // clamps to +-15 and otherwise poisons the EMA (at 2rad/s true spin the EMA
  // wandered to ~0.5 and the adaptive bandwidth locked the yaw rate low);
  // legitimate spin changes are far slower per frame, so nothing real is lost.
  if (obs.timestamp.has_value()) {
    const double t = *obs.timestamp;
    if (last_obs_time_ > 0.0 && t > last_obs_time_) {
      const double inst_raw =
          angle_difference(center_yaw_obs, last_center_yaw_obs_) /
          (t - last_obs_time_);
      if (std::abs(inst_raw) <= 12.0) {
        const double inst = std::clamp(inst_raw, -15.0, 15.0);
        omega_meas_ema_ = 0.90 * omega_meas_ema_ + 0.10 * inst;
      }
      if (debug_omega_ema_ && (committed_updates_ % 15) == 0) {
        std::fprintf(stderr,
                     "[omega_ema] t=%.2f inst=%+.2f ema=%+.3f committed=%d "
                     "innov_yaw=%+.3f K_yr=%+.4f post_yr=%+.3f R_yaw=%.4f "
                     "P_yaw=%.5f P_yr=%.4f k=%d yaw=%+.3f obs_yaw=%+.3f\n",
                     t, inst_raw, omega_meas_ema_, committed_updates_,
                     innov(3), K(idx.YAW_RATE(), 3), x_post(idx.YAW_RATE()),
                     R(3, 3), ctx.P_prior(idx.YAW(), idx.YAW()),
                     ctx.P_prior(idx.YAW_RATE(), idx.YAW_RATE()), p,
                     x_post(idx.YAW()), center_yaw_obs);
      }
    }
    last_center_yaw_obs_ = center_yaw_obs;
    last_obs_time_ = t;
    ++committed_updates_;
  }

  // Reconstruction check
  trial.reconstruction_pos_error =
      compute_reconstruction_error(x_post, trial.k_post, obs, p);

  // Posterior sanity
  std::string posterior_reject_reason;
  trial.posterior_sanity_pass = check_posterior_sanity(
      ctx.x_prior, x_post, P_post, &posterior_reject_reason);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = posterior_reject_reason.empty()
        ? "posterior_sanity_fail"
        : posterior_reject_reason;
    trial.success = false;
  }

  const auto &ps = config_.vehicle_tracker.ukf_v1.posterior_sanity;
  trial.x_post = fyt::auto_aim::apply_state_constraints(
      trial.x_post, idx.R1(), idx.R2(), idx.DZA(), ps.min_r, ps.max_r,
      ps.min_dza, ps.max_dza);

  return trial;
}

UkfTrial VehicleUkfBackendV1::tryUpdateDual(
    const PredictContext &ctx, const ObservationData &obs0,
    const ObservationData &obs1, int panel_id_0, int panel_id_1) const {
  UkfTrial trial;
  trial.hypothesis.kind = HypothesisKind::Dual;
  trial.hypothesis.assignments[0] = {0, panel_id_0};
  trial.hypothesis.assignments[1] = {1, panel_id_1};
  trial.hypothesis.assignment_count = 2;

  MeasurementEval eval = evaluateDual(ctx, obs0, obs1, panel_id_0, panel_id_1);
  trial.eval = eval;
  if (!eval.valid) {
    trial.reject_reason = eval.reject_reason;
    return trial;
  }

  const int p0 = ((panel_id_0 % 4) + 4) % 4;
  const int p1 = ((panel_id_1 % 4) + 4) % 4;

  double center_yaw_obs0 = normalize_angle(obs0.yaw - p0 * (M_PI / 2.0));
  double center_yaw_obs1 = normalize_angle(obs1.yaw - p1 * (M_PI / 2.0));
  Eigen::Matrix<double, 8, 1> z_obs;
  z_obs << obs0.x, obs0.y, obs0.z, center_yaw_obs0,
           obs1.x, obs1.y, obs1.z, center_yaw_obs1;

  Eigen::MatrixXd sigma_pts;
  Eigen::VectorXd Wm, Wc;
  generate_sigma_points(ctx.x_prior, ctx.P_prior, sigma_pts, Wm, Wc);
  int n_sigma = sigma_pts.rows();
  int n = process_model_->state_dim();

  Eigen::MatrixXd z_pred_pts(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Vector4d z0 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p0);
    Eigen::Vector4d z1 =
        obs_model_single(sigma_pts.row(i).transpose(), ctx.k_prior, p1);
    z_pred_pts.row(i) << z0.transpose(), z1.transpose();
  }

  Eigen::Matrix<double, 8, 1> z_pred = Eigen::Matrix<double, 8, 1>::Zero();
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += Wm(i) * z_pred_pts.row(i).transpose();
  }
  z_pred(3) = weighted_angle_mean(z_pred_pts, Wm, 3);
  z_pred(7) = weighted_angle_mean(z_pred_pts, Wm, 7);

  Eigen::Matrix<double, 8, 1> innov = z_obs - z_pred;
  innov(3) = fold_flipped_yaw(angle_difference(z_obs(3), z_pred(3)),
                                  std::hypot(obs0.x, obs0.y));
  innov(7) = fold_flipped_yaw(angle_difference(z_obs(7), z_pred(7)),
                                  std::hypot(obs1.x, obs1.y));

  const auto &v1 = config_.vehicle_tracker.ukf_v1;
  double sp = v1.sigma_pos_xy;
  double sz = v1.sigma_pos_z;
  double sy = v1.sigma_yaw;
  double r_scale = v1.dual_raw_R_scale;
  Eigen::Matrix<double, 8, 8> R = dual_measurement_covariance(
      obs0, obs1, sp, sz, sy, r_scale);

  Eigen::MatrixXd diff_z(n_sigma, 8);
  for (int i = 0; i < n_sigma; ++i) {
    diff_z.row(i) = z_pred_pts.row(i) - z_pred.transpose();
    diff_z(i, 3) = angle_difference(z_pred_pts(i, 3), z_pred(3));
    diff_z(i, 7) = angle_difference(z_pred_pts(i, 7), z_pred(7));
  }

  Eigen::Matrix<double, 8, 8> Pzz = R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n, 8);
  for (int i = 0; i < n_sigma; ++i) {
    Eigen::Matrix<double, 8, 1> dz = diff_z.row(i).transpose();
    Pzz += Wc(i) * dz * dz.transpose();
    Eigen::VectorXd dx = sigma_pts.row(i).transpose() - ctx.x_prior;
    Pxz += Wc(i) * dx * dz.transpose();
  }

  Eigen::Matrix<double, 8, 8> S = Pzz;
  Eigen::LLT<Eigen::Matrix<double, 8, 8>> llt(S);
  if (llt.info() != Eigen::Success) {
    trial.reject_reason = "S_not_spd_in_tryUpdate_dual";
    return trial;
  }

  Eigen::Matrix<double, 8, 8> S_inv = S.inverse();
  Eigen::MatrixXd K = Pxz * S_inv;

  // Conservative structural gain for dual: small but non-zero
  auto idx = state_idx_;
  const auto &du = config_.vehicle_tracker.ukf_v1.dual_update;
  K.row(idx.R1()) *= du.structural_gain_r;
  K.row(idx.R2()) *= du.structural_gain_r;
  K.row(idx.DZA()) *= du.structural_gain_dza;

  Eigen::VectorXd x_post = ctx.x_prior + K * innov;
  x_post(idx.YAW()) = normalize_angle(x_post(idx.YAW()));
  Eigen::MatrixXd P_post = ctx.P_prior - K * S * K.transpose();
  P_post = 0.5 * (P_post + P_post.transpose());
  P_post = ensure_positive_definite(P_post, 1e-6);

  trial.success = true;
  trial.x_post = x_post;
  trial.P_post = P_post;
  trial.k_post = 0;
  trial.last_k_post = 0;

  double recon0 =
      compute_reconstruction_error(x_post, trial.k_post, obs0, p0);
  double recon1 =
      compute_reconstruction_error(x_post, trial.k_post, obs1, p1);
  trial.reconstruction_pos_error = std::max(recon0, recon1);

  std::string posterior_reject_reason;
  trial.posterior_sanity_pass = check_posterior_sanity(
      ctx.x_prior, x_post, P_post, &posterior_reject_reason);

  if (!trial.posterior_sanity_pass) {
    trial.reject_reason = posterior_reject_reason.empty()
        ? "posterior_sanity_fail"
        : posterior_reject_reason;
    trial.success = false;
  }

  const auto &ps = config_.vehicle_tracker.ukf_v1.posterior_sanity;
  trial.x_post = fyt::auto_aim::apply_state_constraints(
      trial.x_post, idx.R1(), idx.R2(), idx.DZA(), ps.min_r, ps.max_r,
      ps.min_dza, ps.max_dza);

  return trial;
}

void VehicleUkfBackendV1::commit(const UkfTrial &trial) {
  if (!trial.success) return;

  x_ = trial.x_post;
  P_ = trial.P_post;
  k_ = trial.k_post;
  last_k_ = trial.last_k_post;

  if (trial.hypothesis.kind == HypothesisKind::Single) {
    current_panel_id_ = trial.hypothesis.assignments[0].panel_id;
  } else {
    current_panel_id_ = trial.hypothesis.assignments[0].panel_id;
  }

  last_nis_ = trial.eval.nis;
  if (trial.eval.innovation.size() >= 3) {
    last_innov_xyz_ = trial.eval.innovation.head<3>();
  }
  if (trial.eval.innovation.size() >= 4) {
    last_innov_yaw_ = trial.eval.innovation(3);
  }
  last_update_type_ =
      (trial.hypothesis.kind == HypothesisKind::Single) ? 1 : 2;

  apply_state_constraints();
  P_ = ensure_positive_definite(P_);
}

void VehicleUkfBackendV1::noteArmorAngle(double angle_rad, double timestamp) {
  if (!initialized_) return;
  if (last_armor_angle_time_ > 0.0 && timestamp > last_armor_angle_time_) {
    const double dt = timestamp - last_armor_angle_time_;
    const double delta = angle_difference(angle_rad, last_armor_angle_);
    // Plate-to-plate jumps are +-90deg; the per-frame rotation of anything
    // physically relevant is < 45deg.
    if (std::abs(delta) < 0.7 && dt > 1e-3) {
      const double inst = std::clamp(delta / dt, -20.0, 20.0);
      const double witness_alpha = std::clamp(
          config_.vehicle_tracker.ukf_v1.rotation_witness_ema_alpha,
          0.01, 1.0);
      if (armor_angle_valid_streak_ < 5) {
        const double sample_count = armor_angle_valid_streak_ + 1.0;
        armor_angle_ema_ += (inst - armor_angle_ema_) / sample_count;
        armor_angle_magnitude_ema_ +=
            (std::abs(inst) - armor_angle_magnitude_ema_) / sample_count;
      } else {
        armor_angle_ema_ =
            (1.0 - witness_alpha) * armor_angle_ema_ + witness_alpha * inst;
        armor_angle_magnitude_ema_ =
            (1.0 - witness_alpha) * armor_angle_magnitude_ema_ +
            witness_alpha * std::abs(inst);
      }
      ++armor_angle_samples_;
      ++armor_angle_valid_streak_;
      if (debug_omega_ema_ && (armor_angle_samples_ % 30) == 0) {
        std::fprintf(stderr,
                     "[angle_witness] t=%.2f inst=%+.2f ema=%+.2f state_yr=%+.2f\n",
                     timestamp, inst, armor_angle_ema_, x_(state_idx_.YAW_RATE()));
      }
      // Symmetry-trap break: the radial-yaw witness disagrees with the filtered
      // yaw rate. Requires enough samples for the EMA to be meaningful and a
      // clearly non-trivial rate (static-target angle jitter from center
      // noise averages near zero).
      const int yr = state_idx_.YAW_RATE();
      const bool direction_conflict =
          std::abs(x_(yr)) > 2.0 && armor_angle_ema_ * x_(yr) < 0.0;
      const bool missing_rate =
          std::abs(armor_angle_ema_) - std::abs(x_(yr)) > 2.0;
      const double direction_coherence =
          std::abs(armor_angle_ema_) /
          std::max(armor_angle_magnitude_ema_, 1e-6);
      if (armor_angle_valid_streak_ >= 5 &&
          std::abs(armor_angle_ema_) > 2.0 &&
          direction_coherence > 0.70 &&
          (direction_conflict || missing_rate)) {
        const double clamped = std::clamp(armor_angle_ema_, -15.0, 15.0);
        x_(yr) = 0.5 * x_(yr) + 0.5 * clamped;
        P_(yr, yr) = std::max(P_(yr, yr), 4.0);
        omega_meas_ema_ = 0.5 * omega_meas_ema_ + 0.5 * clamped;
      }
    } else {
      // A +-90deg plate switch invalidates the finite difference. Do not
      // carry a stale high-rate witness across the switch: after a real spin
      // stops, the next valid same-panel sample must not resurrect it.
      armor_angle_ema_ = 0.0;
      armor_angle_magnitude_ema_ = 0.0;
      armor_angle_valid_streak_ = 0;
    }
  }
  last_armor_angle_ = angle_rad;
  last_armor_angle_time_ = timestamp;
}

void VehicleUkfBackendV1::resetArmorAngleWitness() {
  last_armor_angle_time_ = -1.0;
  armor_angle_ema_ = 0.0;
  armor_angle_magnitude_ema_ = 0.0;
  armor_angle_valid_streak_ = 0;
}

void VehicleUkfBackendV1::noteDualHeightEvidence(
    double half_height_difference) {
  if (!initialized_ || !std::isfinite(half_height_difference)) return;
  const auto &v1 = config_.vehicle_tracker.ukf_v1;
  const auto &sanity = v1.posterior_sanity;
  const double evidence = std::clamp(
      half_height_difference, sanity.min_dza, sanity.max_dza);

  dual_height_evidence_.push_back(evidence);
  const size_t window =
      static_cast<size_t>(std::max(3, v1.dual_height_evidence_window));
  while (dual_height_evidence_.size() > window) {
    dual_height_evidence_.pop_front();
  }
  if (dual_height_evidence_.size() < static_cast<size_t>(
          std::max(1, v1.dual_height_evidence_min_samples))) {
    return;
  }

  std::vector<double> sorted(
      dual_height_evidence_.begin(), dual_height_evidence_.end());
  const auto middle = sorted.begin() + sorted.size() / 2;
  std::nth_element(sorted.begin(), middle, sorted.end());
  const double median = *middle;
  const int dza_index = state_idx_.DZA();
  const double gain = std::clamp(v1.dual_height_evidence_gain, 0.0, 1.0);
  const double correction = std::clamp(
      gain * (median - x_(dza_index)), -sanity.max_dza_jump,
      sanity.max_dza_jump);
  x_(dza_index) = std::clamp(
      x_(dza_index) + correction, sanity.min_dza, sanity.max_dza);
}

// ── SpinFilterInterface queries ──

Eigen::Vector3d VehicleUkfBackendV1::get_center_position() const {
  auto idx = state_idx_;
  return Eigen::Vector3d(x_(idx.X()), x_(idx.Y()), x_(idx.Z()));
}

std::pair<double, double> VehicleUkfBackendV1::get_radii() const {
  auto idx = state_idx_;
  return {x_(idx.R1()), x_(idx.R2())};
}

double VehicleUkfBackendV1::get_dza() const {
  return x_(state_idx_.DZA());
}

double VehicleUkfBackendV1::get_yaw() const {
  return normalize_angle(x_(state_idx_.YAW()));
}

double VehicleUkfBackendV1::get_raw_yaw() const {
  return x_(state_idx_.YAW());
}

// ── Private helpers ──

bool VehicleUkfBackendV1::check_posterior_sanity(
    const Eigen::VectorXd &x_prior, const Eigen::VectorXd &x_post,
    const Eigen::MatrixXd &P_post, std::string *reject_reason) const {
  auto idx = state_idx_;
  const auto &ps = config_.vehicle_tracker.ukf_v1.posterior_sanity;
  const auto reject = [reject_reason](const std::string &reason) {
    if (reject_reason != nullptr) *reject_reason = reason;
    return false;
  };

  // Center jump check (threshold scales with range: distant PnP center
  // jitter legitimately exceeds the 2m-tuned limit — see
  // distance_noise_factors).
  Eigen::Vector3d prior_center(x_prior(idx.X()), x_prior(idx.Y()),
                                x_prior(idx.Z()));
  Eigen::Vector3d post_center(x_post(idx.X()), x_post(idx.Y()),
                               x_post(idx.Z()));
  const double range = prior_center.head<2>().norm();
  const double lat = std::clamp(range / 2.0, 1.0, 3.0);
  double center_jump = (post_center - prior_center).norm();
  const double center_jump_limit = ps.max_center_jump * lat;
  if (center_jump > center_jump_limit) {
    return reject("posterior_center_jump=" + std::to_string(center_jump) +
                  ">" + std::to_string(center_jump_limit));
  }
  const double vertical_center_jump =
      std::abs(post_center.z() - prior_center.z());
  if (vertical_center_jump > ps.max_vertical_center_jump) {
    return reject(
        "posterior_vertical_center_jump=" +
        std::to_string(vertical_center_jump) + ">" +
        std::to_string(ps.max_vertical_center_jump));
  }

  // Yaw/delta jump check
  double delta_jump = std::abs(angle_difference(x_post(idx.YAW()),
                                                  x_prior(idx.YAW())));
  if (delta_jump > ps.max_yaw_jump) {
    return reject("posterior_yaw_jump=" + std::to_string(delta_jump) +
                  ">" + std::to_string(ps.max_yaw_jump));
  }

  // Radius range check
  double r1 = x_post(idx.R1());
  double r2 = x_post(idx.R2());
  if (r1 < ps.min_r || r1 > ps.max_r || r2 < ps.min_r || r2 > ps.max_r) {
    return reject("posterior_radius_range:r1=" + std::to_string(r1) +
                  ",r2=" + std::to_string(r2));
  }

  // Radius jump check
  const double r1_jump = std::abs(r1 - x_prior(idx.R1()));
  const double r2_jump = std::abs(r2 - x_prior(idx.R2()));
  if (r1_jump > ps.max_r_jump || r2_jump > ps.max_r_jump) {
    return reject("posterior_radius_jump:r1=" + std::to_string(r1_jump) +
                  ",r2=" + std::to_string(r2_jump) +
                  ",limit=" + std::to_string(ps.max_r_jump));
  }

  // DZA range check
  double dza = x_post(idx.DZA());
  if (dza < ps.min_dza || dza > ps.max_dza) {
    return reject("posterior_dza_range=" + std::to_string(dza));
  }
  const double dza_jump = std::abs(dza - x_prior(idx.DZA()));
  if (dza_jump > ps.max_dza_jump) {
    return reject("posterior_dza_jump=" + std::to_string(dza_jump) +
                  ">" + std::to_string(ps.max_dza_jump));
  }

  // P positive semi-definite
  if (!P_post.allFinite()) return reject("posterior_covariance_non_finite");

  if (reject_reason != nullptr) reject_reason->clear();
  return true;
}

double VehicleUkfBackendV1::compute_reconstruction_error(
    const Eigen::VectorXd &x_post, int k, const ObservationData &obs,
    int panel_id) const {
  Eigen::Vector4d z_rebuild = obs_model_single(x_post, k, panel_id);
  Eigen::Vector3d pos_rebuild = z_rebuild.head<3>();
  Eigen::Vector3d pos_obs(obs.x, obs.y, obs.z);
  return (pos_obs - pos_rebuild).norm();
}

void VehicleUkfBackendV1::apply_state_constraints() {
  auto idx = state_idx_;
  x_ = fyt::auto_aim::apply_state_constraints(
      x_, idx.R1(), idx.R2(), idx.DZA(), config_.constraints.min_radius,
      config_.constraints.max_radius, config_.constraints.min_dz,
      config_.constraints.max_dz);
  const auto &v1 = config_.vehicle_tracker.ukf_v1;
  x_(idx.VZ()) = std::clamp(
      x_(idx.VZ()), -v1.max_vertical_speed, v1.max_vertical_speed);
  if (idx.has("AZ")) {
    x_(idx.AZ()) = std::clamp(
        x_(idx.AZ()), -v1.max_vertical_acceleration,
        v1.max_vertical_acceleration);
  }
  x_(idx.YAW()) = normalize_angle(x_(idx.YAW()));
}

}  // namespace fyt::auto_aim::vehicle
