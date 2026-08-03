// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_NORM4_UKF_BACKEND_V1_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_NORM4_UKF_BACKEND_V1_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <cstdlib>
#include <string>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"
#include "max_entropy_tracker/trackers/vehicle/hypothesis/vehicle_hypothesis_types.hpp"
#include "max_entropy_tracker/utils/sigma_points.hpp"

namespace fyt::auto_aim::vehicle {

class VehicleUkfBackendV1 : public SpinFilterInterface {
 public:
  explicit VehicleUkfBackendV1(const UnifiedConfig &config, double dt = 0.05);

  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza);
  void predict(double dt);
  bool initialized() const { return initialized_; }

  PredictContext buildPredictContext() const;

  MeasurementEval evaluateSingle(const PredictContext &ctx,
                                  const ObservationData &obs,
                                  int panel_id) const;

  MeasurementEval evaluateDual(const PredictContext &ctx,
                                const ObservationData &obs0,
                                const ObservationData &obs1,
                                int panel_id_0, int panel_id_1) const;

  UkfTrial tryUpdateSingle(const PredictContext &ctx,
                            const ObservationData &obs,
                            int panel_id) const;

  UkfTrial tryUpdateDual(const PredictContext &ctx,
                          const ObservationData &obs0,
                          const ObservationData &obs1,
                          int panel_id_0, int panel_id_1) const;

  void commit(const UkfTrial &trial);

  // Translation-independent rotation witness fed with one observed armor's
  // radial yaw per frame (no panel semantics): at spin rates up
  // to ~15 rad/s the per-frame bearing advance is < 45deg, so the finite
  // difference is unambiguous even when the detection alternates between
  // physical plates (their +/-90deg steps are discarded). It stays valid when
  // center-position error would corrupt a center-to-armor bearing. When the
  // witness persistently disagrees with the filtered yaw rate, the state is
  // nudged toward it.
  void noteArmorAngle(double angle_rad, double timestamp);
  void resetArmorAngleWitness();
  void noteDualHeightEvidence(double half_height_difference);
  double armor_angle_rate() const { return armor_angle_ema_; }

  // ── SpinFilterInterface ──
  const Eigen::VectorXd &x() const override { return x_; }
  Eigen::VectorXd &x() override { return x_; }
  const Eigen::MatrixXd &P() const override { return P_; }
  Eigen::MatrixXd &P() override { return P_; }
  const DynamicStateIndex &state_idx() const override { return state_idx_; }

  Eigen::Vector3d get_center_position() const override;
  std::pair<double, double> get_radii() const override;
  double get_dza() const override;
  double get_yaw() const override;
  double get_raw_yaw() const override;
  int get_k() const override { return k_; }

  const Eigen::VectorXd &last_innov_xyz() const override { return last_innov_xyz_; }
  double last_innov_yaw() const override { return last_innov_yaw_; }
  double last_nis() const override { return last_nis_; }
  int last_update_type() const override { return last_update_type_; }

 private:
  static std::string r_type_for_panel(int panel_id);
  static std::string armor_layer_for_panel(int panel_id);

  Eigen::Vector4d obs_model_single(const Eigen::VectorXd &x, int k,
                                    int panel_id) const;

  void generate_sigma_points(const Eigen::VectorXd &x,
                              const Eigen::MatrixXd &P,
                              Eigen::MatrixXd &out_sigma_pts,
                              Eigen::VectorXd &out_Wm,
                              Eigen::VectorXd &out_Wc) const;

  Eigen::Vector4d compute_z_pred_and_S(
      const Eigen::MatrixXd &sigma_pts,
      const Eigen::VectorXd &Wm,
      const Eigen::VectorXd &Wc,
      double panel_angle,
      MeasurementEval *out_eval) const;

  bool check_posterior_sanity(const Eigen::VectorXd &x_prior,
                               const Eigen::VectorXd &x_post,
                               const Eigen::MatrixXd &P_post,
                               std::string *reject_reason = nullptr) const;

  double compute_reconstruction_error(
      const Eigen::VectorXd &x_post, int k,
      const ObservationData &obs, int panel_id) const;

  void apply_state_constraints();

  UnifiedConfig config_;
  double dt_ = 0.05;
  std::shared_ptr<CompositeProcessModel> process_model_;
  DynamicStateIndex state_idx_;

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  bool initialized_ = false;

  int k_ = 0;
  int last_k_ = 0;
  int current_panel_id_ = -1;

  Eigen::VectorXd last_innov_xyz_;
  double last_innov_yaw_ = 0.0;
  double last_nis_ = -1.0;
  int last_update_type_ = 0;

  // Measured yaw-rate EMA driving the adaptive rotation process noise (see
  // predict()): finite-differenced center-yaw observations give a
  // static-vs-spin discriminator that does not depend on the (jitter-
  // inflated) state covariance. Updated in const tryUpdateSingle, hence mutable.
  mutable double last_center_yaw_obs_ = 0.0;
  mutable double last_obs_time_ = -1.0;
  mutable double omega_meas_ema_ = 0.0;
  mutable int committed_updates_ = 0;
  // stderr tracing of the EMA when HFUT_DEBUG_OMEGA_EMA is set.
  const bool debug_omega_ema_ = std::getenv("HFUT_DEBUG_OMEGA_EMA") != nullptr;
  mutable int gate_fail_prints_ = 0;
  // Model-free armor-bearing witness state (see noteArmorAngle).
  double last_armor_angle_ = 0.0;
  double last_armor_angle_time_ = -1.0;
  double armor_angle_ema_ = 0.0;
  double armor_angle_magnitude_ema_ = 0.0;
  int armor_angle_samples_ = 0;
  int armor_angle_valid_streak_ = 0;
  std::deque<double> dual_height_evidence_;

  mutable std::unique_ptr<SigmaPointGenerator> sigma_gen_;
};

}  // namespace fyt::auto_aim::vehicle

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_NORM4_UKF_BACKEND_V1_HPP_
