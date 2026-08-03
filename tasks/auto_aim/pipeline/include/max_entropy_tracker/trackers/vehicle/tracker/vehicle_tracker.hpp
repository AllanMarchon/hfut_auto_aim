// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_NORM4_TRACKER_V2_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_NORM4_TRACKER_V2_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/evidence/evidence_builder.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/trackers/vehicle/hypothesis/vehicle_hypothesis_generator.hpp"
#include "max_entropy_tracker/trackers/vehicle/hypothesis/vehicle_hypothesis_types.hpp"
#include "max_entropy_tracker/trackers/vehicle/interfaces/vehicle_backend_interface.hpp"
#include "max_entropy_tracker/trackers/vehicle/backends/vehicle_backend_factory.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

class VehicleArmorTracker : public BaseTracker {
 public:
  explicit VehicleArmorTracker(const UnifiedConfig &config, double dt = 0.05,
                               bool enable_oscillation = false);

  void initialize(const std::vector<ObservationData> &obs, double r1 = 0.15,
                  double r2 = 0.20, double dza = 0.0) override;
  void predict(std::optional<double> target_time = std::nullopt) override;
  bool update(const std::vector<ObservationData> &obs) override;

  Eigen::Vector3d get_center_position() const override;
  double get_yaw() const override;
  std::pair<double, double> get_radii() const override;
  SpinFilterInterface &spin_filter() override;
  const SpinFilterInterface &spin_filter() const override;
  ManeuverResult assess_maneuver() const override;

  Eigen::Vector3d get_publish_velocity() const override;
  bool is_ambiguous_single_mode() const override;
  bool supports_ambiguous_single_semantics() const override { return false; }
  int effective_num_armors() const override;
  double confidence_scale() const override;
  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message() const override;

  /// Gated tracker: frames rejected by NIS/posterior/reconstruction gates do
  /// not touch the filter, so report them as not committed.
  bool last_update_committed() const override { return last_update_committed_; }

  const vehicle::HypothesisDebugFrame &last_hypothesis_debug() const {
    return last_hypothesis_debug_;
  }

  const vehicle::VehicleDebugSnapshot &debug_snapshot() const { return debug_snapshot_; }
  const evidence::ArmorEvidenceFrame &last_evidence_frame() const { return evidence_frame_; }
  vehicle::VehicleTrackerMode current_mode() const { return mode_; }

 private:
  void populate_debug_snapshot();

  // Warmup (0/1 dual-seed)
  void init_warmup(const std::vector<ObservationData> &obs, double r1, double r2, double dza);
  bool run_warmup(const std::vector<ObservationData> &obs);
  void promote_warmup_winner();

  // Mode routing
  void set_mode(vehicle::VehicleTrackerMode m);
  void apply_mode_routing();
  void select_topk(std::vector<vehicle::MeasurementEval> &evals,
                   const std::vector<vehicle::Hypothesis> &hyps,
                   int topk_count,
                   std::vector<vehicle::TopKEntry> *topk_out,
                   double *confidence_out, double *margin_out) const;

  UnifiedConfig config_;
  std::unique_ptr<vehicle::IStructuredBackend> backend_;
  vehicle::HypothesisGenerator hypothesis_generator_;
  ManeuverDetector maneuver_detector_;

  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;
  // Nominal structure captured at the first initialize() (robot-description
  // defaults). The reject-streak reset falls back to it when the streak is
  // dominated by reconstruction rejects — those mean the *learned* structure
  // no longer fits the observations, so preserving it would re-seed the very
  // corruption that caused the streak.
  double nominal_r1_ = 0.15;
  double nominal_r2_ = 0.20;
  double nominal_dza_ = 0.0;
  bool nominal_structure_set_ = false;
  std::optional<ObservationData> warmup_last_obs_;
  std::optional<int> rotation_witness_track2d_id_;

  int current_panel_id_ = -1;
  vehicle::VehicleTrackerMode mode_ = vehicle::VehicleTrackerMode::STRUCTURED;
  vehicle::WarmupState warmup_state_{};
  bool last_update_committed_ = false;
  // Consecutive gate-rejected frames; drives the sp-style reject-streak reset.
  int consecutive_rejects_ = 0;
  // Rejects within the current streak whose reason is a reconstruction-error
  // gate trip (structure suspect), as opposed to plain NIS/gate misses.
  int reconstruction_reject_streak_ = 0;

  vehicle::HypothesisDebugFrame last_hypothesis_debug_{};
  vehicle::VehicleDebugSnapshot debug_snapshot_{};
  evidence::ArmorEvidenceFrame evidence_frame_{};
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_VEHICLE_NORM4_TRACKER_V2_HPP_
