// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/vehicle/tracker/vehicle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include "max_entropy_tracker/utils/dual_height_evidence.hpp"

namespace fyt::auto_aim {

namespace {

struct StructureEstimate {
  int seed_index = 0;
  double r1 = 0.0;
  double r2 = 0.0;
  double dza = 0.0;
};

std::optional<StructureEstimate> estimateStructure(
    const std::vector<ObservationData> &obs) {
  if (obs.size() < 4) return std::nullopt;

  constexpr int kPairings[3][4] = {
      {0, 1, 2, 3}, {0, 2, 1, 3}, {0, 3, 1, 2}};
  int best_pairing = 0;
  double best_center_error = std::numeric_limits<double>::infinity();
  for (int pairing = 0; pairing < 3; ++pairing) {
    const auto &p = kPairings[pairing];
    Eigen::Vector3d c0(
        0.5 * (obs[p[0]].x + obs[p[1]].x),
        0.5 * (obs[p[0]].y + obs[p[1]].y),
        0.5 * (obs[p[0]].z + obs[p[1]].z));
    Eigen::Vector3d c1(
        0.5 * (obs[p[2]].x + obs[p[3]].x),
        0.5 * (obs[p[2]].y + obs[p[3]].y),
        0.5 * (obs[p[2]].z + obs[p[3]].z));
    const double center_error = (c0 - c1).head<2>().norm();
    if (center_error < best_center_error) {
      best_center_error = center_error;
      best_pairing = pairing;
    }
  }

  const auto &p = kPairings[best_pairing];
  const auto radius = [&obs](int i0, int i1) {
    return 0.5 * std::hypot(obs[i0].x - obs[i1].x,
                            obs[i0].y - obs[i1].y);
  };
  const double r_a = radius(p[0], p[1]);
  const double r_b = radius(p[2], p[3]);
  const double z_a = 0.5 * (obs[p[0]].z + obs[p[1]].z);
  const double z_b = 0.5 * (obs[p[2]].z + obs[p[3]].z);
  if (!std::isfinite(r_a) || !std::isfinite(r_b) ||
      r_a < 0.05 || r_b < 0.05 || r_a > 0.50 || r_b > 0.50 ||
      !std::isfinite(z_a) || !std::isfinite(z_b)) {
    return std::nullopt;
  }

  StructureEstimate estimate;
  if (z_a <= z_b) {
    estimate.r1 = r_a;
    estimate.r2 = r_b;
    estimate.seed_index = p[0];
  } else {
    estimate.r1 = r_b;
    estimate.r2 = r_a;
    estimate.seed_index = p[2];
  }
  estimate.dza = std::clamp(0.5 * std::abs(z_a - z_b), 0.0, 0.15);
  return estimate;
}

}  // namespace

VehicleArmorTracker::VehicleArmorTracker(const UnifiedConfig &config, double dt,
                                         bool /*enable_oscillation*/)
    : BaseTracker(dt),
      config_(config),
      backend_(vehicle::create_backend(
          vehicle::backend_type_from_string(
              config_.vehicle_tracker.backend_config.backend_type),
          config, dt)),
      maneuver_detector_(config.maneuver) {}


void VehicleArmorTracker::initialize(const std::vector<ObservationData> &obs,
                                     double r1, double r2, double dza) {
  if (obs.empty())
    throw std::invalid_argument("At least one observation required");

  if (!nominal_structure_set_) {
    nominal_r1_ = r1;
    nominal_r2_ = r2;
    nominal_dza_ = dza;
    nominal_structure_set_ = true;
  }
  default_r1_ = r1;
  default_r2_ = r2;
  default_dza_ = dza;
  const auto structure = estimateStructure(obs);
  const int seed_index = structure ? structure->seed_index : 0;
  if (structure) {
    default_r1_ = structure->r1;
    default_r2_ = structure->r2;
    default_dza_ = structure->dza;
  }
  warmup_last_obs_ = obs[seed_index];
  rotation_witness_track2d_id_ = obs[seed_index].track2d_id;

  int init_panel = structure ? 0 : obs[seed_index].panel_id.value_or(0);
  backend_->reset(obs[seed_index], init_panel, default_r1_, default_r2_, default_dza_);
  current_panel_id_ = init_panel;
  warmup_state_.active = false;
  set_mode(vehicle::VehicleTrackerMode::STRUCTURED);

  transition_to(TrackerState::INITIALIZING);
  mark_initialized();
  update_time(obs[seed_index].timestamp.value_or(0.0));
}

void VehicleArmorTracker::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;
  double dt = compute_dt(target_time);
  if (dt <= 0.0) return;
  backend_->predict(dt);
  if (target_time.has_value()) update_time(target_time.value());
}

bool VehicleArmorTracker::update(const std::vector<ObservationData> &obs) {
  if (!is_initialized() || obs.empty()) {
    last_update_committed_ = false;
    handle_observation_loss(config_.tracker.tracking_thres,
                            config_.tracker.lost_thres);
    return false;
  }

  double obs_ts =
      obs[0].timestamp.value_or(current_time_.value_or(0.0));

  // Follow one detector track while it remains visible. The NN may return two
  // plates in a different container order on adjacent frames; selecting
  // obs.front() would then inject repeated +/-pi/2 switches and prevent the
  // rotation witness from ever accumulating at speed.
  const ObservationData* witness = nullptr;
  if (rotation_witness_track2d_id_.has_value()) {
    const auto match = std::find_if(
        obs.begin(), obs.end(), [this](const ObservationData& observation) {
          return observation.track2d_id == rotation_witness_track2d_id_;
        });
    if (match != obs.end()) witness = &*match;
  }
  if (witness == nullptr) {
    const auto tracked = std::find_if(
        obs.begin(), obs.end(), [](const ObservationData& observation) {
          return observation.track2d_id.has_value();
        });
    witness = tracked != obs.end() ? &*tracked : &obs.front();
    rotation_witness_track2d_id_ = witness->track2d_id;
  }
  const bool planar_pnp = witness->ba_pnp.has_value() &&
      witness->ba_pnp->valid && witness->ba_pnp->pose_estimate_mode != 0;
  const double view_direction = std::atan2(witness->y, witness->x) + M_PI;
  const double facing_angle = std::abs(std::remainder(
      witness->yaw - view_direction, 2.0 * M_PI));
  if (planar_pnp && facing_angle < 0.45) {
    backend_->resetArmorAngleWitness();
  } else {
    backend_->noteArmorAngle(witness->yaw, obs_ts);
  }

  // Predict to observation time.
  if (current_time_.has_value() && obs_ts > current_time_.value()) {
    backend_->predict(obs_ts - current_time_.value());
  }

  // The same-frame height difference is independent of center Z motion and
  // gives dza a dedicated observable channel whenever adjacent plates appear.
  if (const auto evidence = selectDualHeightEvidence(
          obs, backend_->spin_filter().get_yaw())) {
    backend_->noteDualHeightEvidence(*evidence);
  }

  // Legacy warmup is kept as a defensive path for old replay configs, but normal
  // runtime initializes directly into the unified structured hypothesis pipeline.
  if (warmup_state_.active) {
    bool warmup_ok = run_warmup(obs);
    update_time(obs_ts);
    increment_frame();
    last_update_committed_ = warmup_ok;
    if (warmup_ok) {
      handle_observation_received(config_.tracker.tracking_thres);
    } else {
      handle_observation_loss(config_.tracker.tracking_thres,
                              config_.tracker.lost_thres);
    }

    last_hypothesis_debug_.valid = true;
    last_hypothesis_debug_.obs_count = static_cast<int>(obs.size());
    last_hypothesis_debug_.committed = false;
    last_hypothesis_debug_.degraded = true;
    last_hypothesis_debug_.decision_reason = warmup_state_.warmup_reason;

    populate_debug_snapshot();
    return warmup_ok;
  }

  // Build prior snapshot — all hypotheses evaluated from this single prior.
  auto ctx = backend_->buildPredictContext();

  // Generate all regular hypotheses for this frame. Single and multi-armor
  // observations share the same evaluate -> gate -> commit path; multi-armor
  // frames simply add dual hypotheses to the candidate set.
  auto hypotheses = hypothesis_generator_.generate(obs);
  hypothesis_generator_.attach_prior(&hypotheses);

  // Evaluate all hypotheses from the same prior.
  std::vector<vehicle::MeasurementEval> evals;
  evals.reserve(hypotheses.size());

  for (const auto &hyp : hypotheses) {
    vehicle::MeasurementEval eval;
    if (hyp.kind == vehicle::HypothesisKind::Single) {
      int panel = hyp.assignments[0].panel_id;
      eval = backend_->evaluateSingle(ctx, obs[hyp.assignments[0].obs_index],
                                       panel);
    } else {
      int p0 = hyp.assignments[0].panel_id;
      int p1 = hyp.assignments[1].panel_id;
      eval = backend_->evaluateDual(ctx, obs[hyp.assignments[0].obs_index],
                                     obs[hyp.assignments[1].obs_index], p0, p1);
    }
    // Combine prior log weight into score.
    const double obs_dim =
        (hyp.kind == vehicle::HypothesisKind::Single) ? 4.0 : 8.0;
    eval.score = (eval.log_likelihood / obs_dim) + hyp.prior_log_weight;
    evals.push_back(eval);
  }

  // Select TopK, compute confidence.
  const auto &sel_cfg = config_.vehicle_tracker.hypothesis_selector;
  int topk_count = std::max(1, sel_cfg.topk);
  std::vector<vehicle::TopKEntry> topk;
  double top1_confidence = 0.0, top1_top2_margin = 0.0;
  select_topk(evals, hypotheses, topk_count, &topk,
              &top1_confidence, &top1_top2_margin);

  bool committed = false;
  std::string decision_reason;

  // Find the first gate-passing hypothesis in ranked order.
  int best_idx = -1;
  for (size_t i = 0; i < topk.size(); ++i) {
    if (topk[i].eval.gate_pass && topk[i].eval.valid) {
      best_idx = static_cast<int>(i);
      break;
    }
  }

  // Re-acquisition leniency: while not firmly TRACKING, accept the best
  // hypothesis when it only fails the gate by a bounded margin. Distant
  // vision targets produce innovations (reversal overshoot, yaw garbage at
  // range) that no fixed gate tolerates, so the track otherwise bounces
  // between all_gate_fail and re-init forever without reaching TRACKING.
  if (best_idx < 0 && state() != TrackerState::TRACKING) {    const auto &gt = config_.vehicle_tracker.ukf_v1.gate;
    const double relax = std::max(1.0, gt.init_relax);
    for (size_t i = 0; i < topk.size(); ++i) {
      const auto &e = topk[i].eval;
      if (!e.valid) continue;
      const bool single =
          topk[i].hypothesis.kind == vehicle::HypothesisKind::Single;
      const double nis_thr =
          (single ? gt.single_total_nis : gt.dual_total_nis) * relax;
      const double pos_thr =
          (single ? gt.single_pos_chi2 : gt.dual_each_pos_chi2) * relax;
      const double yaw_thr =
          (single ? gt.single_yaw_chi2 : gt.dual_each_yaw_chi2) * relax;
      if (e.nis <= nis_thr && e.chi2_pos <= pos_thr &&
          e.chi2_yaw <= yaw_thr) {
        best_idx = static_cast<int>(i);
        break;
      }
    }
  }

  // Persistent-conflict leniency: a single strict-gate failure is rejected as
  // a possibly anomalous observation, but when the strict gate fails again on
  // the next frame while the relaxed (init_relax) gate passes, the conflict
  // is persistent — the prediction has diverged, not the observation. Trust
  // the observation and commit the top1 hypothesis instead of letting a
  // crooked predictor reject accurate observations until the reject-streak
  // reset fires.
  if (best_idx < 0 && state() == TrackerState::TRACKING &&
      consecutive_rejects_ >= 1 && !topk.empty() && topk[0].eval.valid) {
    const auto &gt = config_.vehicle_tracker.ukf_v1.gate;
    const double relax = std::max(1.0, gt.init_relax);
    const auto &e = topk[0].eval;
    const bool single =
        topk[0].hypothesis.kind == vehicle::HypothesisKind::Single;
    const double nis_thr =
        (single ? gt.single_total_nis : gt.dual_total_nis) * relax;
    const double pos_thr =
        (single ? gt.single_pos_chi2 : gt.dual_each_pos_chi2) * relax;
    const double yaw_thr =
        (single ? gt.single_yaw_chi2 : gt.dual_each_yaw_chi2) * relax;
    if (e.nis <= nis_thr && e.chi2_pos <= pos_thr && e.chi2_yaw <= yaw_thr) {
      best_idx = 0;
    }
  }

  if (mode_ != vehicle::VehicleTrackerMode::STRUCTURED) {
    set_mode(vehicle::VehicleTrackerMode::STRUCTURED);
  }
  // Panel-switch hysteresis: near a panel boundary the adjacent-panel
  // assignment fits almost as well as the current one, and plain top1
  // selection flickers between them. Each flip re-anchors the plate 90deg
  // away and the filter absorbs the implied jump as phantom whole-vehicle
  // translation. Keep the previous panel unless the switching hypothesis
  // beats the stay hypothesis by a clear score margin; a genuine plate
  // switch quickly makes the stale panel fail the gate, so real rotation is
  // only delayed by a frame or two.
  const double panel_hysteresis = sel_cfg.panel_switch_hysteresis;
  if (best_idx >= 0 && panel_hysteresis > 0.0 && current_panel_id_ >= 0) {
    const auto &best_hyp = topk[best_idx].hypothesis;
    if (best_hyp.kind == vehicle::HypothesisKind::Single) {
      const int best_panel =
          ((best_hyp.assignments[0].panel_id % 4) + 4) % 4;
      const int stay_panel = ((current_panel_id_ % 4) + 4) % 4;
      if (best_panel != stay_panel) {
        for (size_t i = 0; i < topk.size(); ++i) {
          const auto &cand = topk[i];
          if (cand.hypothesis.kind != vehicle::HypothesisKind::Single ||
              !cand.eval.valid || !cand.eval.gate_pass) {
            continue;
          }
          const int cand_panel =
              ((cand.hypothesis.assignments[0].panel_id % 4) + 4) % 4;
          if (cand_panel == stay_panel &&
              cand.eval.score + panel_hysteresis >= topk[best_idx].eval.score) {
            best_idx = static_cast<int>(i);
            break;
          }
        }
      }
    }
  }
  if (best_idx >= 0) {
    // Commit gate: confidence / margin check before attempting trial.
    bool commit_gate_pass = true;
    std::ostringstream gate_oss;

    if (top1_confidence < sel_cfg.min_top1_confidence) {
      commit_gate_pass = false;
      gate_oss << "conf=" << top1_confidence
               << "<" << sel_cfg.min_top1_confidence;
    }
    if (top1_top2_margin < sel_cfg.min_top1_top2_margin) {
      commit_gate_pass = false;
      if (!gate_oss.str().empty()) gate_oss << ",";
      gate_oss << "margin=" << top1_top2_margin
               << "<" << sel_cfg.min_top1_top2_margin;
    }

    if (commit_gate_pass) {
      const auto &best_hyp = topk[best_idx].hypothesis;
      vehicle::UkfTrial trial;

      if (best_hyp.kind == vehicle::HypothesisKind::Single) {
        trial = backend_->tryUpdateSingle(
            ctx, obs[best_hyp.assignments[0].obs_index],
            best_hyp.assignments[0].panel_id);
      } else {
        trial = backend_->tryUpdateDual(
            ctx, obs[best_hyp.assignments[0].obs_index],
            obs[best_hyp.assignments[1].obs_index],
            best_hyp.assignments[0].panel_id,
            best_hyp.assignments[1].panel_id);
      }

      // Triple gate: trial success + posterior sanity + reconstruction error.
      bool trial_ok = trial.success && trial.posterior_sanity_pass;
      if (trial_ok &&
          trial.reconstruction_pos_error > sel_cfg.max_reconstruction_pos_error) {
        trial_ok = false;
        std::ostringstream oss;
        oss << "reconstruction=" << trial.reconstruction_pos_error
            << ">" << sel_cfg.max_reconstruction_pos_error;
        trial.reject_reason = oss.str();
      }

      if (trial_ok) {
        backend_->commit(trial);
        committed = true;

        if (best_hyp.kind == vehicle::HypothesisKind::Single) {
          current_panel_id_ = best_hyp.assignments[0].panel_id;
        }

        std::ostringstream oss;
        oss << "committed_" << best_hyp.debug_name
            << "_nis=" << trial.eval.nis
            << "_score=" << topk[best_idx].eval.score
            << "_conf=" << top1_confidence
            << "_margin=" << top1_top2_margin
            << "_recon=" << trial.reconstruction_pos_error;
        if (trial.center_jump_clamped) oss << "_cjc";
        decision_reason = oss.str();
      } else {
        std::ostringstream oss;
        oss << "trial_rejected:" << trial.reject_reason;
        decision_reason = oss.str();
      }
    } else {
      decision_reason = "commit_gate_fail:" + gate_oss.str();
    }
  } else {
    decision_reason = "all_gate_fail";
  }

  // Update state tracking.
  update_time(obs_ts);
  increment_frame();
  last_update_committed_ = committed;

  if (committed) {
    consecutive_rejects_ = 0;
    reconstruction_reject_streak_ = 0;
    handle_observation_received(config_.tracker.tracking_thres);
  } else {
    ++consecutive_rejects_;
    if (decision_reason.rfind("trial_rejected:reconstruction", 0) == 0) {
      ++reconstruction_reject_streak_;
    }
    // sp_vision_25 tracker.cpp:83-90 style reject-streak reset: when the top-1
    // hypothesis is still NIS-sane but the tracker has been rejected for a
    // whole streak (stale/diverged state healing too slowly), reinitialize
    // from the current observation. Structure choice: preserve the learned
    // structure for plain NIS/gate streaks (post-blackout reacquisition, the
    // structure is still valid and relearning is slow); but when the streak is
    // dominated by reconstruction rejects the learned structure itself no
    // longer fits the observations — re-seeding it would preserve the
    // corruption (seen as a permanent yaw-rate collapse after a loop-rate
    // dip), so fall back to the nominal robot-description structure instead.
    // top1 gate is RELAXED (init_relax): during a hard jink the strict gate
    // fails for every hypothesis, so requiring it would skip the reset and
    // park the tracker in PREDICT_ONLY until lost_thres drops it — the exact
    // "stale estimate holds while detections are present" failure. A second,
    // gate-free force-reset fires before lost_thres as the last resort.
    const auto &gt = config_.vehicle_tracker.ukf_v1.gate;
    const double relax = std::max(1.0, gt.init_relax);
    const auto relaxed_gate_ok = [&]() {
      if (topk.empty() || !topk[0].eval.valid) return false;
      const auto &e = topk[0].eval;
      const bool single =
          topk[0].hypothesis.kind == vehicle::HypothesisKind::Single;
      const double nis_thr =
          (single ? gt.single_total_nis : gt.dual_total_nis) * relax;
      const double pos_thr =
          (single ? gt.single_pos_chi2 : gt.dual_each_pos_chi2) * relax;
      const double yaw_thr =
          (single ? gt.single_yaw_chi2 : gt.dual_each_yaw_chi2) * relax;
      return e.nis <= nis_thr && e.chi2_pos <= pos_thr &&
             e.chi2_yaw <= yaw_thr;
    };
    const bool top1_gate_ok = relaxed_gate_ok();
    const int reset_streak = config_.tracker.reject_reset_streak_frames;
    const int force_streak =
        std::max(reset_streak, config_.tracker.lost_thres - 4);
    const bool force_reset = !obs.empty() &&
        consecutive_rejects_ >= force_streak;
    if ((consecutive_rejects_ >= reset_streak && top1_gate_ok) ||
        force_reset) {
      const bool structure_suspect = force_reset ||
          (reconstruction_reject_streak_ * 2 >= consecutive_rejects_);
      const auto [cur_r1, cur_r2] = get_radii();
      const double cur_dza = backend_->spin_filter().get_dza();
      const double seed_r1 = structure_suspect ? nominal_r1_ : cur_r1;
      const double seed_r2 = structure_suspect ? nominal_r2_ : cur_r2;
      const double seed_dza = structure_suspect ? nominal_dza_ : cur_dza;
      std::fprintf(stderr,
                   "[vehicle] reject streak %d >= %d, reset from current obs "
                   "(structure %s: r1=%.3f r2=%.3f dza=%.3f)\n",
                   consecutive_rejects_,
                   config_.tracker.reject_reset_streak_frames,
                   structure_suspect ? "nominal" : "learned",
                   seed_r1, seed_r2, seed_dza);
      initialize(obs, seed_r1, seed_r2, seed_dza);
      consecutive_rejects_ = 0;
      reconstruction_reject_streak_ = 0;
      committed = true;
      last_update_committed_ = true;
      update_time(obs_ts);
      increment_frame();
      handle_observation_received(config_.tracker.tracking_thres);
      decision_reason = "reject_streak_reset";
      last_hypothesis_debug_.valid = true;
      last_hypothesis_debug_.obs_count = static_cast<int>(obs.size());
      last_hypothesis_debug_.committed = true;
      last_hypothesis_debug_.degraded = false;
      last_hypothesis_debug_.topk = topk;
      last_hypothesis_debug_.top1_confidence = top1_confidence;
      last_hypothesis_debug_.top1_top2_margin = top1_top2_margin;
      last_hypothesis_debug_.decision_reason = decision_reason;
      populate_debug_snapshot();
      return true;
    }
    std::fprintf(stderr, "[norm4v3] obs=%zu NOT committed: %s\n",
                 obs.size(), decision_reason.c_str());
    handle_observation_loss(config_.tracker.tracking_thres,
                            config_.tracker.lost_thres);
  }

  // Populate debug frame.
  last_hypothesis_debug_.valid = true;
  last_hypothesis_debug_.obs_count = static_cast<int>(obs.size());
  last_hypothesis_debug_.committed = committed;
  last_hypothesis_debug_.degraded = !committed;
  last_hypothesis_debug_.topk = topk;
  last_hypothesis_debug_.top1_confidence = top1_confidence;
  last_hypothesis_debug_.top1_top2_margin = top1_top2_margin;
  last_hypothesis_debug_.decision_reason = decision_reason;

  populate_debug_snapshot();

  // Return true when observations were processed, even if not committed, so
  // TrackerManager keeps the tracker alive; the committed flag is reported
  // separately through last_update_committed().
  return true;
}

Eigen::Vector3d VehicleArmorTracker::get_center_position() const {
  return backend_->spin_filter().get_center_position();
}

double VehicleArmorTracker::get_yaw() const {
  // Published TrackedRobot uses the shared armors_offset convention where
  // armor 0 is at local (-r, 0). Shift the internal center yaw by pi so
  // downstream projection reconstructs armor positions consistently.
  return normalize_angle(backend_->spin_filter().get_yaw() + M_PI);
}

std::pair<double, double> VehicleArmorTracker::get_radii() const {
  return backend_->spin_filter().get_radii();
}

SpinFilterInterface &VehicleArmorTracker::spin_filter() {
  return backend_->spin_filter();
}

const SpinFilterInterface &VehicleArmorTracker::spin_filter() const {
  return backend_->spin_filter();
}

ManeuverResult VehicleArmorTracker::assess_maneuver() const {
  const auto &sf = backend_->spin_filter();
  return maneuver_detector_.detect(sf.last_nis(),
                                   sf.last_innov_xyz().norm(),
                                   sf.last_update_type());
}

Eigen::Vector3d VehicleArmorTracker::get_publish_velocity() const {
  const auto &sf = backend_->spin_filter();
  const auto &idx = sf.state_idx();
  const auto &x = sf.x();
  Eigen::Vector3d vel(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  return vel;
}

bool VehicleArmorTracker::is_ambiguous_single_mode() const {
  return false;
}

int VehicleArmorTracker::effective_num_armors() const { return 4; }

double VehicleArmorTracker::confidence_scale() const {
  return 1.0;
}

std::vector<geometry_msgs::msg::Pose>
VehicleArmorTracker::build_armors_offset_for_message() const {
  return {};
}

void VehicleArmorTracker::select_topk(
    std::vector<vehicle::MeasurementEval> &evals,
    const std::vector<vehicle::Hypothesis> &hyps,
    int topk_count,
    std::vector<vehicle::TopKEntry> *topk_out,
    double *confidence_out, double *margin_out) const {
  const int n = static_cast<int>(evals.size());
  if (n == 0) {
    *confidence_out = 0.0;
    *margin_out = 0.0;
    return;
  }

  // Create indexed list.
  std::vector<int> indices(n);
  std::iota(indices.begin(), indices.end(), 0);

  // Sort: gate_pass first, then by score descending.
  std::sort(indices.begin(), indices.end(),
            [&evals](int a, int b) {
              if (evals[a].gate_pass != evals[b].gate_pass)
                return evals[a].gate_pass;
              return evals[a].score > evals[b].score;
            });

  // Take TopK.
  const int k = std::min(topk_count, n);
  topk_out->clear();
  topk_out->reserve(k);

  // Compute log-sum-exp over topk for softmax.
  double max_score = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < k; ++i) {
    max_score = std::max(max_score, evals[indices[i]].score);
  }

  double logsumexp = 0.0;
  std::vector<double> weights(k);
  for (int i = 0; i < k; ++i) {
    weights[i] = std::exp(evals[indices[i]].score - max_score);
    logsumexp += weights[i];
  }

  for (int i = 0; i < k; ++i) {
    weights[i] /= logsumexp;
    vehicle::TopKEntry entry;
    entry.hypothesis = hyps[indices[i]];
    entry.eval = evals[indices[i]];
    entry.normalized_weight = weights[i];
    topk_out->push_back(entry);
  }

  *confidence_out = topk_out->empty() ? 0.0 : topk_out->front().normalized_weight;
  *margin_out = (k >= 2)
                    ? (topk_out->at(0).eval.score - topk_out->at(1).eval.score)
                    : std::numeric_limits<double>::infinity();
}

void VehicleArmorTracker::populate_debug_snapshot() {
  debug_snapshot_.valid = last_hypothesis_debug_.valid;
  debug_snapshot_.track_mode = 0;
  debug_snapshot_.current_panel_id = current_panel_id_;
  debug_snapshot_.mode_state = static_cast<int>(mode_);

  if (!last_hypothesis_debug_.topk.empty()) {
    const auto &top = last_hypothesis_debug_.topk.front();
    debug_snapshot_.candidate_panel_id = top.hypothesis.assignments[0].panel_id;
    debug_snapshot_.candidate_prob = top.normalized_weight;
    debug_snapshot_.entropy_norm = 1.0 - top.normalized_weight;
    debug_snapshot_.max_prob = top.normalized_weight;

    if (last_hypothesis_debug_.topk.size() >= 2) {
      const auto &second = last_hypothesis_debug_.topk[1];
      debug_snapshot_.candidate_margin =
          top.eval.score - second.eval.score;
    }
  }

  debug_snapshot_.binding_confidence = last_hypothesis_debug_.top1_confidence;
  debug_snapshot_.degraded_single_obs_mode = last_hypothesis_debug_.degraded;
  debug_snapshot_.committed = last_hypothesis_debug_.committed;
  debug_snapshot_.top1_confidence = last_hypothesis_debug_.top1_confidence;
  debug_snapshot_.top1_top2_margin = last_hypothesis_debug_.top1_top2_margin;
  debug_snapshot_.decision_reason = last_hypothesis_debug_.decision_reason;
  debug_snapshot_.warmup_active = warmup_state_.active ? 1 : 0;
  if (!last_hypothesis_debug_.topk.empty()) {
    debug_snapshot_.top1_nis = last_hypothesis_debug_.topk.front().eval.nis;
  }
}

// ── Warmup / Mode Routing ──

void VehicleArmorTracker::init_warmup(const std::vector<ObservationData> &obs,
                                       double r1, double r2, double dza) {
  warmup_state_ = vehicle::WarmupState{};
  warmup_state_.active = true;

  warmup_state_.h0.seed_panel = 0;
  warmup_state_.h0.r1 = r1;
  warmup_state_.h0.r2 = r2;
  warmup_state_.h0.dza = dza;

  warmup_state_.h1.seed_panel = 1;
  warmup_state_.h1.r1 = r1;
  warmup_state_.h1.r2 = r2;
  warmup_state_.h1.dza = dza;

  // Initialize the structured backend with H0 (panel 0) for predict continuity.
  // The warmup internally evaluates both H0 and H1 via evaluateSingle.
  backend_->reset(obs[0], 0, r1, r2, dza);
  warmup_last_obs_ = obs[0];
  current_panel_id_ = -1;  // unresolved during legacy warmup

  warmup_state_.warmup_reason = "warmup_init_seed_01";
}

bool VehicleArmorTracker::run_warmup(const std::vector<ObservationData> &obs) {
  const auto &warmup_cfg = config_.vehicle_tracker.warmup;
  warmup_state_.total_frames++;

  // Predict and get prior context.
  auto ctx = backend_->buildPredictContext();

  // Use first observation for single-obs shallow evaluation on both seeds.
  const auto &primary_obs = obs[0];
  warmup_last_obs_ = primary_obs;

  // Evaluate H0 (panel 0 assumption).
  auto eval_h0 = backend_->evaluateSingle(ctx, primary_obs, 0);
  if (eval_h0.gate_pass && eval_h0.valid) {
    warmup_state_.h0.gate_pass_count++;
    warmup_state_.h0.accumulated_score += eval_h0.log_likelihood;
  }
  warmup_state_.h0.total_frames++;

  // Evaluate H1 (panel 1 assumption).
  auto eval_h1 = backend_->evaluateSingle(ctx, primary_obs, 1);
  if (eval_h1.gate_pass && eval_h1.valid) {
    warmup_state_.h1.gate_pass_count++;
    warmup_state_.h1.accumulated_score += eval_h1.log_likelihood;
  }
  warmup_state_.h1.total_frames++;

  if (!std::isfinite(eval_h0.log_likelihood) ||
      !std::isfinite(eval_h1.log_likelihood)) {
    warmup_state_.settle_frames = 0;
    warmup_state_.warmup_reason = "warmup_nonfinite_score";
    return true;
  }

  // Compute margin between H0 and H1.
  double margin = eval_h0.log_likelihood - eval_h1.log_likelihood;
  double abs_margin = std::abs(margin);

  // Compute confidence via softmax between the two.
  double max_score = std::max(eval_h0.log_likelihood, eval_h1.log_likelihood);
  double w0 = std::exp(eval_h0.log_likelihood - max_score);
  double w1 = std::exp(eval_h1.log_likelihood - max_score);
  double confidence = std::max(w0, w1) / (w0 + w1);

  bool h0_better = eval_h0.log_likelihood > eval_h1.log_likelihood;

  std::ostringstream oss;
  oss << "warmup_f" << warmup_state_.total_frames
      << "_h0=" << eval_h0.log_likelihood
      << "_h1=" << eval_h1.log_likelihood
      << "_margin=" << abs_margin
      << "_conf=" << confidence;
  warmup_state_.warmup_reason = oss.str();

  // Check convergence conditions.
  bool has_min_frames =
      warmup_state_.total_frames >= warmup_cfg.warmup_frames;

  if (has_min_frames &&
      abs_margin > warmup_cfg.min_margin_to_commit &&
      confidence > warmup_cfg.min_confidence_to_commit) {
    warmup_state_.settle_frames++;

    if (warmup_state_.settle_frames >= warmup_cfg.min_settle_frames) {
      // Converged: promote winner.
      warmup_state_.active = false;
      warmup_state_.winning_branch = h0_better ? 0 : 1;
      warmup_state_.final_margin = abs_margin;
      warmup_state_.final_confidence = confidence;

      promote_warmup_winner();
      return true;
    }
  } else {
    warmup_state_.settle_frames = 0;
  }

  // Timeout: fall back to the unified structured path.
  if (warmup_state_.total_frames > warmup_cfg.warmup_frames * 3) {
    warmup_state_.active = false;
    current_panel_id_ = 0;
    set_mode(vehicle::VehicleTrackerMode::STRUCTURED);
    warmup_state_.warmup_reason += "_timeout_structured";
  }

  // During legacy warmup, do not commit; backend stays predict-only.
  return true;
}

void VehicleArmorTracker::promote_warmup_winner() {
  int winner_panel = warmup_state_.winning_branch == 0 ? 0 : 1;
  if (warmup_last_obs_.has_value()) {
    backend_->reset(warmup_last_obs_.value(), winner_panel, default_r1_,
                    default_r2_, default_dza_);
  }
  current_panel_id_ = winner_panel;
  set_mode(vehicle::VehicleTrackerMode::STRUCTURED);

  std::ostringstream oss;
  oss << "warmup_promoted_panel" << winner_panel
      << "_margin=" << warmup_state_.final_margin
      << "_conf=" << warmup_state_.final_confidence;
  warmup_state_.warmup_reason = oss.str();
}

void VehicleArmorTracker::set_mode(vehicle::VehicleTrackerMode m) {
  mode_ = m;
  apply_mode_routing();
}

void VehicleArmorTracker::apply_mode_routing() {
  const auto &routing = config_.vehicle_tracker.mode_routing;
  // Phase-1 routing contract in this tracker:
  // Ordinary 4-panel targets always publish structured robot semantics. Single
  // and multi-observation frames differ only by hypothesis kind, not by pipeline.
  // Outpost keeps its separate 3-panel special-case tracker.
  (void)routing;
}

}  // namespace fyt::auto_aim
