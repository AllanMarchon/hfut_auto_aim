// ROS-free Pipeline orchestration (replaces GimbalPipelineNode).
// Constructor mirrors the node ctor order; updateTracking/computeCommand replace
// armorsCallback/timerCallback.
#include "pipeline.hpp"
#include "param_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>

#include "max_entropy_tracker/msg_converter.hpp"
#include "max_entropy_tracker/utils/tracking_output_policy.hpp"
#include "rm_utils/logger/log.hpp"

namespace hfut::pipeline {

using namespace fyt::auto_aim;
namespace rd = fyt::auto_aim::robot_description;

Pipeline::Pipeline(const std::string& config_path, const std::string& node_name,
                   const PipelineOverrides& overrides)
    : Pipeline(std::vector<std::string>{config_path}, node_name, overrides) {}

Pipeline::Pipeline(const std::vector<std::string>& config_paths,
                   const std::string& node_name,
                   const PipelineOverrides& overrides) {
  // FYT loggers used by the inlined tracker/selector code.
  try { FYT_REGISTER_LOGGER("target_selector", "/tmp/hfut_auto_aim_log", INFO); } catch (...) {}
  try { FYT_REGISTER_LOGGER("gimbal_pipeline", "/tmp/hfut_auto_aim_log", INFO); } catch (...) {}

  loadParametersFromYaml(config_paths, node_name);

  // ── 1. Declare all parameters (lifted) ──
  declareTrackerParameters();
  declareTargetSelectorParameters();
  declareGimbalControllerParameters();

  // ── 2. Read common / tracker params ──
  target_frame_ = get_parameter("target_frame").as_string();
  source_frame_ = get_parameter("source_frame").as_string();
  predict_rate_ = get_parameter("predict_rate").as_double();
  debug_mode_ = get_parameter("debug_mode").as_bool();
  tracker_timeout_s_ = std::max(get_parameter("tracker_timeout").as_double(), 1e-3);

  tracker_config_ = UnifiedConfig::create_default();
  applyTrackerParamsToConfig();

  if (!std::isfinite(overrides.observation_noise_scale) ||
      overrides.observation_noise_scale <= 0.0) {
    throw std::invalid_argument("observation_noise_scale must be finite and > 0");
  }
  const double observation_scale = overrides.observation_noise_scale;
  auto scale_sigma = [observation_scale](double& sigma) {
    sigma = std::max(1e-6, sigma * observation_scale);
  };
  scale_sigma(tracker_config_.ukf.obs_noise_pos);
  scale_sigma(tracker_config_.ukf.obs_noise_yaw);
  scale_sigma(tracker_config_.ukf.ypd_sigma_azi);
  scale_sigma(tracker_config_.ukf.ypd_sigma_ele);
  scale_sigma(tracker_config_.ukf.ypd_sigma_dist_coeff);
  scale_sigma(tracker_config_.ukf.dual_obs_noise_pos);
  scale_sigma(tracker_config_.ukf.dual_obs_noise_yaw);
  scale_sigma(tracker_config_.norm4_v2.ukf_v1.sigma_pos_xy);
  scale_sigma(tracker_config_.norm4_v2.ukf_v1.sigma_pos_z);
  scale_sigma(tracker_config_.norm4_v2.ukf_v1.sigma_yaw);
  auto scale_v3 = [&scale_sigma](auto& config) {
    scale_sigma(config.sigma_pos_xy);
    scale_sigma(config.sigma_pos_z);
    scale_sigma(config.sigma_yaw);
  };
  scale_v3(tracker_config_.vehicle_tracker.ukf_v1);
  scale_v3(tracker_config_.vehicle_tracker.ukf_v2);
  scale_v3(tracker_config_.vehicle_tracker.inekf);
  scale_sigma(tracker_config_.outpost.v3_observation_sigma_pos_xy);
  scale_sigma(tracker_config_.outpost.v3_observation_sigma_pos_z);

  auto require_non_negative = [](double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
    }
  };
  auto require_positive = [](double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be finite and > 0");
    }
  };
  require_non_negative(
      overrides.motion_guard.stationary_speed_deadband_mps,
      "motion_guard.stationary_speed_deadband_mps");
  require_non_negative(
      overrides.max_temp_lost_prediction_s,
      "max_temp_lost_prediction_s");
  max_temp_lost_prediction_s_ = overrides.max_temp_lost_prediction_s;
  require_non_negative(
      overrides.temp_lost_coast_max_s, "temp_lost_coast_max_s");
  temp_lost_coast_max_s_ = overrides.temp_lost_coast_max_s;
  require_non_negative(
      overrides.temp_lost_coast_min_speed_mps,
      "temp_lost_coast_min_speed_mps");
  temp_lost_coast_min_speed_mps_ = overrides.temp_lost_coast_min_speed_mps;
  if (!std::isfinite(overrides.attitude_mount_pitch_deg)) {
    throw std::invalid_argument(
        "attitude_mount_pitch_deg must be finite");
  }
  attitude_mount_pitch_rad_ =
      overrides.attitude_mount_pitch_deg * M_PI / 180.0;
  require_positive(
      overrides.attitude_ema_alpha, "attitude_ema_alpha");
  if (overrides.attitude_ema_alpha > 1.0) {
    throw std::invalid_argument("attitude_ema_alpha must be <= 1");
  }
  attitude_ema_alpha_ = overrides.attitude_ema_alpha;
  attitude_apply_to_geometry_ = overrides.attitude_apply_to_geometry;
  if (!std::isfinite(overrides.id_association_max_distance_m) ||
      overrides.id_association_max_distance_m < 0.0) {
    throw std::invalid_argument(
        "id_association_max_distance_m must be finite and >= 0");
  }
  id_association_max_distance_m_ = overrides.id_association_max_distance_m;
  require_positive(
      overrides.motion_guard.max_linear_speed_mps,
      "motion_guard.max_linear_speed_mps");
  require_positive(
      overrides.motion_guard.max_linear_acceleration_mps2,
      "motion_guard.max_linear_acceleration_mps2");
  require_positive(
      overrides.motion_guard.temp_lost_velocity_half_life_s,
      "motion_guard.temp_lost_velocity_half_life_s");
  require_positive(
      overrides.motion_guard.velocity_reset_std_mps,
      "motion_guard.velocity_reset_std_mps");
  require_positive(
      overrides.motion_guard.acceleration_reset_std_mps2,
      "motion_guard.acceleration_reset_std_mps2");
  require_positive(
      overrides.motion_guard.max_yaw_rate_rad_s,
      "motion_guard.max_yaw_rate_rad_s");
  require_non_negative(
      overrides.motion_guard.yaw_rate_deadband_rad_s,
      "motion_guard.yaw_rate_deadband_rad_s");
  require_positive(
      overrides.motion_guard.yaw_rate_reset_std_rad_s,
      "motion_guard.yaw_rate_reset_std_rad_s");
  tracker_config_.tracker.motion_guard = overrides.motion_guard;


  // ── 3. Tracker manager ──
  double dt = (predict_rate_ > 0) ? (1.0 / predict_rate_) : 0.01;
  tracker_manager_ = std::make_unique<TrackerManager>(
      tracker_config_, dt,
      get_parameter("default_r1").as_double(),
      get_parameter("default_r2").as_double(),
      get_parameter("default_dza").as_double(),
      tracker_timeout_s_,
      get_parameter("enable_oscillation_detection").as_bool());

  // ── 4. Robot description facade + projection-mode policy ──
  robot_description_facade_ = std::make_unique<rd::RobotDescriptionFacade>();
  robot_description_facade_->setStrictUnknownReject(
      get_parameter("robot_description.strict_unknown_reject").as_bool());
  {
    auto mode_raw = get_parameter("robot_description.default_projection_mode").as_string();
    std::string mode_lower = mode_raw;
    std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    using RU = rd::TrackedRobotUsage;
    RU::ProjectionMode default_mode = (mode_lower == "full_se3")
        ? RU::ProjectionMode::FULL_SE3 : RU::ProjectionMode::YAW_PLANE;
    const auto ids_vec = get_parameter("robot_description.full_se3_ids").as_string_array();
    std::unordered_set<std::string> full_se3_ids(ids_vec.begin(), ids_vec.end());
    const auto types_vec = get_parameter("robot_description.full_se3_robot_types").as_integer_array();
    std::unordered_set<uint8_t> full_se3_types;
    for (auto v : types_vec) if (v >= 0 && v <= 255) full_se3_types.insert(static_cast<uint8_t>(v));
    RU::setProjectionModePolicy(default_mode, full_se3_ids, full_se3_types);
  }

  // ── 5. Target selector config ──
  selection_config_.reference_yaw = get_parameter("selector.reference_yaw").as_double();
  selection_config_.max_yaw_deviation = get_parameter("selector.max_yaw_deviation").as_double();
  selection_config_.max_distance = get_parameter("selector.max_distance").as_double();
  selection_config_.min_confidence = get_parameter("selector.min_confidence").as_double();
  selection_config_.hysteresis_threshold = get_parameter("selector.hysteresis_threshold").as_double();
  selection_config_.priority_robot_ids = get_parameter("selector.priority_robot_ids").as_string_array();
  selection_config_.blocked_robot_ids = get_parameter("selector.blocked_robot_ids").as_string_array();
  selection_config_.sticky_lock_frames = get_parameter("selector.sticky_lock_frames").as_int();
  selection_config_.sticky_lost_frames = get_parameter("selector.sticky_lost_frames").as_int();
  selector_strategy_name_ = get_parameter("selector.strategy").as_string();
  initSelectionStrategy();

  // ── 6. Controller ──
  bullet_speed_ = overrides.bullet_speed.value_or(
      get_parameter("controller.bullet_speed").as_double());
  if (!std::isfinite(bullet_speed_) || bullet_speed_ <= 0.0) {
    throw std::invalid_argument("bullet_speed must be finite and > 0");
  }
  control_rate_ = get_parameter("controller.control_rate").as_double();
  current_gimbal_strategy_name_ = overrides.controller_strategy.empty()
      ? get_parameter("controller.strategy").as_string()
      : overrides.controller_strategy;
  ballistic_mode_ = get_parameter("controller.ballistic_mode").as_string();

  initGimbalComponents();
  initGimbalStrategies();
  configureControllerFromParams();

  FYT_INFO(
      "gimbal_pipeline",
      "Pipeline ready: target_frame={}, control_strategy={}, selector={}, ballistic={}, "
      "bullet_speed={:.2f}m/s, observation_noise_scale={:.3f}, "
      "temp_lost_output={:.3f}s, motion_guard={}, smoother={}",
      target_frame_, current_gimbal_strategy_name_, selector_strategy_name_, ballistic_mode_,
      bullet_speed_, observation_scale, max_temp_lost_prediction_s_,
      tracker_config_.tracker.motion_guard.enabled ? "on" : "off",
      smoother_config_.enable ? "on" : "off");
}

void Pipeline::initGimbalComponents() {
  position_calculator_ = std::make_shared<gimbal_controller::ArmorPositionCalculator>();
  armor_selector_ = std::make_shared<gimbal_controller::ArmorSelector>();
  ballistic_client_ = std::make_shared<gimbal_controller::BallisticSolverClient>();
  local_compensator_ = std::make_shared<gimbal_controller::LocalTrajectoryCompensator>();
  fire_advisor_ = std::make_shared<gimbal_controller::FireAdvisor>();
  fire_advice_engine_ = std::make_shared<gimbal_controller::FireAdviceEngine>();
  fire_advice_engine_->setComponents(position_calculator_, ballistic_client_,
                                     local_compensator_, fire_advisor_);
  fire_advice_engine_->setBallisticMode(ballistic_mode_);
  gimbal_control_core_ = std::make_shared<gimbal_controller::GimbalControlCore>();
  gimbal_control_core_->setFireModules(fire_advice_engine_, fire_advisor_);
}

void Pipeline::initSelectionStrategy() {
  if (selector_strategy_name_ == "priority_list")
    selection_strategy_ = std::make_unique<PriorityListStrategy>();
  else if (selector_strategy_name_ == "sticky_min_yaw_deviation")
    selection_strategy_ = std::make_unique<StickyMinYawDeviationStrategy>();
  else
    selection_strategy_ = std::make_unique<MinYawDeviationStrategy>();
}

SelectionResult Pipeline::selectTargetInternal(const rm_interfaces::msg::TrackedRobots& robots) {
  selection_config_.current_target_id = current_target_id_;
  auto result = selection_strategy_->selectTarget(robots, selection_config_);
  if (!result.has_value()) { current_target_id_ = ""; return SelectionResult(); }
  current_target_id_ = result->robot_id;
  return *result;
}

rm_interfaces::msg::TrackedRobots Pipeline::buildTrackedRobotsMsg(double sim_time) {
  rm_interfaces::msg::TrackedRobots tracked_msg;
  tracked_msg.header.frame_id = target_frame_;
  tracked_msg.header.stamp.sec = static_cast<int32_t>(sim_time);
  tracked_msg.header.stamp.nanosec =
      static_cast<uint32_t>((sim_time - std::floor(sim_time)) * 1e9);

  for (const auto& rid : tracker_manager_->active_robot_ids()) {
    auto* tracker = tracker_manager_->get(rid);
    if (!tracker || (!tracker->is_tracking() && !tracker->is_temp_lost())) continue;

    auto post = tracker_manager_->post_process_output(rid, sim_time, smoother_config_);
    const SmoothedOutput* smoothed = post.has_smoothed ? &post.smoothed : nullptr;

    if (tracker->is_temp_lost()) {
      // Adaptive dead-reckoning window: a target that was actually moving when
      // last observed keeps its short-term course, so let the gimbal coast on
      // the (velocity-decayed) prediction for longer instead of freezing in
      // place and letting a traversing target leave the FOV. Stationary-ish
      // predictions stay on the tight timeout to avoid steering from jitter.
      const double speed = smoothed
          ? smoothed->velocity.norm()
          : tracker->get_publish_velocity().norm();
      const double allowed_s = speed >= temp_lost_coast_min_speed_mps_
          ? temp_lost_coast_max_s_
          : max_temp_lost_prediction_s_;
      const auto age = tracker_manager_->observation_age(rid, sim_time);
      if (!temporaryPredictionIsFresh(age, allowed_s)) {
        continue;
      }
    }

    const int visible = tracker_manager_->visible_observation_count(rid);

    rd::TrackedRobotBuildInput input{
        tracked_msg.header, target_frame_, rid, *tracker, smoothed, visible};
    auto build = robot_description_facade_->tryBuildTrackedRobot(input);
    if (!build.ok() || build.robot.robot_id.empty()) continue;

    // Attach the plate-derived whole-vehicle attitude estimate (roll/pitch)
    // to the published state. The default yaw-plane projection deliberately
    // treats this noisy slow signal as diagnostic only; calibrated producers
    // can opt into full SE(3) through the projection-mode policy.
    auto attitude_it = attitude_estimators_.find(rid);
    if (attitude_it != attitude_estimators_.end() &&
        attitude_it->second.valid()) {
      const auto attitude = attitude_it->second.estimate(build.robot.yaw);
      if (attitude.valid) {
        last_attitude_estimates_[rid] = attitude;
        if (attitude.trusted_for_geometry || attitude_apply_to_geometry_) {
          auto& robot = build.robot;
          robot.center_pose.position = robot.center_position;
          robot.center_pose.orientation.x = attitude.orientation.x();
          robot.center_pose.orientation.y = attitude.orientation.y();
          robot.center_pose.orientation.z = attitude.orientation.z();
          robot.center_pose.orientation.w = attitude.orientation.w();
          robot.layout_attitude_valid = true;
        }
      }
    }

    tracked_msg.robots.push_back(build.robot);
  }
  return tracked_msg;
}

std::string Pipeline::assign_robot_id(const std::string& detected_id,
                                      const ObservationData& obs) const {
  if (id_association_max_distance_m_ <= 0.0 || !tracker_manager_) {
    return detected_id;
  }
  // Keep the detector's label when a track with that exact id already exists
  // near the observation (identity is unambiguous); otherwise adopt the
  // nearest track within the association radius — the NN number classifier
  // misreads constantly at range, and each misread would otherwise spawn a
  // phantom track that starves the real one.
  const auto views = tracker_manager_->initialized_tracker_views();
  const double max_d2 =
      id_association_max_distance_m_ * id_association_max_distance_m_;
  const ObservationData* pobs = &obs;
  auto dist2 = [&pobs](const auto& center) {
    const double dx = center.x() - pobs->x;
    const double dy = center.y() - pobs->y;
    const double dz = center.z() - pobs->z;
    return dx * dx + dy * dy + dz * dz;
  };
  double best_d2 = max_d2;
  std::string best_id;
  for (const auto& view : views) {
    if (!view.tracker || !view.tracker->is_initialized()) continue;
    const auto center = view.tracker->get_center_position();
    const double d2 = dist2(center);
    if (view.robot_id == detected_id && d2 < max_d2) {
      return detected_id;
    }
    if (d2 < best_d2) {
      best_d2 = d2;
      best_id = view.robot_id;
    }
  }
  return best_id.empty() ? detected_id : best_id;
}

void Pipeline::updateTracking(const rm_interfaces::msg::Armors& armors,
                              const Eigen::Matrix3d& R_cam2world,
                              const Eigen::Vector3d& t_cam2world,
                              double sim_time) {
  // Step 1: group observations by robot id, transforming each armor pose from
  // the camera optical frame to the world frame (replaces the tf2 lookup).
  std::unordered_map<std::string, std::vector<ObservationData>> obs_by_robot;
  const bool strict = robot_description_facade_ && robot_description_facade_->strictUnknownReject();

  for (const auto& armor : armors.armors) {
    if (strict && !robot_description_facade_->isSupportedRobotId(armor.number)) continue;

    // Detector-level block list: a recognized-but-blocked armor is dropped
    // here, so no tracker/selector/attitude downstream ever sees it (the
    // selector-level check alone would still let phantom tracks accumulate).
    if (std::find(selection_config_.blocked_robot_ids.begin(),
                  selection_config_.blocked_robot_ids.end(),
                  armor.number) != selection_config_.blocked_robot_ids.end()) {
      continue;
    }

    // Detector metadata (2D + BA) preserved; 3D pose replaced by world-frame.
    ObservationData obs = armor_to_observation(armor, sim_time);
    Eigen::Vector3d p_cam(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
    Eigen::Vector3d p_world = R_cam2world * p_cam + t_cam2world;

    // Rotate the armor orientation into world, then recompute radial yaw the
    // same way pose_to_observation does (normal->radial = +pi).
    Eigen::Quaterniond q_cam(armor.pose.orientation.w, armor.pose.orientation.x,
                             armor.pose.orientation.y, armor.pose.orientation.z);
    Eigen::Matrix3d R_world = R_cam2world * q_cam.toRotationMatrix();
    double yaw_normal = std::atan2(R_world(1, 0), R_world(0, 0));
    obs.x = p_world.x();
    obs.y = p_world.y();
    obs.z = p_world.z();
    obs.yaw = armor.radial_yaw_valid ? armor.radial_yaw : yaw_normal + M_PI;
    obs.timestamp = sim_time;

    // Reassign the detector's robot id by spatial association: at range the
    // NN number classifier misreads constantly (one physical robot appears as
    // "3"/"4"/"5"/"outpost" frame to frame), spawning phantom tracks that
    // starve the real one — the visible "observer keeps clearing the target"
    // failure. The obs carries no id itself, only the grouping key matters.
    const std::string robot_id = assign_robot_id(armor.number, obs);

    // A v3 direct record carries an exact plate frame and is trusted for
    // geometry. Planar PnP orientations remain a diagnostic slow signal unless
    // tracking.attitude.apply_to_geometry explicitly opts them in.
    if (armor.pose_estimate_mode != 0 || armor.pose_orientation_trusted) {
      auto& estimator = attitude_estimators_
          .try_emplace(robot_id,
                       fyt::auto_aim::BodyAttitudeEstimator(
                           attitude_mount_pitch_rad_, attitude_ema_alpha_))
          .first->second;
      if (armor.pose_orientation_trusted) {
        constexpr double kNominalDirectMountPitchRad = 15.0 * M_PI / 180.0;
        estimator.addPlateOrientation(
            R_world, kNominalDirectMountPitchRad, true);
      } else {
        estimator.addPlateOrientation(R_world);
      }
    }
    if (std::getenv("HFUT_DEBUG_OBS") != nullptr) {
      std::fprintf(stderr,
                   "[obs] t=%.3f id=%s pos=(%.3f,%.3f,%.3f) yaw=%.4f yaw_normal=%.4f "
                   "R=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n",
                   sim_time, armor.number.c_str(), obs.x, obs.y, obs.z, obs.yaw,
                   yaw_normal,
                   R_world(0, 0), R_world(0, 1), R_world(0, 2),
                   R_world(1, 0), R_world(1, 1), R_world(1, 2),
                   R_world(2, 0), R_world(2, 1), R_world(2, 2));
    }

    // Rotate BA covariance (xyz block) into world if present.
    if (obs.ba_pnp.has_value() && obs.ba_pnp->cov_valid && obs.ba_pnp->cov_xyz_yaw.allFinite()) {
      Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
      A.block<3, 3>(0, 0) = R_cam2world;
      obs.ba_pnp->cov_xyz_yaw = A * obs.ba_pnp->cov_xyz_yaw * A.transpose();
      obs.ba_pnp->cov_xyz_yaw = 0.5 * (obs.ba_pnp->cov_xyz_yaw + obs.ba_pnp->cov_xyz_yaw.transpose());
      obs.ba_pnp->frame_aligned = obs.ba_pnp->cov_xyz_yaw.allFinite();
    }
    obs_by_robot[robot_id].push_back(obs);
  }

  // Step 2: tracker frame process.
  tracker_manager_->process_frame(obs_by_robot, sim_time, smoother_config_);

  debug_.tracker_structure_valid = false;
  for (const auto &view : tracker_manager_->initialized_tracker_views()) {
    if (view.tracker == nullptr) continue;
    const auto [r1, r2] = view.tracker->get_radii();
    debug_.tracker_structure_valid = true;
    debug_.tracker_r1 = r1;
    debug_.tracker_r2 = r2;
    debug_.tracker_dza = view.tracker->spin_filter().get_dza();
    break;
  }

  // Step 3: build TrackedRobots + run target selection.
  auto tracked_msg = buildTrackedRobotsMsg(sim_time);
  SelectionResult sel;
  if (!tracked_msg.robots.empty()) {
    sel = selectTargetInternal(tracked_msg);
  } else {
    current_target_id_.clear();
  }

  // Step 4: cache for computeCommand.
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  latest_tracked_robots_ = std::make_shared<rm_interfaces::msg::TrackedRobots>(tracked_msg);
  latest_selected_target_id_ = sel.robot_id;
  latest_selected_confidence_ = sel.confidence;
  latest_update_time_ = sim_time;

  debug_.num_tracked = static_cast<int>(tracked_msg.robots.size());
  debug_.selected_id = sel.robot_id;
  debug_.selected_track_state = -1;
  debug_.selected_state_valid = false;
  debug_.tracked_armor_poses.clear();
  debug_.tracker_update_valid = false;
  debug_.tracker_update_committed = false;
  debug_.tracker_observation_count = 0;
  debug_.tracker_top1_nis = -1.0;
  debug_.tracker_top1_chi2_pos = -1.0;
  debug_.tracker_top1_chi2_yaw = -1.0;
  debug_.tracker_top1_hypothesis.clear();
  debug_.tracker_decision_reason.clear();
  for (const auto& r : tracked_msg.robots) {
    if (r.robot_id == sel.robot_id) {
      debug_.selected_track_state = r.track_state;
      debug_.selected_state = r;
      debug_.selected_state_valid = true;
      if (const auto* vehicle_tracker = dynamic_cast<const VehicleArmorTracker*>(
              tracker_manager_->get(r.robot_id))) {
        const auto& update_debug = vehicle_tracker->last_hypothesis_debug();
        debug_.tracker_update_valid = update_debug.valid;
        debug_.tracker_update_committed = update_debug.committed;
        debug_.tracker_observation_count = update_debug.obs_count;
        debug_.tracker_decision_reason = update_debug.decision_reason;
        if (!update_debug.topk.empty()) {
          const auto& top1 = update_debug.topk.front();
          debug_.tracker_top1_nis = top1.eval.nis;
          debug_.tracker_top1_chi2_pos = top1.eval.chi2_pos;
          debug_.tracker_top1_chi2_yaw = top1.eval.chi2_yaw;
          debug_.tracker_top1_hypothesis = top1.hypothesis.debug_name;
        }
      }
      // Keep the estimator's full-vehicle plate geometry available to the
      // debug overlay for the whole track lifetime, not only while the
      // controller is aiming (control target valid).
      debug_.tracked_armor_poses = position_calculator_->calculatePoses(r);
      const bool uses_large_armor =
          r.robot_type == rm_interfaces::msg::TrackedRobot::HERO_4 ||
          r.robot_type == rm_interfaces::msg::TrackedRobot::OUTPOST_3 ||
          r.robot_type == rm_interfaces::msg::TrackedRobot::BASE ||
          r.robot_type == rm_interfaces::msg::TrackedRobot::BALANCE_2;
      debug_.tracked_armor_width_m = uses_large_armor ? 0.230 : 0.135;
      debug_.tracked_armor_height_m = 0.055;
      break;
    }
  }
  debug_.selected_attitude_valid = false;
  const auto attitude_it = last_attitude_estimates_.find(sel.robot_id);
  if (attitude_it != last_attitude_estimates_.end() && attitude_it->second.valid) {
    debug_.selected_attitude_valid = true;
    debug_.selected_body_pitch_rad = attitude_it->second.pitch_rad;
    debug_.selected_body_roll_rad = attitude_it->second.roll_rad;
  }
}

void Pipeline::updateTrackingControlFrame(
    const rm_interfaces::msg::Armors& armors,
    double sim_time) {
  updateTracking(armors, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), sim_time);
}

rm_interfaces::msg::GimbalCmd Pipeline::computeCommand(double current_yaw,
                                                      double current_pitch,
                                                      double sim_time) {
  gimbal_controller::GimbalControlContext context;
  context.current_time = rclcpp::Time(static_cast<int64_t>(sim_time * 1e9));
  context.current_yaw = current_yaw;
  context.current_pitch = current_pitch;
  context.bullet_speed = bullet_speed_;
  debug_.current_yaw = current_yaw;
  debug_.current_pitch = current_pitch;
  debug_.bullet_speed = bullet_speed_;

  std::string selected_id;
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    auto robots = latest_tracked_robots_;
    selected_id = latest_selected_target_id_;
    const double data_age = sim_time - latest_update_time_;
    const double max_age = std::max(tracker_timeout_s_, 1e-3);
    debug_.data_age = data_age;
    debug_.stale = data_age > max_age;
    debug_.found_selected = false;
    debug_.context_is_tracking = false;
    debug_.context_is_temp_lost = false;

    if (robots && !robots->robots.empty() && data_age <= max_age) {
      const rm_interfaces::msg::TrackedRobot* selected = nullptr;
      if (!selected_id.empty()) {
        for (const auto& r : robots->robots)
          if (r.robot_id == selected_id) { selected = &r; break; }
      } else {
        // Unselected fallback: first robot NOT on the block list. Without the
        // same exclusion the selector applies, a blocked target (e.g. outpost
        // while its handling is disabled) could sneak back in through this
        // path.
        const auto& blocked = selection_config_.blocked_robot_ids;
        for (const auto& r : robots->robots) {
          if (std::find(blocked.begin(), blocked.end(), r.robot_id) != blocked.end()) {
            continue;
          }
          selected = &r;
          selected_id = r.robot_id;
          break;
        }
      }
      if (selected) {
        context.target_robot = *selected;
        context.target_stamp = rclcpp::Time(static_cast<int64_t>(latest_update_time_ * 1e9));
        context.is_tracking = (selected->track_state == rm_interfaces::msg::TrackedRobot::TRACKING);
        context.is_temp_lost = (selected->track_state == rm_interfaces::msg::TrackedRobot::TEMP_LOST);
        auto* tracker = tracker_manager_->get(selected->robot_id);
        context.is_maneuvering = (tracker && tracker->is_initialized())
            ? tracker->assess_maneuver().is_maneuvering : false;
        debug_.found_selected = true;
        debug_.context_is_tracking = context.is_tracking;
        debug_.context_is_temp_lost = context.is_temp_lost;
      }
    }
  }

  const auto result = gimbal_control_core_->compute(
      context, current_gimbal_strategy_name_, selected_id, true);
  debug_.command = result.cmd;
  debug_.delay_audit = result.delay_audit;
  debug_.control_target = result.control_target_debug;
  debug_.mpc = result.mpc_debug;
  debug_.fire_advice = result.fire_advice_debug;
  return result.cmd;
}

void Pipeline::updateFov(double fov_half_yaw, double fov_half_pitch) {
  if (gimbal_control_core_ && std::isfinite(fov_half_yaw) &&
      std::isfinite(fov_half_pitch) && fov_half_yaw > 0.0 &&
      fov_half_pitch > 0.0) {
    gimbal_control_core_->updateFov(fov_half_yaw, fov_half_pitch);
  }
}

}  // namespace hfut::pipeline
