// Lifted from the gimbal_pipeline_node.cpp constructor controller-config region
// (lines ~464-997): the gimbal_controller setter calls. Wrapped as
// Pipeline::configureControllerFromParams(). Param helpers from param_helpers.hpp.
#include "pipeline.hpp"
#include "param_helpers.hpp"

#include <algorithm>
#include <cmath>

#include "gimbal_controller/strategies/current_position_strategy.hpp"
#include "gimbal_controller/strategies/predicted_position_strategy.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"
#include "gimbal_controller/strategies/state_machine_strategy.hpp"

namespace hfut::pipeline {

using namespace fyt::auto_aim;

void Pipeline::configureControllerFromParams() {

  // Configure solver parameters
  double shooting_range_w = get_parameter("controller.solver.shooting_range_width").as_double();
  double shooting_range_h = get_parameter("controller.solver.shooting_range_height").as_double();
  const double min_shooting_angle_deg =
      get_parameter("controller.solver.min_shooting_angle").as_double();
  double side_angle = get_parameter("controller.solver.side_angle").as_double();
  double min_switching_v_yaw = get_parameter("controller.solver.min_switching_v_yaw").as_double();
  double prediction_delay = readUnifiedDoubleParameter(
    *this,
    "controller.delay.prediction_extra_s",
    {
      "controller.solver.prediction_delay",
      "solver.prediction_delay",
      "controller.state_machine.prediction_delay",
      "state_machine.prediction_delay",
      "controller.mpc.prediction_delay_s",
      "mpc.prediction_delay_s"
    });
  double max_prediction_time = readCompatDoubleParameter(
    *this, "controller.solver.max_prediction_time", "solver.max_prediction_time");
  double max_tracking_v_yaw = get_parameter("controller.solver.max_tracking_v_yaw").as_double();
  int transfer_thresh = get_parameter("controller.solver.transfer_thresh").as_int();
  double gravity = get_parameter("controller.solver.gravity").as_double();
  double resistance = get_parameter("controller.solver.resistance").as_double();
  int iteration_times = get_parameter("controller.solver.iteration_times").as_int();
  // 自瞄轨迹规划器（轨迹视角）：兵种适配只需云台角加速度上限
  gimbal_controller::AimTrajectoryPlannerConfig aim_planner_cfg;
  aim_planner_cfg.enable =
      get_parameter("controller.aim_planner.enable").as_bool();
  aim_planner_cfg.max_yaw_acc =
      get_parameter("controller.aim_planner.max_yaw_acc").as_double();
  aim_planner_cfg.max_pitch_acc =
      get_parameter("controller.aim_planner.max_pitch_acc").as_double();
  aim_planner_cfg.q_angle =
      get_parameter("controller.aim_planner.q_angle").as_double();
  aim_planner_cfg.q_rate =
      get_parameter("controller.aim_planner.q_rate").as_double();
  aim_planner_cfg.r_acc =
      get_parameter("controller.aim_planner.r_acc").as_double();
  aim_planner_cfg.half_horizon =
      get_parameter("controller.aim_planner.half_horizon").as_int();
  aim_planner_cfg.dt =
      get_parameter("controller.aim_planner.dt").as_double();
  aim_planner_cfg.fire_delay_s =
      get_parameter("controller.aim_planner.fire_delay_s").as_double();
  aim_planner_cfg.fire_thresh =
      get_parameter("controller.aim_planner.fire_thresh").as_double();
  double pitch_offset = get_parameter("controller.solver.pitch_offset").as_double();
  double yaw_offset = get_parameter("controller.solver.yaw_offset").as_double();
  double facing_enter_angle = get_parameter("controller.solver.facing_enter_angle").as_double();
  double facing_exit_angle = get_parameter("controller.solver.facing_exit_angle").as_double();
  double switch_movement_margin = get_parameter("controller.solver.switch_movement_margin").as_double();
  bool radial_dynamic_enable = get_parameter("controller.solver.radial_dynamic.enable").as_bool();
  double radial_dynamic_v_yaw_ref = get_parameter("controller.solver.radial_dynamic.v_yaw_ref").as_double();
  double radial_dynamic_shrink_ratio = get_parameter("controller.solver.radial_dynamic.shrink_ratio").as_double();
  double radial_dynamic_min_angle_deg = get_parameter("controller.solver.radial_dynamic.min_angle_deg").as_double();
  double radial_dynamic_bias_gain_deg = get_parameter("controller.solver.radial_dynamic.bias_gain_deg").as_double();
  double radial_dynamic_max_bias_deg = get_parameter("controller.solver.radial_dynamic.max_bias_deg").as_double();
  bool virtual_auto_switch_enable = get_parameter("controller.solver.virtual_pose.auto_switch.enable").as_bool();
  double virtual_auto_switch_enter_vyaw =
    get_parameter("controller.solver.virtual_pose.auto_switch.enter_vyaw").as_double();
  double virtual_auto_switch_exit_vyaw =
    get_parameter("controller.solver.virtual_pose.auto_switch.exit_vyaw").as_double();
  std::string virtual_auto_switch_method_str =
    get_parameter("controller.solver.virtual_pose.auto_switch.selection_method").as_string();
  int virtual_auto_switch_fixed_id =
    get_parameter("controller.solver.virtual_pose.auto_switch.fixed_id").as_int();
  int virtual_fixed_id = get_parameter("controller.solver.virtual_pose.fixed_id").as_int();
  double sp_vision_low_speed_vyaw =
    get_parameter("controller.solver.sp_vision.low_speed_vyaw").as_double();
  double sp_vision_shootable_angle_deg =
    get_parameter("controller.solver.sp_vision.shootable_angle_deg").as_double();
  double sp_vision_coming_angle_deg =
    get_parameter("controller.solver.sp_vision.coming_angle_deg").as_double();
  double sp_vision_leaving_angle_deg =
    get_parameter("controller.solver.sp_vision.leaving_angle_deg").as_double();
  double sp_vision_outpost_coming_angle_deg =
    get_parameter("controller.solver.sp_vision.outpost_coming_angle_deg").as_double();
  double sp_vision_outpost_leaving_angle_deg =
    get_parameter("controller.solver.sp_vision.outpost_leaving_angle_deg").as_double();
  bool sp_vision_hold_current_until_jump =
    get_parameter("controller.solver.sp_vision.hold_current_until_jump").as_bool();
  bool sp_vision_zero_speed_fallback =
    get_parameter("controller.solver.sp_vision.zero_speed_fallback").as_bool();
  double controller_delay = readUnifiedDoubleParameter(
    *this,
    "controller.delay.control_latency_s",
    {
      "controller.solver.controller_delay",
      "solver.controller_delay",
      "controller.mpc.control_delay_s",
      "mpc.control_delay_s"
    });
  double trigger_to_muzzle_s = readUnifiedDoubleParameter(
    *this,
    "controller.delay.trigger_to_muzzle_s",
    {
      "controller.fire.trigger_to_muzzle_s",
      "controller.solver.trigger_to_muzzle_s",
      "solver.trigger_to_muzzle_s"
    });
  double max_processing_delay_s = readUnifiedDoubleParameter(
    *this,
    "controller.delay.max_processing_delay_s",
    {
      "controller.mpc.max_processing_delay_s",
      "mpc.max_processing_delay_s"
    });
  std::string selection_method_str = get_parameter("controller.solver.selection_method").as_string();
  std::string fire_policy = get_parameter("controller.fire.decision_policy").as_string();
  int fire_flight_time_iters = readUnifiedIntParameter(
    *this,
    "controller.delay.flight_time_iters",
    {
      "controller.fire.flight_time_iters",
      "controller.mpc.flight_time_iters",
      "mpc.flight_time_iters"
    });
  double fire_facing_filter_opening_angle_deg =
    get_parameter("controller.fire.facing_filter_opening_angle_deg").as_double();
  bool fire_use_gimbal_kinematics =
    get_parameter("controller.fire.use_gimbal_kinematics").as_bool();
  const bool fire_velocity_low_pass_enable =
    get_parameter("controller.fire.velocity_low_pass.enable").as_bool();
  const double fire_velocity_low_pass_alpha =
    get_parameter("controller.fire.velocity_low_pass.alpha").as_double();
  const double fire_velocity_low_pass_reset_timeout_s =
    get_parameter("controller.fire.velocity_low_pass.reset_timeout_s").as_double();
  const bool fire_probability_enable =
    get_parameter("controller.fire.probability.enable").as_bool();
  const double fire_probability_window_ms =
    get_parameter("controller.fire.probability.future_window_ms").as_double();
  const double fire_probability_step_ms =
    get_parameter("controller.fire.probability.future_step_ms").as_double();
  const std::string fire_probability_window_fusion =
    get_parameter("controller.fire.probability.window_fusion").as_string();
  const double fire_probability_softmax_beta =
    get_parameter("controller.fire.probability.softmax_beta").as_double();
  const std::string fire_probability_gate_strategy =
    get_parameter("controller.fire.probability.gate.strategy").as_string();
  const int fire_probability_burst_count =
    get_parameter("controller.fire.probability.burst.burst_bullet_count").as_int();
  const int fire_probability_min_hit_count =
    get_parameter("controller.fire.probability.burst.min_hit_count").as_int();
  const double fire_probability_ref_p0 =
    get_parameter("controller.fire.probability.evidence.reference_probability_p0").as_double();
  const double fire_probability_evidence_window_ms =
    get_parameter("controller.fire.probability.evidence.window_ms").as_double();
  const double fire_probability_evidence_log_clip =
    get_parameter("controller.fire.probability.evidence.log_clip").as_double();
  const double fire_probability_evidence_epsilon =
    get_parameter("controller.fire.probability.evidence.epsilon").as_double();
  const bool fire_probability_neutralize_unshootable_samples =
    get_parameter("controller.fire.probability.evidence.neutralize_unshootable_samples").as_bool();
  const double fire_probability_negative_evidence_scale =
    get_parameter("controller.fire.probability.evidence.negative_evidence_scale").as_double();
  const double fire_probability_negative_clip_scale =
    get_parameter("controller.fire.probability.evidence.negative_clip_scale").as_double();
  const double fire_probability_evidence_deadband =
    get_parameter("controller.fire.probability.evidence.deadband").as_double();
  const double fire_probability_temperature =
    get_parameter("controller.fire.probability.temperature.value").as_double();
  const double fire_probability_theta_on_cold =
    get_parameter("controller.fire.probability.temperature.theta_on_cold").as_double();
  const double fire_probability_theta_on_hot =
    get_parameter("controller.fire.probability.temperature.theta_on_hot").as_double();
  const double fire_probability_theta_hold_cold =
    get_parameter("controller.fire.probability.temperature.theta_hold_cold").as_double();
  const double fire_probability_theta_hold_hot =
    get_parameter("controller.fire.probability.temperature.theta_hold_hot").as_double();
  const double fire_probability_theta_reset_cold =
    get_parameter("controller.fire.probability.temperature.theta_reset_cold").as_double();
  const double fire_probability_theta_reset_hot =
    get_parameter("controller.fire.probability.temperature.theta_reset_hot").as_double();
  const double fire_probability_min_fire_ms =
    get_parameter("controller.fire.probability.commit.min_fire_ms").as_double();
  const double fire_probability_cooldown_ms =
    get_parameter("controller.fire.probability.commit.cooldown_ms").as_double();
  fire_prob_vis_enable_ = get_parameter("controller.fire.visualization.enable").as_bool();
  fire_prob_vis_ellipse_samples_ =
    get_parameter("controller.fire.visualization.ellipse_samples").as_int();
  fire_prob_vis_max_impact_points_ =
    get_parameter("controller.fire.visualization.max_impact_points").as_int();
  fire_prob_image_debug_enable_ =
    get_parameter("controller.fire.visualization.image_debug.enable").as_bool();
  fire_prob_image_debug_publish_rate_hz_ = std::max(
    get_parameter("controller.fire.visualization.image_debug.publish_rate_hz").as_double(), 0.1);
  fire_prob_image_debug_width_ = std::max(
    static_cast<int>(get_parameter("controller.fire.visualization.image_debug.width").as_int()), 320);
  fire_prob_image_debug_height_ = std::max(
    static_cast<int>(get_parameter("controller.fire.visualization.image_debug.height").as_int()), 240);
  fire_prob_image_debug_show_text_ =
    get_parameter("controller.fire.visualization.image_debug.show_text").as_bool();
  fire_prob_image_debug_show_sigma_ellipse_ =
    get_parameter("controller.fire.visualization.image_debug.show_sigma_ellipse").as_bool();
  fire_prob_image_debug_show_velocity_fan_ =
    get_parameter("controller.fire.visualization.image_debug.show_velocity_fan").as_bool();
  const std::string fire_target_visibility_policy =
    get_parameter("controller.fire.target_visibility_policy").as_string();

  if (fire_target_visibility_policy == "facing_only") {
    const bool opening_overridden =
      this->hasParameterOverride("controller.fire.facing_filter_opening_angle_deg");
    if (!opening_overridden || fire_facing_filter_opening_angle_deg >= 180.0 - 1e-9) {
      fire_facing_filter_opening_angle_deg = std::clamp(2.0 * facing_exit_angle, 0.0, 180.0);
    }
  } else if (fire_target_visibility_policy == "legacy_all") {
    fire_facing_filter_opening_angle_deg = 180.0;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Unknown controller.fire.target_visibility_policy='%s', fallback to facing_only.",
      fire_target_visibility_policy.c_str());
    fire_facing_filter_opening_angle_deg = std::clamp(2.0 * facing_exit_angle, 0.0, 180.0);
  }

  facing_enter_angle_deg_ = facing_enter_angle;
  facing_exit_angle_deg_ = facing_exit_angle;
  radial_dynamic_enable_ = radial_dynamic_enable;
  radial_dynamic_v_yaw_ref_ = std::max(radial_dynamic_v_yaw_ref, 1e-6);
  radial_dynamic_shrink_ratio_ = std::clamp(radial_dynamic_shrink_ratio, 0.0, 1.0);
  radial_dynamic_min_angle_deg_ = std::max(radial_dynamic_min_angle_deg, 0.0);
  radial_dynamic_bias_gain_deg_ = std::max(radial_dynamic_bias_gain_deg, 0.0);
  radial_dynamic_max_bias_deg_ = std::max(radial_dynamic_max_bias_deg, 0.0);
  virtual_auto_switch_enable_ = virtual_auto_switch_enable;
  mpc_dt_debug_ = std::max(get_parameter("controller.mpc.dt").as_double(), 1e-4);

  armor_selector_->setParameters(side_angle, min_switching_v_yaw);
  armor_selector_->setFacingParameters(facing_enter_angle, facing_exit_angle);
  armor_selector_->setLowSpinFacingParameters(
      get_parameter("controller.solver.facing_enter_low_spin_angle").as_double(),
      get_parameter("controller.solver.facing_exit_low_spin_angle").as_double());
  armor_selector_->setSwitchMovementMargin(switch_movement_margin);
  armor_selector_->setRadialDynamicParameters(
    radial_dynamic_enable,
    radial_dynamic_v_yaw_ref,
    radial_dynamic_shrink_ratio,
    radial_dynamic_min_angle_deg,
    radial_dynamic_bias_gain_deg,
    radial_dynamic_max_bias_deg);
  armor_selector_->setVirtualPoseParameters(
    virtual_auto_switch_enable,
    virtual_auto_switch_enter_vyaw,
    virtual_auto_switch_exit_vyaw);
  gimbal_controller::ArmorSelector::SelectionMethod auto_switch_method =
    gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_POSE;
  if (virtual_auto_switch_method_str == "virtual_fixed_id") {
    auto_switch_method = gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_FIXED_ID;
  }
  armor_selector_->setVirtualAutoSwitchMethod(auto_switch_method);
  armor_selector_->setVirtualAutoSwitchFixedId(virtual_auto_switch_fixed_id);
  armor_selector_->setVirtualFixedId(virtual_fixed_id);
  armor_selector_->setSpVisionParameters(
    sp_vision_low_speed_vyaw,
    sp_vision_shootable_angle_deg,
    sp_vision_coming_angle_deg,
    sp_vision_leaving_angle_deg,
    sp_vision_outpost_coming_angle_deg,
    sp_vision_outpost_leaving_angle_deg,
    sp_vision_hold_current_until_jump,
    sp_vision_zero_speed_fallback);

  // 配置选板策略
  gimbal_controller::ArmorSelector::SelectionMethod sel_method =
    gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_FACING;
  if (selection_method_str == "min_movement") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT;
  } else if (selection_method_str == "min_movement_with_radial") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_RADIAL;
  } else if (selection_method_str == "decision_angle") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::DECISION_ANGLE;
  } else if (selection_method_str == "virtual_pose") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_POSE;
  } else if (selection_method_str == "virtual_fixed_id") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::VIRTUAL_FIXED_ID;
  } else if (selection_method_str == "facing_or_virtual_pose") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::FACING_OR_VIRTUAL_POSE;
  } else if (selection_method_str == "facing_or_virtual_fixed_id") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::FACING_OR_VIRTUAL_FIXED_ID;
  } else if (selection_method_str == "sp_vision_25" || selection_method_str == "sp_vision") {
    sel_method = gimbal_controller::ArmorSelector::SelectionMethod::SP_VISION_25;
  }
  armor_selector_->setSelectionMethod(sel_method);
  radial_selection_enabled_ =
    (sel_method == gimbal_controller::ArmorSelector::SelectionMethod::MIN_MOVEMENT_WITH_RADIAL);
  RCLCPP_INFO(get_logger(), "[GimbalController] selection_method: %s", selection_method_str.c_str());
  RCLCPP_INFO(
    get_logger(),
    "[DelayUnified] pred_extra=%.4fs ctrl_latency=%.4fs trig2muzzle=%.4fs max_proc=%.4fs iters=%d",
    prediction_delay,
    controller_delay,
    trigger_to_muzzle_s,
    max_processing_delay_s,
    fire_flight_time_iters);
  RCLCPP_INFO(
    get_logger(),
    "[FireVisibility] policy=%s opening=%.2fdeg use_kinematics=%s vel_lpf=%s alpha=%.2f reset=%.3fs",
    fire_target_visibility_policy.c_str(),
    fire_facing_filter_opening_angle_deg,
    fire_use_gimbal_kinematics ? "true" : "false",
    fire_velocity_low_pass_enable ? "true" : "false",
    std::clamp(fire_velocity_low_pass_alpha, 0.0, 1.0),
    std::max(fire_velocity_low_pass_reset_timeout_s, 0.0));

  fire_advisor_->setParameters(shooting_range_w, shooting_range_h,
                               min_shooting_angle_deg);
  if (fire_policy == "ellipse") {
    fire_advisor_->setDecisionPolicy(
      std::make_shared<gimbal_controller::EllipseFireDecisionPolicy>());
  } else {
    fire_advisor_->setDecisionPolicy(
      std::make_shared<gimbal_controller::AxisThresholdFireDecisionPolicy>());
  }
  if (fire_advice_engine_) {
    fire_advice_engine_->setFlightTimeIterations(fire_flight_time_iters);
    fire_advice_engine_->setFacingFilterOpeningAngleDeg(fire_facing_filter_opening_angle_deg);
    fire_advice_engine_->setUseGimbalKinematics(fire_use_gimbal_kinematics);
    gimbal_controller::FireAdviceVelocityLowPassConfig velocity_filter_cfg;
    velocity_filter_cfg.enable = fire_velocity_low_pass_enable;
    velocity_filter_cfg.alpha = fire_velocity_low_pass_alpha;
    velocity_filter_cfg.reset_timeout_s = fire_velocity_low_pass_reset_timeout_s;
    fire_advice_engine_->setVelocityLowPassConfig(velocity_filter_cfg);
    gimbal_controller::fire_advice::ProbabilityConfig prob_cfg;
    prob_cfg.enable = fire_probability_enable;
    prob_cfg.future_window_ms = std::max(fire_probability_window_ms, 0.0);
    prob_cfg.future_step_ms = std::max(fire_probability_step_ms, 1.0);
    prob_cfg.softmax_fusion = (fire_probability_window_fusion == "softmax");
    prob_cfg.softmax_beta = fire_probability_softmax_beta;
    prob_cfg.use_tracker_covariance =
      get_parameter("controller.fire.probability.use_tracker_covariance").as_bool();
    prob_cfg.strict_covariance =
      get_parameter("controller.fire.probability.strict_covariance").as_bool();
    prob_cfg.fallback_sigma_x =
      get_parameter("controller.fire.probability.fallback_sigma_x").as_double();
    prob_cfg.fallback_sigma_y =
      get_parameter("controller.fire.probability.fallback_sigma_y").as_double();
    prob_cfg.fallback_sigma_z =
      get_parameter("controller.fire.probability.fallback_sigma_z").as_double();
    prob_cfg.sigma_x0 =
      get_parameter("controller.fire.probability.ballistic_sigma_x0").as_double();
    prob_cfg.sigma_y0 =
      get_parameter("controller.fire.probability.ballistic_sigma_y0").as_double();
    prob_cfg.sigma_z0 =
      get_parameter("controller.fire.probability.ballistic_sigma_z0").as_double();
    prob_cfg.growth_x =
      get_parameter("controller.fire.probability.ballistic_growth_x").as_double();
    prob_cfg.growth_y =
      get_parameter("controller.fire.probability.ballistic_growth_y").as_double();
    prob_cfg.growth_z =
      get_parameter("controller.fire.probability.ballistic_growth_z").as_double();
    prob_cfg.enable_normal_velocity_weight =
      get_parameter("controller.fire.probability.normal_velocity_weight.enable").as_bool();
    prob_cfg.normal_v_ref =
      get_parameter("controller.fire.probability.normal_velocity_weight.v_ref").as_double();
    prob_cfg.normal_w_min =
      get_parameter("controller.fire.probability.normal_velocity_weight.w_min").as_double();
    prob_cfg.enable_normal_velocity_gate =
      get_parameter("controller.fire.probability.normal_velocity_gate.enable").as_bool();
    prob_cfg.require_front_face =
      get_parameter("controller.fire.probability.normal_velocity_gate.require_front_face").as_bool();
    prob_cfg.normal_v_activate_min =
      get_parameter("controller.fire.probability.normal_velocity_gate.v_activate_min").as_double();
    prob_cfg.front_face_epsilon =
      get_parameter("controller.fire.probability.normal_velocity_gate.front_epsilon").as_double();
    prob_cfg.max_complement_angle_deg =
      get_parameter("controller.fire.probability.normal_velocity_gate.max_complement_angle_deg").as_double();
    // Reuse solver hitbox size as probability hit rectangle in SI meters.
    prob_cfg.armor_width_m = std::max(shooting_range_w, 1e-6);
    prob_cfg.armor_height_m = std::max(shooting_range_h, 1e-6);

    gimbal_controller::fire_advice::SigmaPointConfig sigma_cfg;
    sigma_cfg.enable = get_parameter("controller.fire.probability.sigma_point.enable").as_bool();
    sigma_cfg.use_unscented =
      get_parameter("controller.fire.probability.sigma_point.method").as_string() == "unscented";
    sigma_cfg.sigma_v0 =
      get_parameter("controller.fire.probability.sigma_point.sigma_v0").as_double();
    sigma_cfg.sigma_delay =
      get_parameter("controller.fire.probability.sigma_point.sigma_delay").as_double();
    sigma_cfg.rho = get_parameter("controller.fire.probability.sigma_point.rho").as_double();
    sigma_cfg.alpha = get_parameter("controller.fire.probability.sigma_point.alpha").as_double();
    sigma_cfg.beta = get_parameter("controller.fire.probability.sigma_point.beta").as_double();
    sigma_cfg.kappa = get_parameter("controller.fire.probability.sigma_point.kappa").as_double();

    gimbal_controller::fire_advice::FireGateConfig gate_cfg;
    if (fire_probability_gate_strategy == "burst_evidence") {
      gate_cfg.strategy = gimbal_controller::fire_advice::FireGateConfig::Strategy::kBurstEvidence;
    } else if (fire_probability_gate_strategy == "legacy") {
      gate_cfg.strategy = gimbal_controller::fire_advice::FireGateConfig::Strategy::kLegacy;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Unknown controller.fire.probability.gate.strategy='%s', fallback to legacy.",
        fire_probability_gate_strategy.c_str());
      gate_cfg.strategy = gimbal_controller::fire_advice::FireGateConfig::Strategy::kLegacy;
    }
    gate_cfg.integrator_mode =
      get_parameter("controller.fire.probability.gate.mode").as_string() == "integrator";
    gate_cfg.alpha = get_parameter("controller.fire.probability.gate.alpha").as_double();
    gate_cfg.fire_on_th = get_parameter("controller.fire.probability.gate.fire_on_th").as_double();
    gate_cfg.fire_off_th = get_parameter("controller.fire.probability.gate.fire_off_th").as_double();
    gate_cfg.integrator_base_probability =
      get_parameter("controller.fire.probability.gate.integrator_base_probability").as_double();
    gate_cfg.integrator_rise =
      get_parameter("controller.fire.probability.gate.integrator_rise").as_double();
    gate_cfg.integrator_fall =
      get_parameter("controller.fire.probability.gate.integrator_fall").as_double();
    gate_cfg.burst_bullet_count = std::max(fire_probability_burst_count, 1);
    gate_cfg.min_hit_count = std::max(fire_probability_min_hit_count, 1);
    gate_cfg.reference_probability_p0 = fire_probability_ref_p0;
    gate_cfg.evidence_window_ms = std::max(fire_probability_evidence_window_ms, 0.0);
    gate_cfg.log_evidence_clip = fire_probability_evidence_log_clip;
    gate_cfg.evidence_epsilon = fire_probability_evidence_epsilon;
    gate_cfg.neutralize_unshootable_samples = fire_probability_neutralize_unshootable_samples;
    gate_cfg.negative_evidence_scale = fire_probability_negative_evidence_scale;
    gate_cfg.negative_clip_scale = fire_probability_negative_clip_scale;
    gate_cfg.evidence_deadband = fire_probability_evidence_deadband;
    gate_cfg.temperature = fire_probability_temperature;
    gate_cfg.theta_on_cold = fire_probability_theta_on_cold;
    gate_cfg.theta_on_hot = fire_probability_theta_on_hot;
    gate_cfg.theta_hold_cold = fire_probability_theta_hold_cold;
    gate_cfg.theta_hold_hot = fire_probability_theta_hold_hot;
    gate_cfg.theta_reset_cold = fire_probability_theta_reset_cold;
    gate_cfg.theta_reset_hot = fire_probability_theta_reset_hot;
    gate_cfg.min_fire_ms = std::max(fire_probability_min_fire_ms, 0.0);
    gate_cfg.cooldown_ms = std::max(fire_probability_cooldown_ms, 0.0);

    fire_advice_engine_->setProbabilityConfig(prob_cfg, sigma_cfg, gate_cfg);
  }
  if (gimbal_control_core_) {
    gimbal_controller::FireDecisionConfig fire_cfg;
    fire_cfg.prediction_delay_s = std::max(prediction_delay, 0.0);
    fire_cfg.control_latency_s = std::max(controller_delay, 0.0);
    fire_cfg.trigger_to_muzzle_s = std::max(trigger_to_muzzle_s, 0.0);
    fire_cfg.max_processing_delay_s = std::max(max_processing_delay_s, 0.0);
    fire_cfg.yaw_offset_rad = yaw_offset * M_PI / 180.0;
    fire_cfg.pitch_offset_rad = pitch_offset * M_PI / 180.0;
    fire_cfg.include_processing_delay = true;
    fire_cfg.include_control_latency_in_target_prediction = false;
    fire_cfg.min_consecutive_frames =
        get_parameter("controller.fire.min_consecutive_frames").as_int();
    gimbal_control_core_->setFireDecisionConfig(fire_cfg);
  }
  local_compensator_->setParameters(bullet_speed_, gravity, resistance,
                                    iteration_times);

  // Strategy-specific configuration
  auto predicted_strategy = std::dynamic_pointer_cast<
      gimbal_controller::PredictedPositionStrategy>(
      gimbal_strategies_["predicted"]);
  if (predicted_strategy) {
    predicted_strategy->setPredictionParameters(prediction_delay, max_prediction_time);
    predicted_strategy->setMaxProcessingDelay(max_processing_delay_s);
    predicted_strategy->setManualOffset(pitch_offset, yaw_offset);
    predicted_strategy->setTrackingCenterParams(max_tracking_v_yaw, transfer_thresh);
    predicted_strategy->setControllerDelay(controller_delay);
    predicted_strategy->setTriggerToMuzzleDelay(trigger_to_muzzle_s);
    predicted_strategy->setAimPlannerConfig(aim_planner_cfg);
  }
  auto current_strategy = std::dynamic_pointer_cast<
      gimbal_controller::CurrentPositionStrategy>(
      gimbal_strategies_["current"]);
  if (current_strategy) {
    current_strategy->setManualOffset(pitch_offset, yaw_offset);
    current_strategy->setControllerDelay(controller_delay);
    current_strategy->setMaxProcessingDelay(max_processing_delay_s);
    current_strategy->setTriggerToMuzzleDelay(trigger_to_muzzle_s);
  }

  // Configure adaptive controller_delay (AIMD)
  bool   adaptive_enable   = get_parameter("controller.solver.adaptive_delay.enable").as_bool();
  int    adaptive_threshold = get_parameter("controller.solver.adaptive_delay.fire_wait_threshold").as_int();
  double adaptive_mul      = get_parameter("controller.solver.adaptive_delay.mul_factor").as_double();
  double adaptive_step     = get_parameter("controller.solver.adaptive_delay.add_step").as_double();
  double adaptive_max      = get_parameter("controller.solver.adaptive_delay.max_delay").as_double();
  double adaptive_min      = get_parameter("controller.solver.adaptive_delay.min_delay").as_double();
  double adaptive_max_lin  = get_parameter("controller.solver.adaptive_delay.max_linear_speed").as_double();
  double adaptive_max_ang  = get_parameter("controller.solver.adaptive_delay.max_angular_speed").as_double();

  RCLCPP_INFO(get_logger(),
    "[GimbalController] adaptive_delay: %s (init=%.4f s, min=%.4f, max=%.4f, "
    "add_step=%.4f, mul=%.2f, thresh=%d, max_lin=%.1f, max_ang=%.1f)",
    adaptive_enable ? "ENABLED" : "disabled",
    controller_delay, adaptive_min, adaptive_max,
    adaptive_step, adaptive_mul, adaptive_threshold,
    adaptive_max_lin, adaptive_max_ang);

  if (predicted_strategy) {
    predicted_strategy->setAdaptiveDelayParams(
      adaptive_enable, controller_delay,
      adaptive_min, adaptive_max,
      adaptive_step, adaptive_mul, adaptive_threshold,
      adaptive_max_lin, adaptive_max_ang);
  }
  if (current_strategy) {
    current_strategy->setAdaptiveDelayParams(
      adaptive_enable, controller_delay,
      adaptive_min, adaptive_max,
      adaptive_step, adaptive_mul, adaptive_threshold,
      adaptive_max_lin, adaptive_max_ang);
  }

  double sm_facing_enter = get_parameter("controller.state_machine.facing_enter_angle").as_double();
  double sm_facing_exit = get_parameter("controller.state_machine.facing_exit_angle").as_double();
  double sm_spin_thresh = get_parameter("controller.state_machine.spin_v_yaw_thresh").as_double();
  double sm_calm_thresh = get_parameter("controller.state_machine.calm_v_yaw_thresh").as_double();
  int sm_spin_enter = get_parameter("controller.state_machine.spin_enter_count").as_int();
  int sm_spin_exit = get_parameter("controller.state_machine.spin_exit_count").as_int();
  double sm_side_angle = get_parameter("controller.state_machine.side_angle").as_double();
  double sm_prediction_delay = prediction_delay;
  double sm_max_prediction = readCompatDoubleParameter(
    *this,
    "controller.state_machine.max_prediction_time",
    "state_machine.max_prediction_time");

  auto sm_strategy_ptr = std::dynamic_pointer_cast<
      gimbal_controller::StateMachineStrategy>(
      gimbal_strategies_["state_machine"]);
  if (sm_strategy_ptr) {
    sm_strategy_ptr->setFacingParameters(sm_facing_enter, sm_facing_exit);
    sm_strategy_ptr->setSpinParameters(sm_spin_thresh, sm_calm_thresh,
                                       sm_spin_enter, sm_spin_exit,
                                       sm_side_angle);
    sm_strategy_ptr->setPredictionParameters(sm_prediction_delay,
                                             sm_max_prediction);
    sm_strategy_ptr->setMaxProcessingDelay(max_processing_delay_s);
    sm_strategy_ptr->setManualOffset(pitch_offset, yaw_offset);
    sm_strategy_ptr->setTriggerToMuzzleDelay(trigger_to_muzzle_s);
  }

  // ── 配置 GimbalCmd 输出端保护滤波器 ──
  {
    gimbal_controller::GimbalCmdFilterConfig fcfg;
    fcfg.enable_clamping             = get_parameter("controller.output_filter.enable_clamping").as_bool();
    fcfg.max_yaw_diff                = get_parameter("controller.output_filter.max_yaw_diff").as_double();
    fcfg.max_pitch_diff              = get_parameter("controller.output_filter.max_pitch_diff").as_double();
    fcfg.enable_outlier_rejection    = get_parameter("controller.output_filter.enable_outlier_rejection").as_bool();
    fcfg.outlier_threshold_yaw       = get_parameter("controller.output_filter.outlier_threshold_yaw").as_double();
    fcfg.outlier_threshold_pitch     = get_parameter("controller.output_filter.outlier_threshold_pitch").as_double();
    fcfg.max_outlier_count           = get_parameter("controller.output_filter.max_outlier_count").as_int();
    fcfg.enable_rate_limiter         = get_parameter("controller.output_filter.enable_rate_limiter").as_bool();
    fcfg.max_yaw_rate                = get_parameter("controller.output_filter.max_yaw_rate").as_double();
    fcfg.max_pitch_rate              = get_parameter("controller.output_filter.max_pitch_rate").as_double();
    fcfg.enable_moving_average       = get_parameter("controller.output_filter.enable_moving_average").as_bool();
    fcfg.moving_average_window_size  = get_parameter("controller.output_filter.moving_average_window_size").as_int();
    fcfg.enable_ema                  = get_parameter("controller.output_filter.enable_ema").as_bool();
    fcfg.ema_alpha                   = get_parameter("controller.output_filter.ema_alpha").as_double();
    fcfg.enable_one_euro             = get_parameter("controller.output_filter.enable_one_euro").as_bool();
    fcfg.one_euro_freq               = get_parameter("controller.output_filter.one_euro_freq").as_double();
    fcfg.one_euro_min_cutoff         = get_parameter("controller.output_filter.one_euro_min_cutoff").as_double();
    fcfg.one_euro_beta               = get_parameter("controller.output_filter.one_euro_beta").as_double();
    fcfg.one_euro_d_cutoff           = get_parameter("controller.output_filter.one_euro_d_cutoff").as_double();
    if (gimbal_control_core_) {
      gimbal_control_core_->setFilterConfig(fcfg);
      gimbal_control_core_->setIdleHold(
          get_parameter("controller.idle_hold_s").as_double());
    }
    RCLCPP_INFO(get_logger(),
      "[GimbalCmdFilter] clamp=%s(%.1f°,%.1f°) outlier=%s(%.1f°,%.1f°,max%d)"
      " rate=%s(%.1f°,%.1f°) mean=%s(win=%d) ema=%s(a=%.2f) 1euro=%s(f=%.0f,mc=%.2f,b=%.4f)",
      fcfg.enable_clamping ? "ON" : "off", fcfg.max_yaw_diff, fcfg.max_pitch_diff,
      fcfg.enable_outlier_rejection ? "ON" : "off",
      fcfg.outlier_threshold_yaw, fcfg.outlier_threshold_pitch, fcfg.max_outlier_count,
      fcfg.enable_rate_limiter ? "ON" : "off", fcfg.max_yaw_rate, fcfg.max_pitch_rate,
      fcfg.enable_moving_average ? "ON" : "off", fcfg.moving_average_window_size,
      fcfg.enable_ema ? "ON" : "off", fcfg.ema_alpha,
      fcfg.enable_one_euro ? "ON" : "off",
      fcfg.one_euro_freq, fcfg.one_euro_min_cutoff, fcfg.one_euro_beta);
  }
}

}  // namespace hfut::pipeline
