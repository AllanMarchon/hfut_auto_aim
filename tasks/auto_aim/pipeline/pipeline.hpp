// ROS-free orchestration class replacing GimbalPipelineNode. Holds the tracker
// manager, target selector, and gimbal control core, and exposes two entry
// points instead of ROS callbacks:
//   updateTracking(armors, R_cam2world, t_cam2world, sim_time)  // was armorsCallback
//   computeCommand(gimbal_yaw, gimbal_pitch, sim_time)          // was timerCallback
//
// The big mechanical config methods (declare*Parameters, applyTrackerParams-
// ToConfig, initGimbalStrategies) are lifted verbatim from the node into
// pipeline_params.cpp; this class inherits ParameterHost so they compile
// unchanged. Construction reads gimbal_pipeline.yaml.
#ifndef HFUT_PIPELINE_PIPELINE_HPP_
#define HFUT_PIPELINE_PIPELINE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>
#include <rm_interfaces/msg/tracked_robots.hpp>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/tracker_manager.hpp"
#include "max_entropy_tracker/utils/body_attitude_estimator.hpp"
#include "max_entropy_tracker/utils/output_smoother.hpp"

#include "target_selector/selection_strategy.hpp"
#include "target_selector/strategies/min_yaw_deviation_strategy.hpp"
#include "target_selector/strategies/priority_list_strategy.hpp"
#include "target_selector/strategies/sticky_min_yaw_deviation_strategy.hpp"

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/ballistic_solver_client.hpp"
#include "gimbal_controller/fire_advice_engine.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/gimbal_control_core.hpp"
#include "gimbal_controller/gimbal_control_strategy.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"

#include "parameter_host.hpp"

namespace hfut::pipeline {

using fyt::auto_aim::UnifiedConfig;
using fyt::auto_aim::ObservationData;
using fyt::auto_aim::TrackerManager;
using fyt::auto_aim::SmootherConfig;
using fyt::auto_aim::SmoothedOutput;
using fyt::auto_aim::BaseTracker;
using fyt::auto_aim::SelectionStrategyPtr;
using fyt::auto_aim::SelectionConfig;
using fyt::auto_aim::SelectionResult;

struct PipelineOverrides {
  std::optional<double> bullet_speed;
  // Optional per-process controller strategy override (e.g. "mpc") used by
  // simulation regressions without mutating the checked-in default config.
  std::string controller_strategy;
  double observation_noise_scale{1.0};
  double max_temp_lost_prediction_s{0.15};
  // Adaptive TEMP_LOST dead-reckoning: targets moving faster than
  // temp_lost_coast_min_speed_mps_ may coast on the decayed prediction for up
  // to temp_lost_coast_max_s_ instead of the tight stationary timeout.
  double temp_lost_coast_max_s{0.5};
  double temp_lost_coast_min_speed_mps{0.3};
  // Whole-vehicle attitude (roll/pitch) estimation from plate orientations.
  // mount_pitch_rad is the armor mount pitch (nominal 15 deg); use the
  // platform-specific apparent value (webots mesh reads ~7 deg through PnP).
  double attitude_mount_pitch_deg{15.0};
  double attitude_ema_alpha{0.1};
  // Noisy planar PnP attitude remains diagnostic unless explicitly opted in.
  // Trusted full-orientation producers bypass this switch.
  bool attitude_apply_to_geometry{false};
  // Spatial robot-id reassignment radius for detector id misreads (0=off).
  double id_association_max_distance_m{0.6};
  fyt::auto_aim::TrackerMotionGuardParameters motion_guard;
};

class Pipeline : public ParameterHost {
 public:
  // config_path: gimbal_pipeline.yaml (ROS-style). node_name: top YAML key.
  explicit Pipeline(const std::string& config_path,
                    const std::string& node_name = "gimbal_pipeline",
                    const PipelineOverrides& overrides = {});

  // Multi-file form: topic config files deep-merged in order (later wins).
  explicit Pipeline(const std::vector<std::string>& config_paths,
                    const std::string& node_name = "gimbal_pipeline",
                    const PipelineOverrides& overrides = {});

  // Tracker update from one detection frame. Armor poses are in the camera
  // optical frame; R/t transform them to the world (odom) frame, replacing the
  // old tf2 lookup. sim_time is the frame timestamp in seconds.
  void updateTracking(const rm_interfaces::msg::Armors& armors,
                      const Eigen::Matrix3d& R_cam2world,
                      const Eigen::Vector3d& t_cam2world,
                      double sim_time);

  // Direct simulator measurements are already expressed in the shooter-
  // centered, odom-aligned control frame, so no camera/PnP transform applies.
  void updateTrackingControlFrame(const rm_interfaces::msg::Armors& armors,
                                  double sim_time);

  // Control step. Current gimbal feedback (radians) + now (seconds). Returns
  // the gimbal command (angles in DEGREES, per the ROS GimbalCmd convention).
  rm_interfaces::msg::GimbalCmd computeCommand(double current_yaw,
                                               double current_pitch,
                                               double sim_time);

  // Forward the current camera half-FOV (radians) to strategies that use image
  // bounds, mirroring the ROS node's camera_info callback.
  void updateFov(double fov_half_yaw, double fov_half_pitch);

  bool debugMode() const { return debug_mode_; }

  // Last-decision snapshot for diagnostics (filled by updateTracking +
  // computeCommand). Lets bringup_sim print/plot exactly why a frame produced
  // mode=-1 without threading state through the return values.
  struct DebugSnapshot {
    int num_tracked = 0;          // robots in latest tracked_msg
    std::string selected_id;      // selector output
    int selected_track_state = -1;// TrackedRobot::{DETECTING/TRACKING/TEMP_LOST}, -1 if none
    bool context_is_tracking = false;   // what computeCommand fed the controller
    bool context_is_temp_lost = false;
    double data_age = 0.0;        // sim_time - latest_update_time at control
    bool stale = false;           // data_age > tracker_timeout (cache gate tripped)
    bool found_selected = false;  // selected robot present in cache
    bool selected_state_valid = false;
    rm_interfaces::msg::TrackedRobot selected_state;
    bool tracker_structure_valid = false;
    double tracker_r1 = 0.0;
    double tracker_r2 = 0.0;
    double tracker_dza = 0.0;
    bool tracker_update_valid = false;
    bool tracker_update_committed = false;
    int tracker_observation_count = 0;
    double tracker_top1_nis = -1.0;
    double tracker_top1_chi2_pos = -1.0;
    double tracker_top1_chi2_yaw = -1.0;
    std::string tracker_top1_hypothesis;
    std::string tracker_decision_reason;
    // Selected robot's whole-vehicle attitude estimate (invalid when the
    // estimator has not seen enough plates yet).
    bool selected_attitude_valid = false;
    double selected_body_pitch_rad = 0.0;
    double selected_body_roll_rad = 0.0;
    double current_yaw = 0.0;
    double current_pitch = 0.0;
    double bullet_speed = 0.0;
    rm_interfaces::msg::GimbalCmd command;
    gimbal_controller::DelayAuditSnapshot delay_audit;
    gimbal_controller::ControlTargetDebugSnapshot control_target;
    gimbal_controller::MpcDebugSnapshot mpc;
    gimbal_controller::FireAdviceDebugSnapshot fire_advice;
    // Selected/tracked robot's estimated plate geometry for the debug overlay,
    // so the estimator's full-vehicle estimate stays visible while a track is
    // alive — not only in the AIM state (control target valid).
    std::vector<fyt::auto_aim::robot_description::ArmorWorldPose> tracked_armor_poses;
    double tracked_armor_width_m = 0.135;
    double tracked_armor_height_m = 0.055;
  };
  const DebugSnapshot& lastDebug() const { return debug_; }

 private:
  // ── lifted verbatim from the node (pipeline_params.cpp) ──
  void declareTrackerParameters();
  void declareTargetSelectorParameters();
  void declareGimbalControllerParameters();
  void applyTrackerParamsToConfig();
  void initGimbalStrategies();

  // ── setup (pipeline.cpp) ──
  void initGimbalComponents();
  void initSelectionStrategy();
  void configureControllerFromParams();  // condensed from the node ctor tail
  SelectionResult selectTargetInternal(const rm_interfaces::msg::TrackedRobots& robots);
  rm_interfaces::msg::TrackedRobots buildTrackedRobotsMsg(double sim_time);
  // Spatial robot-id association for detector id misreads (see updateTracking).
  std::string assign_robot_id(const std::string& detected_id,
                              const ObservationData& obs) const;

  // ── config / params ──
  UnifiedConfig tracker_config_;
  std::string target_frame_{"odom"};
  std::string source_frame_{"camera_optical_frame"};
  double predict_rate_{20.0};
  bool debug_mode_{false};
  double tracker_timeout_s_{0.5};
  double max_temp_lost_prediction_s_{0.15};
  double temp_lost_coast_max_s_{0.5};
  double temp_lost_coast_min_speed_mps_{0.3};
  double attitude_mount_pitch_rad_{15.0 * M_PI / 180.0};
  double attitude_ema_alpha_{0.1};
  bool attitude_apply_to_geometry_{false};
  // Max center distance for spatial robot-id reassignment (0 = disabled).
  double id_association_max_distance_m_{0.6};
  double bullet_speed_{22.5};
  double control_rate_{250.0};
  bool fire_prob_vis_enable_{false};
  std::string ballistic_mode_{"local"};

  // Debug/state members the lifted controller-config block assigns to. Kept so
  // the verbatim code compiles; the debug-image/marker consumers are dropped.
  double facing_enter_angle_deg_{40.0};
  double facing_exit_angle_deg_{55.0};
  bool fire_prob_image_debug_enable_{false};
  int fire_prob_image_debug_height_{540};
  double fire_prob_image_debug_publish_rate_hz_{10.0};
  bool fire_prob_image_debug_show_sigma_ellipse_{true};
  bool fire_prob_image_debug_show_text_{true};
  bool fire_prob_image_debug_show_velocity_fan_{true};
  int fire_prob_image_debug_width_{960};
  int fire_prob_vis_ellipse_samples_{64};
  int fire_prob_vis_max_impact_points_{120};
  double mpc_dt_debug_{0.01};
  double radial_dynamic_bias_gain_deg_{0.0};
  bool radial_dynamic_enable_{false};
  double radial_dynamic_max_bias_deg_{0.0};
  double radial_dynamic_min_angle_deg_{5.0};
  double radial_dynamic_shrink_ratio_{0.6};
  double radial_dynamic_v_yaw_ref_{8.0};
  bool radial_selection_enabled_{false};
  bool virtual_auto_switch_enable_{false};
  std::string current_gimbal_strategy_name_{"current"};
  std::string selector_strategy_name_{"min_yaw_deviation"};
  std::string current_target_id_;

  // ── chain components ──
  std::unique_ptr<TrackerManager> tracker_manager_;
  std::unique_ptr<fyt::auto_aim::robot_description::RobotDescriptionFacade> robot_description_facade_;
  SmootherConfig smoother_config_;
  SelectionStrategyPtr selection_strategy_;
  SelectionConfig selection_config_;
  // Per-robot whole-vehicle attitude (roll/pitch) estimators fed by plate
  // world orientations; latest estimate per robot id for output/debug.
  std::unordered_map<std::string, fyt::auto_aim::BodyAttitudeEstimator>
      attitude_estimators_;
  std::unordered_map<std::string, fyt::auto_aim::BodyAttitudeEstimate>
      last_attitude_estimates_;

  std::shared_ptr<gimbal_controller::ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<gimbal_controller::ArmorSelector> armor_selector_;
  std::shared_ptr<gimbal_controller::BallisticSolverClient> ballistic_client_;
  std::shared_ptr<gimbal_controller::LocalTrajectoryCompensator> local_compensator_;
  std::shared_ptr<gimbal_controller::FireAdvisor> fire_advisor_;
  std::shared_ptr<gimbal_controller::FireAdviceEngine> fire_advice_engine_;
  std::shared_ptr<gimbal_controller::GimbalControlCore> gimbal_control_core_;
  std::unordered_map<std::string, gimbal_controller::GimbalControlStrategy::SharedPtr>
      gimbal_strategies_;

  // ── shared tracker→control cache (was guarded by ROS threads; here single
  //    threaded but kept for parity) ──
  std::mutex pipeline_mutex_;
  rm_interfaces::msg::TrackedRobots::SharedPtr latest_tracked_robots_;
  std::string latest_selected_target_id_;
  double latest_selected_confidence_{0.0};
  double latest_update_time_{0.0};

  DebugSnapshot debug_;
};

}  // namespace hfut::pipeline

#endif  // HFUT_PIPELINE_PIPELINE_HPP_
