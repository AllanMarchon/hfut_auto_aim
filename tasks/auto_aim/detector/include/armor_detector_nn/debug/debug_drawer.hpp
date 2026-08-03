#ifndef ARMOR_DETECTOR_NN_DEBUG_DRAWER_HPP_
#define ARMOR_DETECTOR_NN_DEBUG_DRAWER_HPP_

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

struct DebugOverlayStatus
{
  bool fire_advice{false};
  bool fire_evaluated{false};
  bool fire_valid{false};
  bool fire_facing_ok{false};
  bool probability_enabled{false};
  bool tracking{false};
  bool temp_lost{false};
  bool stale{false};
  bool tracks_center{false};
  bool virtual_target{false};
  int mode{-1};
  int track_state{-1};
  int detections{0};
  int tracked_targets{0};
  int fire_candidates{0};
  std::string target_id;
  std::string strategy;
  double gimbal_yaw_rad{0.0};
  double gimbal_pitch_rad{0.0};
  double command_yaw_rad{0.0};
  double command_pitch_rad{0.0};
  double yaw_diff_rad{0.0};
  double pitch_diff_rad{0.0};
  double distance_m{0.0};
  double target_speed_mps{0.0};
  double target_yaw_rate_radps{0.0};
  bool body_attitude_valid{false};
  double body_pitch_rad{0.0};
  double body_roll_rad{0.0};
  double r1_m{0.0};
  double r2_m{0.0};
  double prediction_ms{0.0};
  double flight_ms{0.0};
  double data_age_ms{0.0};
  double fire_yaw_error_rad{0.0};
  double fire_pitch_error_rad{0.0};
  double hit_probability{0.0};
  double fps{0.0};
  double loop_latency_ms{0.0};
  double inference_ms{0.0};
  double pose_ms{0.0};
  std::string fire_hold_reason;
};

struct DebugOverlayGeometry
{
  int locked_detection_index{-1};
  bool has_center{false};
  bool has_predicted_center{false};
  bool has_velocity_tip{false};
  bool has_current_selected_armor{false};
  bool has_control_target{false};
  bool has_fire_target{false};
  cv::Point2f center;
  cv::Point2f predicted_center;
  cv::Point2f velocity_tip;
  cv::Point2f current_selected_armor;
  cv::Point2f control_target;
  cv::Point2f fire_target;
  std::vector<std::array<cv::Point2f, 4>> current_armor_outlines;
};

class DebugDrawer {
public:
  DebugDrawer();

  void drawDetections(
    cv::Mat& image,
    const std::vector<ArmorDetection>& detections,
    bool show_confidence = true) const;

  void drawProfiler(
    cv::Mat& image,
    double fps,
    double latency_ms,
    const std::string& backend_name,
    const std::string& precision) const;

  void drawArmorsCount(cv::Mat& image, int count) const;

  void drawCrosshair(cv::Mat& image) const;

  void drawControlOverlay(
    cv::Mat& image,
    const std::vector<ArmorDetection>& detections,
    const DebugOverlayStatus& status,
    const DebugOverlayGeometry& geometry) const;

  void setClassColor(const std::string& label, const cv::Scalar& color);

private:
  static cv::Scalar generateColor(const std::string& label);

  std::unordered_map<std::string, cv::Scalar> class_colors_;
};

}  // namespace fyt::auto_aim

#endif
