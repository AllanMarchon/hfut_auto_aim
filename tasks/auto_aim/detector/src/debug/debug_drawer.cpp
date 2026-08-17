#include "armor_detector_nn/debug/debug_drawer.hpp"

#include <iomanip>
#include <algorithm>
#include <cmath>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace fyt::auto_aim {

namespace {

const cv::Scalar kLockColor(255, 0, 255);
const cv::Scalar kCenterColor(255, 255, 0);
const cv::Scalar kPredictionColor(255, 190, 0);
const cv::Scalar kCurrentArmorColor(40, 255, 80);
const cv::Scalar kCurrentArmorLinkColor(20, 170, 60);
const cv::Scalar kFireTargetColor(0, 255, 255);

std::string fixed(double value, int precision = 1)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

void putTextOutlined(
  cv::Mat & image, const std::string & text, const cv::Point & origin,
  double scale, const cv::Scalar & color, int thickness = 1)
{
  cv::putText(
    image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
    cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
  cv::putText(
    image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
    color, thickness, cv::LINE_AA);
}

void drawDiamond(
  cv::Mat & image, const cv::Point2f & point, int radius,
  const cv::Scalar & color, int thickness)
{
  const cv::Point p(cvRound(point.x), cvRound(point.y));
  const std::vector<cv::Point> diamond = {
    cv::Point(p.x, p.y - radius), cv::Point(p.x + radius, p.y),
    cv::Point(p.x, p.y + radius), cv::Point(p.x - radius, p.y)};
  cv::polylines(image, diamond, true, color, thickness, cv::LINE_AA);
}

const char * trackStateName(int state)
{
  switch (state) {
    case 0: return "DETECTING";
    case 1: return "TRACKING";
    case 2: return "TEMP_LOST";
    case 3: return "LOST";
    default: return "NONE";
  }
}

const char * modeName(int mode)
{
  switch (mode) {
    case 1: return "NORMAL";
    case 0: return "UNKNOWN";
    case -1: return "NO_MEAS";
    default: return "UNKNOWN";
  }
}

}  // namespace

DebugDrawer::DebugDrawer() {
  // Pre-assign colors for common labels
  class_colors_["B1"] = cv::Scalar(255, 100, 100);
  class_colors_["B2"] = cv::Scalar(255, 80, 80);
  class_colors_["B3"] = cv::Scalar(200, 60, 60);
  class_colors_["B4"] = cv::Scalar(200, 40, 40);
  class_colors_["B5"] = cv::Scalar(180, 20, 20);
  class_colors_["BO"] = cv::Scalar(150, 50, 50);
  class_colors_["BS"] = cv::Scalar(180, 80, 80);
  class_colors_["R1"] = cv::Scalar(100, 100, 255);
  class_colors_["R2"] = cv::Scalar(80, 80, 255);
  class_colors_["R3"] = cv::Scalar(60, 60, 200);
  class_colors_["R4"] = cv::Scalar(40, 40, 200);
  class_colors_["R5"] = cv::Scalar(20, 20, 180);
  class_colors_["RO"] = cv::Scalar(50, 50, 150);
  class_colors_["RS"] = cv::Scalar(80, 80, 180);
}

void DebugDrawer::drawDetections(
    cv::Mat& image,
    const std::vector<ArmorDetection>& detections,
    bool show_confidence) const
{
  for (const auto& d : detections) {
    cv::Scalar color = generateColor(d.publish_number + "_" +
        (d.color == fyt::EnemyColor::RED ? "R" : "B"));

    std::vector<cv::Point> outline;
    outline.reserve(4);
    for (const auto& kp : d.keypoints) {
      outline.emplace_back(cv::Point(cvRound(kp.x), cvRound(kp.y)));
    }

    cv::polylines(image, outline, true, color, 1, cv::LINE_AA);

    if (!outline.empty()) {
      std::string label = d.publish_number;
      if (show_confidence) {
        label += " " + fixed(d.confidence, 2);
      }
      const auto top = *std::min_element(
        outline.begin(), outline.end(),
        [](const cv::Point & lhs, const cv::Point & rhs) {return lhs.y < rhs.y;});
      putTextOutlined(image, label, cv::Point(top.x, std::max(15, top.y - 5)), 0.45, color);
    }
  }
}

void DebugDrawer::drawProfiler(
    cv::Mat& image,
    double fps,
    double latency_ms,
    const std::string& backend_name,
    const std::string& precision) const
{
  int y = 30;
  auto putLine = [&](const std::string& text) {
    cv::putText(image, text, cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    y += 22;
  };

  std::ostringstream ss;
  ss << "Backend: " << backend_name << " | " << precision;
  putLine(ss.str());

  ss.str("");
  ss << "FPS: " << std::fixed << std::setprecision(1) << fps
     << "  Latency: " << std::setprecision(1) << latency_ms << " ms";
  putLine(ss.str());
}

void DebugDrawer::drawArmorsCount(cv::Mat& image, int count) const {
  std::ostringstream ss;
  ss << "Armors: " << count;
  cv::putText(image, ss.str(), cv::Point(10, 74),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
}

void DebugDrawer::drawCrosshair(cv::Mat& image) const {
  if (image.empty()) {
    return;
  }

  const cv::Point center(image.cols / 2, image.rows / 2);
  const cv::Scalar color(0, 255, 255);
  cv::circle(image, center, 4, color, 1, cv::LINE_AA);
}

void DebugDrawer::drawControlOverlay(
  cv::Mat & image,
  const std::vector<ArmorDetection> & detections,
  const DebugOverlayStatus & status,
  const DebugOverlayGeometry & geometry) const
{
  if (image.empty()) {
    return;
  }

  // Current vehicle estimate: connect adjacent armor centers, then draw each plate.
  std::vector<cv::Point> current_centers;
  current_centers.reserve(geometry.current_armor_outlines.size());
  for (const auto & outline : geometry.current_armor_outlines) {
    cv::Point2f center;
    for (const auto & corner : outline) {
      center += corner;
    }
    current_centers.emplace_back(cvRound(center.x * 0.25F), cvRound(center.y * 0.25F));
  }
  if (current_centers.size() >= 2) {
    cv::polylines(
      image, current_centers, current_centers.size() > 2,
      kCurrentArmorLinkColor, 1, cv::LINE_AA);
  }
  for (const auto & outline : geometry.current_armor_outlines) {
    std::vector<cv::Point> pixels;
    pixels.reserve(outline.size());
    for (const auto & corner : outline) {
      pixels.emplace_back(cvRound(corner.x), cvRound(corner.y));
    }
    cv::polylines(image, pixels, true, kCurrentArmorColor, 1, cv::LINE_AA);
  }

  if (geometry.has_center) {
    cv::circle(image, geometry.center, 6, kCenterColor, 2, cv::LINE_AA);
    putTextOutlined(
      image, "CENTER", geometry.center + cv::Point2f(8.0F, -8.0F), 0.4, kCenterColor);
  }
  if (geometry.has_predicted_center) {
    cv::circle(image, geometry.predicted_center, 8, kPredictionColor, 1, cv::LINE_AA);
    if (geometry.has_center) {
      cv::line(image, geometry.center, geometry.predicted_center, kPredictionColor, 1, cv::LINE_AA);
    }
  }
  if (geometry.has_center && geometry.has_velocity_tip) {
    cv::arrowedLine(
      image, geometry.center, geometry.velocity_tip, kCenterColor, 2, cv::LINE_AA, 0, 0.25);
  }

  if (geometry.locked_detection_index >= 0 &&
    geometry.locked_detection_index < static_cast<int>(detections.size()))
  {
    const auto & detection = detections[geometry.locked_detection_index];
    std::vector<cv::Point> outline;
    for (const auto & keypoint : detection.keypoints) {
      outline.emplace_back(cvRound(keypoint.x), cvRound(keypoint.y));
    }
    cv::polylines(image, outline, true, kLockColor, 4, cv::LINE_AA);
    if (!outline.empty()) {
      const auto top = *std::min_element(
        outline.begin(), outline.end(),
        [](const cv::Point & lhs, const cv::Point & rhs) {return lhs.y < rhs.y;});
      putTextOutlined(image, "LOCK", cv::Point(top.x, std::max(20, top.y - 20)), 0.55, kLockColor, 2);
    }
  }

  if (geometry.has_current_selected_armor) {
    cv::circle(image, geometry.current_selected_armor, 5, kLockColor, 2, cv::LINE_AA);
  }
  if (geometry.has_control_target) {
    const cv::Scalar aim_color = status.fire_advice ?
      cv::Scalar(0, 255, 0) : cv::Scalar(0, 80, 255);
    drawDiamond(image, geometry.control_target, 11, aim_color, 2);
    putTextOutlined(
      image, status.tracks_center ? "AIM CENTER" : "AIM",
      geometry.control_target + cv::Point2f(12.0F, 23.0F), 0.48, aim_color, 2);
    if (geometry.has_current_selected_armor) {
      cv::line(
        image, geometry.current_selected_armor, geometry.control_target,
        aim_color, 1, cv::LINE_AA);
    }
  }
  if (geometry.has_fire_target) {
    cv::circle(image, geometry.fire_target, 9, kFireTargetColor, 2, cv::LINE_AA);
    putTextOutlined(
      image, "TARGET", geometry.fire_target + cv::Point2f(11.0F, 16.0F),
      0.42, kFireTargetColor);
  }
  // Compact translucent status panel in the requested top-left location.
  const int panel_width = std::min(430, image.cols);
  const int panel_height = std::min(236, image.rows);
  cv::Mat panel = image(cv::Rect(0, 0, panel_width, panel_height));
  cv::Mat tint(panel.size(), panel.type(), cv::Scalar(12, 15, 18));
  cv::addWeighted(tint, 0.72, panel, 0.28, 0.0, panel);

  const cv::Scalar fire_color = status.fire_advice ?
    cv::Scalar(60, 255, 80) : cv::Scalar(60, 110, 255);
  putTextOutlined(
    image, status.fire_advice ? "FIRE: YES" :
      ("FIRE: HOLD" + (status.fire_hold_reason.empty() ? std::string() :
      " / " + status.fire_hold_reason)),
    cv::Point(12, 25), 0.66, fire_color, 2);

  const std::string target = status.target_id.empty() ? "NONE" : status.target_id;
  const std::string target_mode = status.tracks_center ? "CENTER" :
    (status.virtual_target ? "VIRTUAL" : "ARMOR");
  const std::vector<std::string> lines = {
    "Target " + target + "  " + trackStateName(status.track_state) +
      "  " + modeName(status.mode) + "  " + target_mode,
    "Gimbal  Y " + fixed(status.gimbal_yaw_rad, 3) + "  P " +
      fixed(status.gimbal_pitch_rad, 3) + " rad",
    "Command Y " + fixed(status.command_yaw_rad, 3) + "  P " +
      fixed(status.command_pitch_rad, 3) + " rad",
    "Delta   Y " + fixed(status.yaw_diff_rad, 3) + "  P " +
      fixed(status.pitch_diff_rad, 3) + " rad",
    "Distance " + fixed(status.distance_m, 2) + " m  Speed " +
      fixed(status.target_speed_mps, 2) + " m/s  Turn " +
      fixed(status.target_yaw_rate_radps, 2) + " rad/s",
    "Body  R " + (status.body_attitude_valid
        ? fixed(status.body_roll_rad, 3) : std::string("-.---")) +
      " rad  P " + (status.body_attitude_valid
        ? fixed(status.body_pitch_rad, 3) : std::string("-.---")) + " rad",
    "R1 " + fixed(status.r1_m, 3) + " m  R2 " + fixed(status.r2_m, 3) + " m",
    "Predict " + fixed(status.prediction_ms, 1) + " ms  Flight " +
      fixed(status.flight_ms, 1) + " ms  Age " + fixed(status.data_age_ms, 1) + " ms",
    "Fire err Y " + fixed(status.fire_yaw_error_rad, 3) + "  P " +
      fixed(status.fire_pitch_error_rad, 3) + " rad  P(hit) " +
      (status.probability_enabled ? fixed(100.0 * status.hit_probability, 1) + "%" : "OFF"),
    "Det " + std::to_string(status.detections) + "  Track " +
      std::to_string(status.tracked_targets) + "  Candidates " +
      std::to_string(status.fire_candidates) + "  " + status.strategy,
    "FPS " + fixed(status.fps, 1) + "  Loop " + fixed(status.loop_latency_ms, 1) +
      " ms  Infer " + fixed(status.inference_ms, 1) + " ms  Pose " +
      fixed(status.pose_ms, 1) + " ms"
  };
  int y = 49;
  for (const auto & line : lines) {
    putTextOutlined(image, line, cv::Point(12, y), 0.43, cv::Scalar(235, 235, 235));
    y += 20;
  }

  if (status.stale) {
    putTextOutlined(image, "STALE TRACK DATA", cv::Point(250, 25), 0.48, cv::Scalar(0, 0, 255), 2);
  } else if (status.temp_lost) {
    putTextOutlined(image, "PREDICT ONLY", cv::Point(275, 25), 0.48, cv::Scalar(0, 190, 255), 2);
  }
}

void DebugDrawer::setClassColor(const std::string& label, const cv::Scalar& color) {
  class_colors_[label] = color;
}

cv::Scalar DebugDrawer::generateColor(const std::string& label) {
  // Deterministic color from label hash
  std::hash<std::string> hasher;
  size_t h = hasher(label);
  return cv::Scalar(
    static_cast<int>((h >> 0) & 0xFF),
    static_cast<int>((h >> 8) & 0xFF),
    static_cast<int>((h >> 16) & 0xFF));
}

}  // namespace fyt::auto_aim
