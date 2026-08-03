// bringup_sim: ros-free entry point for the Webots and Gestalt sim chains.
//
// Full chain (Phase C): read frame -> NN detect -> pose estimate (camera frame)
// -> build Armors -> Pipeline.updateTracking (transforms to world, tracks,
// selects) -> Pipeline.computeCommand (MPC/predicted controller) -> convert
// degrees->radians -> write gimbal_command.bin.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "hfut_auto_aim/camera_frame.hpp"
#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/camera/webots_bridge_camera.hpp"
#include "io/gestalt/gestalt_bridge_client.hpp"
#include "io/gestalt/gestalt_idle_scanner.hpp"
#include "io/gestalt/gestalt_latest_frame_receiver.hpp"
#include "io/gestalt/gestalt_protocol.hpp"
#include "io/gimbal/webots_bridge_gimbal.hpp"
#include "io/plotter/plotter.hpp"

#include "config_loader.hpp"
#include "armor_detector_nn/core/armor_detector_nn.hpp"
#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"
#include "armor_detector_nn/core/pose_refine/pose_refiner.hpp"
#include "armor_detector_nn/debug/debug_drawer.hpp"
#include "armor_detector_traditional/armor_detection_adapter.hpp"
#include "armor_detector_traditional/light_bar_detector.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "pipeline.hpp"
#include "rm_utils/logger/log.hpp"

#include <rm_interfaces/msg/armors.hpp>

namespace {
std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

std::string escapeJsonString(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += ch; break;
    }
  }
  return escaped;
}

enum class BridgePath { webots, gestalt };

BridgePath parseBridgePath(const std::string& value) {
  if (value == "webots" || value == "file") return BridgePath::webots;
  if (value == "gestalt") return BridgePath::gestalt;
  throw std::invalid_argument(
      "bridge.path must be webots or gestalt, got: " + value);
}

const char* bridgePathName(BridgePath path) {
  return path == BridgePath::gestalt ? "gestalt" : "webots";
}

fyt::EnemyColor parseEnemyColor(const std::string& value) {
  if (value == "red") return fyt::EnemyColor::RED;
  if (value == "blue") return fyt::EnemyColor::BLUE;
  if (value == "white") return fyt::EnemyColor::WHITE;
  throw std::invalid_argument(
      "bridge.gestalt.enemy_color must be red, blue, or white, got: " + value);
}

// Resolve `package://<pkg>/<rel>` URLs through the ament_index_cpp compat shim
// (the detector asset dir), same convention as the detector backends.
std::string resolvePackageUrl(const std::string& raw_path) {
  if (raw_path.empty()) return raw_path;
  if (raw_path.compare(0, 10, "package://") == 0) {
    auto slash = raw_path.find('/', 10);
    std::string pkg = raw_path.substr(10, slash - 10);
    std::string rel = raw_path.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
  }
  return raw_path;
}

void scaleDetectionKeypointsForPose(
    std::vector<fyt::auto_aim::ArmorDetection>& detections,
    double scale) {
  if (std::abs(scale - 1.0) <= 1e-12) return;
  for (auto& detection : detections) {
    cv::Point2f center(0.0F, 0.0F);
    for (const auto& point : detection.keypoints) center += point;
    center *= 0.25F;
    for (auto& point : detection.keypoints) {
      point = center + (point - center) * static_cast<float>(scale);
    }
    detection.center = center;
  }
}

hfut::io::BridgeInputMode parseInputMode(const std::string& value) {
  if (value == "vision" || value == "visual") return hfut::io::BridgeInputMode::vision;
  if (value == "armor_pose" || value == "direct") {
    return hfut::io::BridgeInputMode::armor_pose;
  }
  throw std::invalid_argument(
      "bridge.input_mode must be vision/visual or armor_pose, got: " + value);
}

Eigen::Vector3d readVector3(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(name + " must be a three-element sequence");
  }
  Eigen::Vector3d value(
      node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!value.allFinite()) {
    throw std::invalid_argument(name + " values must be finite");
  }
  return value;
}

// Build an Armors message (camera frame) from detections + poses, mirroring
// armor_detector_nn_node's publish path with publish_in_target_frame=false.
rm_interfaces::msg::Armors buildArmors(
    const std::vector<fyt::auto_aim::ArmorDetection>& dets,
    const std::vector<fyt::auto_aim::PoseEstimate>& poses) {
  rm_interfaces::msg::Armors msg;
  for (size_t i = 0; i < dets.size(); ++i) {
    rm_interfaces::msg::Armor a;
    a.number = dets[i].publish_number;
    a.type = dets[i].publish_type;
    a.detection_confidence = dets[i].confidence;
    a.track_id = dets[i].track_id;
    a.has_image_geometry = true;
    a.bbox_xywh[0] = dets[i].bbox.x;
    a.bbox_xywh[1] = dets[i].bbox.y;
    a.bbox_xywh[2] = dets[i].bbox.width;
    a.bbox_xywh[3] = dets[i].bbox.height;
    for (int k = 0; k < 4; ++k) {
      a.image_corners[k].x = dets[i].keypoints[k].x;
      a.image_corners[k].y = dets[i].keypoints[k].y;
      a.image_corners[k].z = 0.0f;
    }
    a.corners_ordering = 0;
    if (i < poses.size() && poses[i].valid) {
      const auto& p = poses[i];
      a.pose.position.x = p.translation.x();
      a.pose.position.y = p.translation.y();
      a.pose.position.z = p.translation.z();
      a.pose.orientation.x = p.rotation.x();
      a.pose.orientation.y = p.rotation.y();
      a.pose.orientation.z = p.rotation.z();
      a.pose.orientation.w = p.rotation.w();
      a.pose_estimate_mode = static_cast<uint8_t>(p.mode);
      a.pose_quality_score = static_cast<float>(p.quality_score);
      a.reproj_error_raw = static_cast<float>(p.reproj_error_raw);
      a.reproj_error_refined = static_cast<float>(p.reproj_error_refined);
      a.pose_condition_number = static_cast<float>(p.condition_number);
      a.pose_num_points = static_cast<uint16_t>(p.num_points);
      a.pose_num_inliers = static_cast<uint16_t>(p.num_inliers);
      a.pose_covariance_valid = p.covariance_valid;
      a.ippe_yaw_ambiguity = static_cast<float>(p.ippe_yaw_ambiguity);
      if (p.covariance_valid) {
        for (int r = 0; r < 4; ++r)
          for (int c = 0; c < 4; ++c)
            a.pose_covariance_xyz_yaw[r * 4 + c] = p.covariance_xyz_yaw(r, c);
      }
    }
    msg.armors.push_back(std::move(a));
  }
  return msg;
}

rm_interfaces::msg::Armors buildDirectArmors(const hfut::CameraFrame& frame) {
  rm_interfaces::msg::Armors msg;
  msg.armors.reserve(frame.direct_armors.size());

  for (const auto& measurement : frame.direct_armors) {
    const double position_variance =
        measurement.position_noise_std_m * measurement.position_noise_std_m;
    const double yaw_variance =
        measurement.yaw_noise_std_rad * measurement.yaw_noise_std_rad;
    rm_interfaces::msg::Armor armor;
    armor.number = measurement.number;
    armor.type = measurement.type;
    armor.detection_confidence = static_cast<float>(measurement.confidence);
    armor.pose.position.x = measurement.position_control.x();
    armor.pose.position.y = measurement.position_control.y();
    armor.pose.position.z = measurement.position_control.z();
    armor.radial_yaw_valid = true;
    armor.radial_yaw = measurement.radial_yaw;

    if (measurement.surface_orientation_valid) {
      const auto q = measurement.surface_orientation_control.normalized();
      armor.pose.orientation.x = q.x();
      armor.pose.orientation.y = q.y();
      armor.pose.orientation.z = q.z();
      armor.pose.orientation.w = q.w();
      armor.pose_orientation_trusted = true;
    } else {
      // updateTracking expects detector orientation whose X axis is the plate
      // normal; the tracker converts normal yaw to radial yaw by adding pi.
      const double normal_yaw = measurement.radial_yaw - M_PI;
      armor.pose.orientation.z = std::sin(normal_yaw * 0.5);
      armor.pose.orientation.w = std::cos(normal_yaw * 0.5);
    }
    armor.pose_estimate_mode = 0;  // Direct simulator measurement, not a PnP solution.
    armor.pose_quality_score = static_cast<float>(measurement.confidence);
    armor.pose_num_points = 1;
    armor.pose_num_inliers = 1;
    armor.pose_covariance_valid = position_variance > 0.0 || yaw_variance > 0.0;
    if (armor.pose_covariance_valid) {
      armor.pose_covariance_xyz_yaw.fill(0.0);
      armor.pose_covariance_xyz_yaw[0] = position_variance;
      armor.pose_covariance_xyz_yaw[5] = position_variance;
      armor.pose_covariance_xyz_yaw[10] = position_variance;
      armor.pose_covariance_xyz_yaw[15] = yaw_variance;
    }
    msg.armors.push_back(std::move(armor));
  }
  return msg;
}

bool projectControlPoint(
    const hfut::CameraFrame& frame,
    const Eigen::Vector3d& point_control,
    cv::Point2f& pixel) {
  if (!point_control.allFinite()) return false;
  const Eigen::Vector3d point_camera =
      frame.R_cam2world().transpose() * (point_control - frame.t_cam2world);
  if (!point_camera.allFinite() || point_camera.z() <= 0.02) return false;
  pixel.x = static_cast<float>(
      frame.intrinsics.fx * point_camera.x() / point_camera.z() + frame.intrinsics.cx);
  pixel.y = static_cast<float>(
      frame.intrinsics.fy * point_camera.y() / point_camera.z() + frame.intrinsics.cy);
  return std::isfinite(pixel.x) && std::isfinite(pixel.y);
}

bool pointInImage(const cv::Point2f& point, const cv::Mat& image, float margin = 24.0F) {
  return point.x >= -margin && point.y >= -margin &&
         point.x < image.cols + margin && point.y < image.rows + margin;
}

bool projectArmorOutline(
    const hfut::CameraFrame& frame,
    const Eigen::Vector3d& armor_center,
    const Eigen::Vector3d& width_axis_world,
    const Eigen::Vector3d& height_axis_world,
    double armor_width,
    double armor_height,
    const cv::Mat& image,
    std::array<cv::Point2f, 4>& outline) {
  if (!armor_center.allFinite() || !width_axis_world.allFinite() ||
      !height_axis_world.allFinite() || width_axis_world.norm() <= 1e-6 ||
      height_axis_world.norm() <= 1e-6 || armor_width <= 0.0 ||
      armor_height <= 0.0) {
    return false;
  }

  const Eigen::Vector3d right = width_axis_world.normalized();
  Eigen::Vector3d up = height_axis_world -
      right * height_axis_world.dot(right);
  if (up.norm() <= 1e-6) return false;
  up.normalize();
  const double half_width = armor_width * 0.5;
  const double half_height = armor_height * 0.5;
  const std::array<Eigen::Vector3d, 4> world_corners = {
      armor_center - right * half_width + up * half_height,
      armor_center + right * half_width + up * half_height,
      armor_center + right * half_width - up * half_height,
      armor_center - right * half_width - up * half_height};

  cv::Point2f projected_center;
  if (!projectControlPoint(frame, armor_center, projected_center) ||
      !pointInImage(projected_center, image, 120.0F)) {
    return false;
  }
  for (size_t i = 0; i < world_corners.size(); ++i) {
    if (!projectControlPoint(frame, world_corners[i], outline[i])) {
      return false;
    }
  }
  return true;
}

struct InputArmorOverlay {
  std::array<cv::Point2f, 4> outline;
  cv::Point2f center;
  cv::Point2f normal_tip;
  bool has_normal_tip{false};
  std::string label;
};

void drawInputArmorOverlays(
    cv::Mat& image,
    const std::vector<InputArmorOverlay>& overlays,
    const cv::Scalar& color) {
  for (const auto& overlay : overlays) {
    std::vector<cv::Point> pixels;
    pixels.reserve(overlay.outline.size());
    for (const auto& corner : overlay.outline) {
      pixels.emplace_back(cvRound(corner.x), cvRound(corner.y));
    }
    cv::polylines(image, pixels, true, color, 2, cv::LINE_AA);
    cv::circle(image, overlay.center, 3, color, cv::FILLED, cv::LINE_AA);
    if (overlay.has_normal_tip) {
      cv::arrowedLine(
          image, overlay.center, overlay.normal_tip, color,
          1, cv::LINE_AA, 0, 0.25);
    }

    const auto top = *std::min_element(
        pixels.begin(), pixels.end(),
        [](const cv::Point& lhs, const cv::Point& rhs) {
          return lhs.y < rhs.y;
        });
    const cv::Point label_origin(top.x, std::max(15, top.y - 6));
    cv::putText(
        image, overlay.label, label_origin, cv::FONT_HERSHEY_SIMPLEX,
        0.42, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(
        image, overlay.label, label_origin, cv::FONT_HERSHEY_SIMPLEX,
        0.42, color, 1, cv::LINE_AA);
  }
}

cv::Point2f detectionCenter(const fyt::auto_aim::ArmorDetection& detection) {
  cv::Point2f center(0.0F, 0.0F);
  for (const auto& point : detection.keypoints) center += point;
  return center * 0.25F;
}

int findLockedDetection(
    const std::vector<fyt::auto_aim::ArmorDetection>& detections,
    const std::string& selected_id,
    const cv::Point2f& projected_armor) {
  int best_index = -1;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < detections.size(); ++i) {
    if (!selected_id.empty() && detections[i].publish_number != selected_id) continue;
    const double distance = cv::norm(detectionCenter(detections[i]) - projected_armor);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}
}  // namespace

int main(int argc, char** argv) {
  using namespace fyt::auto_aim;
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  std::signal(SIGPIPE, SIG_IGN);

  // Positional args = config paths (flags like --debug are skipped); --debug
  // enables the imshow overlay + UDP scalar stream to PlotJuggler.
  bool debug = false;
  std::string input_mode_override;
  std::string bridge_path_override;
  std::string detector_impl_override;
  std::string controller_strategy_override;
  std::string config_dir = "configs";
  std::string diagnostics_path;
  int gestalt_read_fd = -1;
  int gestalt_write_fd = -1;
  const char* snapshot_env = std::getenv("HFUT_DEBUG_SNAPSHOT");
  const std::string debug_snapshot_path = snapshot_env ? snapshot_env : "";
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--debug") {
      debug = true;
    } else if (a == "--armor-pose") {
      input_mode_override = "armor_pose";
    } else if (a == "--gestalt") {
      bridge_path_override = "gestalt";
      input_mode_override = "vision";
    } else if (a.rfind("--input-mode=", 0) == 0) {
      input_mode_override = a.substr(std::string("--input-mode=").size());
      if (input_mode_override == "gestalt") {
        bridge_path_override = "gestalt";
        input_mode_override = "vision";
      }
    } else if (a.rfind("--detector=", 0) == 0) {
      detector_impl_override = a.substr(std::string("--detector=").size());
    } else if (a.rfind("--strategy=", 0) == 0) {
      controller_strategy_override = a.substr(std::string("--strategy=").size());
    } else if (a.rfind("--bridge-path=", 0) == 0) {
      bridge_path_override = a.substr(std::string("--bridge-path=").size());
    } else if (a.rfind("--config-dir=", 0) == 0) {
      config_dir = a.substr(std::string("--config-dir=").size());
    } else if (a.rfind("--gestalt-read-fd=", 0) == 0) {
      gestalt_read_fd = std::stoi(a.substr(std::string("--gestalt-read-fd=").size()));
    } else if (a.rfind("--gestalt-write-fd=", 0) == 0) {
      gestalt_write_fd = std::stoi(a.substr(std::string("--gestalt-write-fd=").size()));
    } else if (a == "--diagnostics") {
      const char* bridge_dir = std::getenv("WEBOTS_ROS_FREE_BRIDGE_DIR");
      diagnostics_path = std::string(bridge_dir && bridge_dir[0] ? bridge_dir
                                                                 : "/tmp/hfut_auto_aim_webots") +
                         "/tracking_diagnostics.jsonl";
    } else if (a.rfind("--diagnostics=", 0) == 0) {
      diagnostics_path = a.substr(std::string("--diagnostics=").size());
    }
  }
  // Topic-split config set: master switches in gimbal_pipeline.yaml, details
  // distributed over simulation/detector/tracker/controller.yaml.
  const std::string master_cfg = config_dir + "/gimbal_pipeline.yaml";
  const std::string simulation_cfg = config_dir + "/simulation.yaml";
  const std::string tracker_cfg = config_dir + "/tracker.yaml";
  const std::string controller_cfg = config_dir + "/controller.yaml";
  const std::string detector_cfg = config_dir + "/detector.yaml";

  hfut::io::BridgeInputMode input_mode = hfut::io::BridgeInputMode::vision;
  BridgePath bridge_path = BridgePath::webots;
  std::string bridge_dir;
  std::string gestalt_host;
  int gestalt_port = hfut::gestalt::kDefaultPort;
  int gestalt_player_id = 0;
  fyt::EnemyColor gestalt_enemy_color = fyt::EnemyColor::BLUE;
  double gestalt_sim_quad_scale = 1.192;
  double webots_keypoint_scale = 1.0;
  bool gestalt_forward_auto_aim_velocity = false;
  hfut::CameraToBarrelExtrinsics camera_to_barrel;
  double bullet_speed = 22.5;
  // Detector chain selection: "nn" (default), "traditional", or "auto"
  // (NN with light-bar fallback when the NN backend fails to initialize).
  std::string detector_impl = "nn";
  fyt::EnemyColor traditional_enemy_color = fyt::EnemyColor::RED;
  bool traditional_enemy_color_set = false;
  fyt::auto_aim::LightBarDetector::LightParams traditional_light_params{
      0.08, 0.4, 40.0, 25};
  fyt::auto_aim::LightBarDetector::ArmorParams traditional_armor_params{
      0.6, 0.8, 3.2, 3.2, 5.0, 35.0};
  int traditional_binary_thres = 160;
  double traditional_classifier_threshold = 0.7;
  std::vector<std::string> traditional_ignore_classes{"negative"};
  bool traditional_use_pca = true;
  std::string traditional_model_path =
      "package://armor_detector_nn/model/lenet.onnx";
  std::string traditional_label_path =
      "package://armor_detector_nn/model/lenet_labels.txt";
  double observation_noise_scale = 0.35;
  double max_temp_lost_prediction_s = 0.15;
  double temp_lost_coast_max_s = 0.5;
  double temp_lost_coast_min_speed_mps = 0.3;
  double id_association_max_distance_m = 0.6;
  double attitude_mount_pitch_deg = 15.0;
  double attitude_ema_alpha = 0.1;
  bool attitude_apply_to_geometry = false;
  fyt::auto_aim::TrackerMotionGuardParameters motion_guard;
  hfut::io::GestaltIdleScanConfig gestalt_scan_config;
  std::unique_ptr<hfut::io::GestaltIdleScanner> gestalt_idle_scanner;
  try {
    const YAML::Node master_root = YAML::LoadFile(master_cfg);
    const YAML::Node sim_root = YAML::LoadFile(simulation_cfg);
    const YAML::Node tracker_root = YAML::LoadFile(tracker_cfg);
    const YAML::Node detector_root = YAML::LoadFile(detector_cfg);

    // 全局模式开关（总控）
    const YAML::Node global = master_root["global"];
    const std::string configured_path =
        global && global["bridge_path"]
            ? global["bridge_path"].as<std::string>()
            : "webots";
    bridge_path = parseBridgePath(
        bridge_path_override.empty() ? configured_path : bridge_path_override);
    const std::string path_name = bridgePathName(bridge_path);
    if (global && global["detector_impl"]) {
      detector_impl = global["detector_impl"].as<std::string>();
    }
    if (!detector_impl_override.empty()) {
      detector_impl = detector_impl_override;
    }
    if (detector_impl != "nn" && detector_impl != "traditional" &&
        detector_impl != "auto") {
      throw std::invalid_argument(
          "global.detector_impl must be \"nn\", \"traditional\", or \"auto\", got: " +
          detector_impl);
    }
    const std::string configured_mode =
        global && global["input_mode"]
            ? global["input_mode"].as<std::string>()
            : "vision";
    input_mode = bridge_path == BridgePath::gestalt
        ? hfut::io::BridgeInputMode::vision
        : parseInputMode(input_mode_override.empty() ? configured_mode : input_mode_override);

    const YAML::Node tracking = tracker_root["tracking"];
    if (tracking && tracking["observation_noise_scale"]) {
      const YAML::Node& noise_scale = tracking["observation_noise_scale"];
      // Per-input-mode form (vision PnP noise is ~4x the armor_pose truth
      // noise, so a single scale cannot fit both); scalar form still accepted.
      if (noise_scale.IsMap()) {
        const char* mode_key =
            input_mode == hfut::io::BridgeInputMode::vision ? "vision" : "armor_pose";
        if (noise_scale[mode_key]) {
          observation_noise_scale = noise_scale[mode_key].as<double>();
        }
      } else {
        observation_noise_scale = noise_scale.as<double>();
      }
    }
    if (tracking && tracking["max_temp_lost_prediction_s"]) {
      max_temp_lost_prediction_s =
          tracking["max_temp_lost_prediction_s"].as<double>();
    }
    if (tracking && tracking["temp_lost_coast_max_s"]) {
      temp_lost_coast_max_s = tracking["temp_lost_coast_max_s"].as<double>();
    }
    if (tracking && tracking["temp_lost_coast_min_speed_mps"]) {
      temp_lost_coast_min_speed_mps =
          tracking["temp_lost_coast_min_speed_mps"].as<double>();
    }
    if (tracking && tracking["id_association_max_distance_m"]) {
      id_association_max_distance_m =
          tracking["id_association_max_distance_m"].as<double>();
    }
    const YAML::Node attitude = tracking ? tracking["attitude"] : YAML::Node{};
    if (attitude) {
      if (attitude["mount_pitch_deg"]) {
        attitude_mount_pitch_deg = attitude["mount_pitch_deg"].as<double>();
      }
      if (attitude["ema_alpha"]) {
        attitude_ema_alpha = attitude["ema_alpha"].as<double>();
      }
      if (attitude["apply_to_geometry"]) {
        attitude_apply_to_geometry =
            attitude["apply_to_geometry"].as<bool>();
      }
    }
    const YAML::Node motion_guard_node = tracking ? tracking["motion_guard"] : YAML::Node{};
    if (motion_guard_node) {
      if (motion_guard_node["enabled"]) {
        motion_guard.enabled = motion_guard_node["enabled"].as<bool>();
      }
      if (motion_guard_node["stationary_speed_deadband_mps"]) {
        motion_guard.stationary_speed_deadband_mps =
            motion_guard_node["stationary_speed_deadband_mps"].as<double>();
      }
      if (motion_guard_node["max_linear_speed_mps"]) {
        motion_guard.max_linear_speed_mps =
            motion_guard_node["max_linear_speed_mps"].as<double>();
      }
      if (motion_guard_node["max_linear_acceleration_mps2"]) {
        motion_guard.max_linear_acceleration_mps2 =
            motion_guard_node["max_linear_acceleration_mps2"].as<double>();
      }
      if (motion_guard_node["temp_lost_velocity_half_life_s"]) {
        motion_guard.temp_lost_velocity_half_life_s =
            motion_guard_node["temp_lost_velocity_half_life_s"].as<double>();
      }
      if (motion_guard_node["velocity_reset_std_mps"]) {
        motion_guard.velocity_reset_std_mps =
            motion_guard_node["velocity_reset_std_mps"].as<double>();
      }
      if (motion_guard_node["acceleration_reset_std_mps2"]) {
        motion_guard.acceleration_reset_std_mps2 =
            motion_guard_node["acceleration_reset_std_mps2"].as<double>();
      }
      if (motion_guard_node["max_yaw_rate_rad_s"]) {
        motion_guard.max_yaw_rate_rad_s =
            motion_guard_node["max_yaw_rate_rad_s"].as<double>();
      }
      if (motion_guard_node["yaw_rate_deadband_rad_s"]) {
        motion_guard.yaw_rate_deadband_rad_s =
            motion_guard_node["yaw_rate_deadband_rad_s"].as<double>();
      }
      if (motion_guard_node["yaw_rate_reset_std_rad_s"]) {
        motion_guard.yaw_rate_reset_std_rad_s =
            motion_guard_node["yaw_rate_reset_std_rad_s"].as<double>();
      }
    }
    if (!std::isfinite(observation_noise_scale) || observation_noise_scale <= 0.0) {
      throw std::invalid_argument(
          "tracking.observation_noise_scale must be finite and > 0");
    }
    auto require_non_negative = [](double value, const char *name) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
      }
    };
    auto require_positive = [](double value, const char *name) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and > 0");
      }
    };
    require_non_negative(
        max_temp_lost_prediction_s,
        "tracking.max_temp_lost_prediction_s");
    require_non_negative(
        motion_guard.stationary_speed_deadband_mps,
        "tracking.motion_guard.stationary_speed_deadband_mps");
    require_positive(
        motion_guard.max_linear_speed_mps,
        "tracking.motion_guard.max_linear_speed_mps");
    require_positive(
        motion_guard.max_linear_acceleration_mps2,
        "tracking.motion_guard.max_linear_acceleration_mps2");
    require_positive(
        motion_guard.temp_lost_velocity_half_life_s,
        "tracking.motion_guard.temp_lost_velocity_half_life_s");
    require_positive(
        motion_guard.velocity_reset_std_mps,
        "tracking.motion_guard.velocity_reset_std_mps");
    require_positive(
        motion_guard.acceleration_reset_std_mps2,
        "tracking.motion_guard.acceleration_reset_std_mps2");
    require_positive(
        motion_guard.max_yaw_rate_rad_s,
        "tracking.motion_guard.max_yaw_rate_rad_s");
    require_non_negative(
        motion_guard.yaw_rate_deadband_rad_s,
        "tracking.motion_guard.yaw_rate_deadband_rad_s");
    require_positive(
        motion_guard.yaw_rate_reset_std_rad_s,
        "tracking.motion_guard.yaw_rate_reset_std_rad_s");

    const YAML::Node camera_root = sim_root["camera_to_barrel"];
    const YAML::Node camera_extrinsics =
        camera_root && camera_root[path_name] ? camera_root[path_name] : camera_root;
    if (camera_extrinsics && camera_extrinsics["xyz"]) {
      camera_to_barrel.xyz = readVector3(
          camera_extrinsics["xyz"], "camera_to_barrel." + path_name + ".xyz");
    }
    if (camera_extrinsics && camera_extrinsics["rpy"]) {
      camera_to_barrel.rpy = readVector3(
          camera_extrinsics["rpy"], "camera_to_barrel." + path_name + ".rpy");
    }

    const YAML::Node controller = sim_root["controller"];
    const YAML::Node bullet_speed_node = controller ? controller["bullet_speed"] : YAML::Node{};
    if (bullet_speed_node) {
      bullet_speed = bullet_speed_node.IsMap()
          ? bullet_speed_node[path_name].as<double>()
          : bullet_speed_node.as<double>();
    }
    if (!std::isfinite(bullet_speed) || bullet_speed <= 0.0) {
      throw std::invalid_argument("controller.bullet_speed must be finite and > 0");
    }

    const YAML::Node traditional = detector_root["detector_traditional"];
    if (traditional) {
      if (traditional["enemy_color"] &&
          !traditional["enemy_color"].as<std::string>().empty()) {
        traditional_enemy_color =
            parseEnemyColor(traditional["enemy_color"].as<std::string>());
        traditional_enemy_color_set = true;
      }
      if (traditional["binary_thres"]) {
        traditional_binary_thres = traditional["binary_thres"].as<int>();
      }
      const YAML::Node light = traditional["light"];
      if (light) {
        if (light["min_ratio"]) {
          traditional_light_params.min_ratio = light["min_ratio"].as<double>();
        }
        if (light["max_ratio"]) {
          traditional_light_params.max_ratio = light["max_ratio"].as<double>();
        }
        if (light["max_angle"]) {
          traditional_light_params.max_angle = light["max_angle"].as<double>();
        }
        if (light["color_diff_thresh"]) {
          traditional_light_params.color_diff_thresh =
              light["color_diff_thresh"].as<int>();
        }
      }
      const YAML::Node armor = traditional["armor"];
      if (armor) {
        if (armor["min_light_ratio"]) {
          traditional_armor_params.min_light_ratio =
              armor["min_light_ratio"].as<double>();
        }
        if (armor["min_small_center_distance"]) {
          traditional_armor_params.min_small_center_distance =
              armor["min_small_center_distance"].as<double>();
        }
        if (armor["max_small_center_distance"]) {
          traditional_armor_params.max_small_center_distance =
              armor["max_small_center_distance"].as<double>();
        }
        if (armor["min_large_center_distance"]) {
          traditional_armor_params.min_large_center_distance =
              armor["min_large_center_distance"].as<double>();
        }
        if (armor["max_large_center_distance"]) {
          traditional_armor_params.max_large_center_distance =
              armor["max_large_center_distance"].as<double>();
        }
        if (armor["max_angle"]) {
          traditional_armor_params.max_angle = armor["max_angle"].as<double>();
        }
      }
      if (traditional["classifier_threshold"]) {
        traditional_classifier_threshold =
            traditional["classifier_threshold"].as<double>();
      }
      if (traditional["ignore_classes"]) {
        traditional_ignore_classes =
            traditional["ignore_classes"].as<std::vector<std::string>>();
      }
      if (traditional["use_pca"]) {
        traditional_use_pca = traditional["use_pca"].as<bool>();
      }
      if (traditional["model_path"]) {
        traditional_model_path = traditional["model_path"].as<std::string>();
      }
      if (traditional["label_path"]) {
        traditional_label_path = traditional["label_path"].as<std::string>();
      }
    }
    const YAML::Node bridge = sim_root["bridge"];
    bridge_dir = bridge && bridge["dir"] ? bridge["dir"].as<std::string>() : "";
    const YAML::Node webots = bridge ? bridge["webots"] : YAML::Node{};
    if (webots && webots["keypoint_scale"]) {
      webots_keypoint_scale = webots["keypoint_scale"].as<double>();
    }
    if (!std::isfinite(webots_keypoint_scale) || webots_keypoint_scale <= 0.0) {
      throw std::invalid_argument("bridge.webots.keypoint_scale must be finite and > 0");
    }
    const YAML::Node gestalt = bridge ? bridge["gestalt"] : YAML::Node{};
    gestalt_host = gestalt && gestalt["host"] ? gestalt["host"].as<std::string>() : "";
    if (gestalt_host.empty()) {
      const char* host_env = std::getenv("GESTALT_BRIDGE_HOST");
      gestalt_host = host_env && host_env[0] ? host_env : "127.0.0.1";
    }
    if (gestalt && gestalt["port"]) gestalt_port = gestalt["port"].as<int>();
    if (gestalt && gestalt["player_id"]) {
      gestalt_player_id = gestalt["player_id"].as<int>();
    }
    if (gestalt && gestalt["enemy_color"]) {
      gestalt_enemy_color =
          parseEnemyColor(gestalt["enemy_color"].as<std::string>());
    }
    // Default enemy color follows the transport path when not pinned in yaml.
    if (!traditional_enemy_color_set) {
      traditional_enemy_color = bridge_path == BridgePath::gestalt
          ? gestalt_enemy_color
          : fyt::EnemyColor::RED;
    }
    if (gestalt && gestalt["sim_quad_scale"]) {
      gestalt_sim_quad_scale = gestalt["sim_quad_scale"].as<double>();
    }
    if (gestalt && gestalt["forward_auto_aim_velocity"]) {
      gestalt_forward_auto_aim_velocity =
          gestalt["forward_auto_aim_velocity"].as<bool>();
    }
    const YAML::Node idle_scan = gestalt ? gestalt["idle_scan"] : YAML::Node{};
    if (idle_scan && idle_scan["enabled"]) {
      gestalt_scan_config.enabled = idle_scan["enabled"].as<bool>();
    }
    if (idle_scan && idle_scan["yaw_rate_deg_s"]) {
      gestalt_scan_config.yaw_rate_deg_s = idle_scan["yaw_rate_deg_s"].as<double>();
    }
    if (idle_scan && idle_scan["pitch_rate_deg_s"]) {
      gestalt_scan_config.pitch_rate_deg_s = idle_scan["pitch_rate_deg_s"].as<double>();
    }
    if (idle_scan && idle_scan["pitch_limit_deg"]) {
      gestalt_scan_config.pitch_limit_deg = idle_scan["pitch_limit_deg"].as<double>();
    }
    if (idle_scan && idle_scan["activation_delay_s"]) {
      gestalt_scan_config.activation_delay_s =
          idle_scan["activation_delay_s"].as<double>();
    }
    if (idle_scan && idle_scan["max_step_s"]) {
      gestalt_scan_config.max_step_s = idle_scan["max_step_s"].as<double>();
    }
    if (gestalt_port <= 0 || gestalt_port > 65535) {
      throw std::invalid_argument("bridge.gestalt.port must be in [1, 65535]");
    }
    if ((gestalt_read_fd >= 0) != (gestalt_write_fd >= 0)) {
      throw std::invalid_argument(
          "--gestalt-read-fd and --gestalt-write-fd must be provided together");
    }
    if (!std::isfinite(gestalt_sim_quad_scale) || gestalt_sim_quad_scale <= 0.0) {
      throw std::invalid_argument("bridge.gestalt.sim_quad_scale must be finite and > 0");
    }
    if (bridge_path == BridgePath::gestalt) {
      gestalt_idle_scanner =
          std::make_unique<hfut::io::GestaltIdleScanner>(gestalt_scan_config);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[bringup_sim] invalid config in %s: %s\n",
                 config_dir.c_str(), e.what());
    return 1;
  }

  FYT_REGISTER_LOGGER("armor_detector", "/tmp/hfut_auto_aim_log", INFO);
  FYT_REGISTER_LOGGER("armor_detector_nn", "/tmp/hfut_auto_aim_log", INFO);

  // ── Detector ──
  DetectorConfig dcfg;
  std::unique_ptr<ArmorDetectorNN> detector;
  std::unique_ptr<LightBarDetector> traditional_detector;
  std::unique_ptr<ArmorPoseEstimatorAdapter> pose_adapter;
  if (input_mode == hfut::io::BridgeInputMode::vision) {
    const bool wants_nn = detector_impl != "traditional";
    const bool allows_traditional = detector_impl != "nn";
    bool nn_ready = false;
    if (wants_nn) {
      dcfg = hfut::detector::loadDetectorConfigFile(detector_cfg);
      detector = std::make_unique<ArmorDetectorNN>(dcfg);
      if (bridge_path == BridgePath::gestalt) {
        detector->setTargetColor(gestalt_enemy_color);
      }
      nn_ready = detector->initialize();
      if (!nn_ready) {
        if (!allows_traditional) {
          std::printf("[bringup_sim] detector init FAILED\n");
          return 1;
        }
        std::printf(
            "[bringup_sim] NN detector init FAILED, falling back to the "
            "traditional light-bar detector\n");
        detector.reset();
      }
    }
    if (allows_traditional && !nn_ready) {
      traditional_detector = std::make_unique<LightBarDetector>(
          traditional_binary_thres, traditional_enemy_color,
          traditional_light_params, traditional_armor_params);
      traditional_detector->classifier = std::make_unique<NumberClassifier>(
          resolvePackageUrl(traditional_model_path),
          resolvePackageUrl(traditional_label_path),
          traditional_classifier_threshold, traditional_ignore_classes);
      if (traditional_use_pca) {
        traditional_detector->corner_corrector =
            std::make_unique<LightCornerCorrector>();
      }
      // The pose adapter settings live under detector.pose; load them even
      // when the NN chain itself is not constructed.
      if (detector_impl == "traditional") {
        dcfg = hfut::detector::loadDetectorConfigFile(detector_cfg);
      }
    }
    pose_adapter = std::make_unique<ArmorPoseEstimatorAdapter>(dcfg.pose);
    if (dcfg.pose.refiner.mode == "single_yaw") {
      pose_adapter->setRefiner(
          std::make_shared<SingleYawRefiner>(dcfg.pose.single_yaw, dcfg.pose.gate));
    }
  }

  // ── Pipeline (tracker + selector + controller) ──
  hfut::pipeline::PipelineOverrides pipeline_overrides;
  pipeline_overrides.bullet_speed = bullet_speed;
  pipeline_overrides.controller_strategy = controller_strategy_override;
  pipeline_overrides.observation_noise_scale = observation_noise_scale;
  pipeline_overrides.max_temp_lost_prediction_s =
      max_temp_lost_prediction_s;
  pipeline_overrides.temp_lost_coast_max_s = temp_lost_coast_max_s;
  pipeline_overrides.temp_lost_coast_min_speed_mps =
      temp_lost_coast_min_speed_mps;
  pipeline_overrides.attitude_mount_pitch_deg = attitude_mount_pitch_deg;
  pipeline_overrides.attitude_ema_alpha = attitude_ema_alpha;
  pipeline_overrides.attitude_apply_to_geometry = attitude_apply_to_geometry;
  pipeline_overrides.id_association_max_distance_m = id_association_max_distance_m;
  pipeline_overrides.motion_guard = motion_guard;
  // Topic files deep-merge in order; the master file (global switches) wins.
  hfut::pipeline::Pipeline pipeline(
      {tracker_cfg, controller_cfg, master_cfg},
      "gimbal_pipeline", pipeline_overrides);

  // ── IO ──
  std::unique_ptr<hfut::io::WebotsBridgeCamera> webots_camera;
  std::unique_ptr<hfut::io::WebotsBridgeGimbal> webots_gimbal;
  std::unique_ptr<hfut::io::GestaltBridgeClient> gestalt_bridge;
  std::unique_ptr<hfut::io::GestaltLatestFrameReceiver> gestalt_frame_receiver;
  if (bridge_path == BridgePath::gestalt) {
    if (gestalt_read_fd >= 0) {
      gestalt_bridge = std::make_unique<hfut::io::GestaltBridgeClient>(
          gestalt_read_fd, gestalt_write_fd, gestalt_player_id);
    } else {
      gestalt_bridge = std::make_unique<hfut::io::GestaltBridgeClient>(
          gestalt_host, static_cast<uint16_t>(gestalt_port), gestalt_player_id);
    }
    gestalt_frame_receiver =
        std::make_unique<hfut::io::GestaltLatestFrameReceiver>(*gestalt_bridge);
  } else {
    webots_camera = std::make_unique<hfut::io::WebotsBridgeCamera>(bridge_dir, input_mode);
    webots_gimbal = std::make_unique<hfut::io::WebotsBridgeGimbal>(bridge_dir);
  }
  std::ofstream diagnostics;
  if (!diagnostics_path.empty()) {
    diagnostics.open(diagnostics_path, std::ios::out | std::ios::trunc);
    if (!diagnostics) {
      std::fprintf(stderr, "[bringup_sim] failed to open diagnostics: %s\n",
                   diagnostics_path.c_str());
      return 1;
    }
    std::printf("[bringup_sim] diagnostics: %s\n", diagnostics_path.c_str());
  }

  // ── Debug visualization (imshow overlay + PlotJuggler UDP scalars) ──
  std::unique_ptr<DebugDrawer> drawer;
  std::unique_ptr<hfut::io::Plotter> plotter;
  double debug_window_scale = 1.0;
  if (const char *scale_env = std::getenv("HFUT_DEBUG_WINDOW_SCALE")) {
    const double parsed = std::atof(scale_env);
    if (std::isfinite(parsed) && parsed > 0.1 && parsed <= 4.0) {
      debug_window_scale = parsed;
    }
  }
  if (debug) {
    drawer = std::make_unique<DebugDrawer>();
    plotter = std::make_unique<hfut::io::Plotter>();  // 127.0.0.1:9870
    cv::namedWindow("hfut_auto_aim", cv::WINDOW_NORMAL);
    cv::resizeWindow("hfut_auto_aim", 960, 720);
    std::printf("[bringup_sim] debug ON: imshow window + PlotJuggler UDP 127.0.0.1:9870\n");
    if (debug_window_scale != 1.0) {
      std::printf("[bringup_sim] debug window scale: %.2f\n", debug_window_scale);
    }
  }

  std::printf("[bringup_sim] bridge path: %s\n",
              bridge_path == BridgePath::gestalt ? "gestalt" : "webots");
  if (gestalt_bridge) {
    if (gestalt_bridge->usesStdioTransport()) {
      std::printf("[bringup_sim] gestalt transport: WSL interop stdio player=%d\n",
                  gestalt_player_id);
    } else {
      std::printf("[bringup_sim] gestalt endpoint: %s:%u player=%d\n",
                  gestalt_bridge->host().c_str(),
                  static_cast<unsigned>(gestalt_bridge->port()), gestalt_player_id);
    }
    std::printf(
        "[bringup_sim] gestalt idle scan: %s yaw=%.1f deg/s pitch=%.1f deg/s "
        "range=[-%.1f,+%.1f] deg delay=%.1fs\n",
        gestalt_scan_config.enabled ? "enabled" : "disabled",
        gestalt_scan_config.yaw_rate_deg_s, gestalt_scan_config.pitch_rate_deg_s,
        gestalt_scan_config.pitch_limit_deg, gestalt_scan_config.pitch_limit_deg,
        gestalt_scan_config.activation_delay_s);
    std::printf(
        "[bringup_sim] gestalt vision: enemy=%s sim_quad_scale=%.3f "
        "auto_aim_velocity=%s\n",
        fyt::enemyColorToString(gestalt_enemy_color).c_str(), gestalt_sim_quad_scale,
        gestalt_forward_auto_aim_velocity ? "forwarded" : "angle-only");
  }
  std::printf("[bringup_sim] input mode: %s\n",
              input_mode == hfut::io::BridgeInputMode::armor_pose ? "armor_pose" : "vision");
  if (input_mode == hfut::io::BridgeInputMode::vision) {
    std::printf(
        "[bringup_sim] detector: impl=%s active=%s enemy=%s\n",
        detector_impl.c_str(),
        traditional_detector ? "traditional" : "nn",
        traditional_detector
            ? fyt::enemyColorToString(traditional_enemy_color).c_str()
            : (bridge_path == BridgePath::gestalt
                   ? fyt::enemyColorToString(gestalt_enemy_color).c_str()
                   : "model"));
  }
  std::printf(
      "[bringup_sim] camera_to_barrel: xyz=[%.4f,%.4f,%.4f]m "
      "rpy=[%.4f,%.4f,%.4f]rad\n",
      camera_to_barrel.xyz.x(), camera_to_barrel.xyz.y(), camera_to_barrel.xyz.z(),
      camera_to_barrel.rpy.x(), camera_to_barrel.rpy.y(), camera_to_barrel.rpy.z());
  std::printf(
      "[bringup_sim] ballistics/tracking: bullet_speed=%.2fm/s "
      "observation_noise_scale=%.3f temp_lost_output=%.3fs "
      "motion_guard=%s\n",
      bullet_speed, observation_noise_scale, max_temp_lost_prediction_s,
      motion_guard.enabled ? "enabled" : "disabled");
  std::printf("[bringup_sim] full chain ready, waiting for input...\n");
  std::fflush(stdout);

  hfut::CameraFrame frame;
  uint64_t frames = 0;
  auto last_log = std::chrono::steady_clock::now();
  auto last_frame_done = std::chrono::steady_clock::now();
  double smoothed_fps = 0.0;
  double smoothed_source_fps = 0.0;
  double previous_source_time_s = 0.0;
  bool idle_scan_was_moving = false;

  while (!g_stop.load()) {
    const bool frame_received = gestalt_frame_receiver
        ? gestalt_frame_receiver->readLatest(frame, std::chrono::milliseconds(2000))
        : webots_camera->read(frame, std::chrono::milliseconds(2000));
    if (!frame_received) {
      std::printf("[bringup_sim] no %s frame for 2s\n",
                  input_mode == hfut::io::BridgeInputMode::armor_pose ? "armor-pose" : "camera");
      std::fflush(stdout);
      continue;
    }
    if (!hfut::applyCameraToBarrelExtrinsics(frame, camera_to_barrel)) {
      std::fprintf(stderr, "[bringup_sim] invalid camera pose in frame %llu\n",
                   static_cast<unsigned long long>(frame.seq));
      continue;
    }
    ++frames;
    if (previous_source_time_s > 0.0) {
      const double source_interval = frame.sim_time_s - previous_source_time_s;
      if (source_interval > 1e-6 && source_interval < 1.0) {
        const double source_fps = 1.0 / source_interval;
        smoothed_source_fps = smoothed_source_fps <= 0.0 ? source_fps :
            0.9 * smoothed_source_fps + 0.1 * source_fps;
      }
    }
    previous_source_time_s = frame.sim_time_s;
    const auto processing_started = std::chrono::steady_clock::now();

    rm_interfaces::msg::Armors armors;
    std::vector<ArmorDetection> dets;
    std::vector<ArmorDetection> pose_detections;
    std::vector<PoseEstimate> poses;
    double pose_latency_ms = 0.0;
    if (input_mode == hfut::io::BridgeInputMode::armor_pose) {
      armors = buildDirectArmors(frame);
      pipeline.updateTrackingControlFrame(armors, frame.sim_time_s);
    } else {
      sensor_msgs::msg::CameraInfo cam_info;
      cam_info.width = frame.intrinsics.width;
      cam_info.height = frame.intrinsics.height;
      cam_info.k = {frame.intrinsics.fx, 0, frame.intrinsics.cx,
                    0, frame.intrinsics.fy, frame.intrinsics.cy, 0, 0, 1};
      cam_info.d.assign(frame.intrinsics.distortion, frame.intrinsics.distortion + 5);
      if (frame.intrinsics.fx > 1e-6 && frame.intrinsics.fy > 1e-6 &&
          frame.intrinsics.width > 0 && frame.intrinsics.height > 0) {
        pipeline.updateFov(
            std::atan(static_cast<double>(frame.intrinsics.width) /
                      (2.0 * frame.intrinsics.fx)),
            std::atan(static_cast<double>(frame.intrinsics.height) /
                      (2.0 * frame.intrinsics.fy)));
      }

      std_msgs::msg::Header header;
      const int64_t frame_time_ns = static_cast<int64_t>(
          std::llround(frame.sim_time_s * 1.0e9));
      header.stamp.sec = static_cast<int32_t>(frame_time_ns / 1000000000LL);
      header.stamp.nanosec = static_cast<uint32_t>(
          frame_time_ns % 1000000000LL);
      if (traditional_detector) {
        // Traditional light-bar chain: no NN inference, no keypoint upscale
        // (corners come from the light-bar geometry + PCA refinement).
        dets = toArmorDetections(traditional_detector->detect(frame.image));
      } else {
        auto results = detector->detectBatch({frame.image}, {header});
        if (!results.empty()) {
          dets = results[0].detections;
        }
      }
      if (!dets.empty()) {
        pose_detections = dets;
        if (!traditional_detector) {
          // PnP-only keypoint correction for each render pipeline's own
          // keypoint bias (see simulation.yaml bridge.webots/gestalt).
          if (bridge_path == BridgePath::gestalt) {
            scaleDetectionKeypointsForPose(pose_detections, gestalt_sim_quad_scale);
          } else {
            scaleDetectionKeypointsForPose(pose_detections, webots_keypoint_scale);
          }
        }
        const auto pose_started = std::chrono::steady_clock::now();
        poses = pose_adapter->estimateBatch(
            pose_detections, cam_info, frame.R_cam2world());
        pose_latency_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - pose_started).count();
        armors = buildArmors(dets, poses);
      }
      pipeline.updateTracking(armors, frame.R_cam2world(), frame.t_cam2world, frame.sim_time_s);
    }

    // Control: gimbal feedback (radians) -> command (degrees) -> wire (radians).
    // "Now" = observation time + measured compute latency of this frame, so the
    // prediction horizon carries the real processing delay instead of assuming
    // the algorithm runs in zero time.
    const double processing_elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - processing_started).count();
    auto cmd_deg = pipeline.computeCommand(
        frame.gimbal_yaw, frame.gimbal_pitch,
        frame.sim_time_s + processing_elapsed_s);
    hfut::GimbalCommand out;
    out.yaw = cmd_deg.yaw * kDegToRad;
    out.pitch = cmd_deg.pitch * kDegToRad;
    out.yaw_diff = cmd_deg.yaw_diff * kDegToRad;
    out.pitch_diff = cmd_deg.pitch_diff * kDegToRad;
    out.yaw_vel = cmd_deg.yaw_v * kDegToRad;
    out.pitch_vel = cmd_deg.pitch_v * kDegToRad;
    out.yaw_acc = cmd_deg.yaw_a * kDegToRad;
    out.pitch_acc = cmd_deg.pitch_a * kDegToRad;
    out.distance = cmd_deg.distance;
    out.fire_advice = cmd_deg.fire_advice;
    out.mode = static_cast<hfut::GimbalMode>(cmd_deg.mode);
    if (gestalt_bridge && !gestalt_forward_auto_aim_velocity) {
      out.yaw_vel = 0.0;
      out.pitch_vel = 0.0;
      out.yaw_acc = 0.0;
      out.pitch_acc = 0.0;
    }
    bool idle_scan_override = false;
    if (gestalt_idle_scanner) {
      const bool auto_aim_active = out.mode == hfut::GimbalMode::normal_measurement;
      idle_scan_override = gestalt_idle_scanner->update(
          auto_aim_active, !dets.empty(), frame.gimbal_yaw, frame.gimbal_pitch,
          std::chrono::steady_clock::now(), out);
      const bool idle_scan_is_moving = gestalt_idle_scanner->scanning();
      if (idle_scan_is_moving != idle_scan_was_moving) {
        std::printf("[bringup_sim] gestalt idle scan %s\n",
                    idle_scan_is_moving ? "started" : "stopped");
        std::fflush(stdout);
        idle_scan_was_moving = idle_scan_is_moving;
      }
    }
    const double command_yaw_deg = out.yaw * kRadToDeg;
    const double command_pitch_deg = out.pitch * kRadToDeg;
    const double command_yaw_diff_deg = out.yaw_diff * kRadToDeg;
    const double command_pitch_diff_deg = out.pitch_diff * kRadToDeg;
    const double command_yaw_velocity_dps = out.yaw_vel * kRadToDeg;
    const double command_pitch_velocity_dps = out.pitch_vel * kRadToDeg;
    if (gestalt_bridge) {
      gestalt_bridge->send(out, frame.sim_time_s);
    } else {
      webots_gimbal->send(out, frame.sim_time_s);
    }

    if (diagnostics) {
      const auto& dbg = pipeline.lastDebug();
      diagnostics << "{\"seq\":" << frame.seq
                  << ",\"sim_time_s\":" << frame.sim_time_s
                  << ",\"input_mode\":\""
                  << (input_mode == hfut::io::BridgeInputMode::armor_pose ? "armor_pose" : "vision")
                  << "\""
                  << ",\"bridge_path\":\""
                  << (bridge_path == BridgePath::gestalt ? "gestalt" : "webots") << "\""
                  << ",\"command_mode\":" << static_cast<int>(out.mode)
                  << ",\"gestalt_idle_scan_override\":"
                  << (idle_scan_override ? "true" : "false")
                  << ",\"gestalt_idle_scan_moving\":"
                  << ((gestalt_idle_scanner && gestalt_idle_scanner->scanning())
                          ? "true" : "false")
                  << ",\"tracked_count\":" << dbg.num_tracked
                  << ",\"selected_id\":\"" << dbg.selected_id << "\""
                  << ",\"track_state\":" << dbg.selected_track_state
                  << ",\"tracker_update\":{\"valid\":"
                  << (dbg.tracker_update_valid ? "true" : "false")
                  << ",\"committed\":"
                  << (dbg.tracker_update_committed ? "true" : "false")
                  << ",\"obs_count\":" << dbg.tracker_observation_count
                  << ",\"top1_nis\":" << dbg.tracker_top1_nis
                  << ",\"top1_chi2_pos\":" << dbg.tracker_top1_chi2_pos
                  << ",\"top1_chi2_yaw\":" << dbg.tracker_top1_chi2_yaw
                  << ",\"top1_hypothesis\":\""
                  << escapeJsonString(dbg.tracker_top1_hypothesis) << "\""
                  << ",\"decision_reason\":\""
                  << escapeJsonString(dbg.tracker_decision_reason) << "\"}"
                  << ",\"direct_armors\":[";
      for (size_t direct_index = 0; direct_index < frame.direct_armors.size(); ++direct_index) {
        if (direct_index > 0) diagnostics << ',';
        const auto& direct = frame.direct_armors[direct_index];
        diagnostics << "{\"number\":\"" << direct.number
                    << "\",\"type\":\"" << direct.type
                    << "\",\"confidence\":" << direct.confidence
                    << ",\"position\":[" << direct.position_control.x() << ','
                    << direct.position_control.y() << ',' << direct.position_control.z()
                    << "],\"radial_yaw\":" << direct.radial_yaw
                    << ",\"position_noise_std_m\":"
                    << direct.position_noise_std_m
                    << ",\"yaw_noise_std_rad\":" << direct.yaw_noise_std_rad
                    << ",\"view_angle_rad\":" << direct.view_angle_rad
                    << ",\"surface_orientation_valid\":"
                    << (direct.surface_orientation_valid ? "true" : "false")
                    << ",\"surface_quaternion_wxyz\":["
                    << direct.surface_orientation_control.w() << ','
                    << direct.surface_orientation_control.x() << ','
                    << direct.surface_orientation_control.y() << ','
                    << direct.surface_orientation_control.z() << "]}";
      }
      diagnostics << ']'
                  << ",\"command\":{\"yaw_deg\":" << command_yaw_deg
                  << ",\"pitch_deg\":" << command_pitch_deg
                  << ",\"yaw_diff_deg\":" << command_yaw_diff_deg
                  << ",\"pitch_diff_deg\":" << command_pitch_diff_deg
                  << ",\"yaw_velocity_dps\":" << command_yaw_velocity_dps
                  << ",\"pitch_velocity_dps\":" << command_pitch_velocity_dps
                  << ",\"yaw_acceleration_dps2\":" << out.yaw_acc * kRadToDeg
                  << ",\"pitch_acceleration_dps2\":" << out.pitch_acc * kRadToDeg
                  << ",\"distance_m\":" << out.distance
                  << ",\"fire_advice\":" << (out.fire_advice ? 1 : 0)
                  << "}"
                  << ",\"mpc\":{\"valid\":"
                  << (dbg.mpc.valid ? "true" : "false")
                  << ",\"qp_success\":"
                  << (dbg.mpc.qp_success ? "true" : "false")
                  << ",\"fallback_used\":"
                  << (dbg.mpc.fallback_used ? "true" : "false")
                  << ",\"maneuver_path\":"
                  << (dbg.mpc.maneuver_path ? "true" : "false")
                  << ",\"cycle\":" << dbg.mpc.cycle
                  << ",\"qp_iterations\":" << dbg.mpc.qp_iterations
                  << ",\"active_bound_size\":" << dbg.mpc.active_bound_size
                  << ",\"active_linear_size\":" << dbg.mpc.active_linear_size
                  << ",\"active_set_size\":" << dbg.mpc.active_set_size
                  << ",\"qp_cost\":" << dbg.mpc.qp_cost
                  << ",\"regularization_eps\":" << dbg.mpc.regularization_eps
                  << ",\"dt\":" << dbg.mpc.dt
                  << ",\"current_state\":["
                  << dbg.mpc.current_state(0) << ',' << dbg.mpc.current_state(1)
                  << ',' << dbg.mpc.current_state(2) << ','
                  << dbg.mpc.current_state(3) << ']'
                  << ",\"next_state\":[" << dbg.mpc.next_state(0) << ','
                  << dbg.mpc.next_state(1) << ',' << dbg.mpc.next_state(2)
                  << ',' << dbg.mpc.next_state(3) << ']'
                  << ",\"applied_acceleration\":["
                  << dbg.mpc.applied_acceleration(0) << ','
                  << dbg.mpc.applied_acceleration(1) << ']'
                  << ",\"reference_states\":[";
      for (size_t index = 0; index < dbg.mpc.reference_states.size(); ++index) {
        if (index > 0) diagnostics << ',';
        const auto& state = dbg.mpc.reference_states[index];
        diagnostics << '[' << state(0) << ',' << state(1) << ',' << state(2)
                    << ',' << state(3) << ']';
      }
      diagnostics << "]}"
                  << ",\"control_target\":{\"valid\":"
                  << (dbg.control_target.valid ? "true" : "false")
                  << ",\"tracks_center\":"
                  << (dbg.control_target.tracks_center ? "true" : "false")
                  << ",\"virtual_target\":"
                  << (dbg.control_target.is_virtual_target ? "true" : "false")
                  << ",\"selected_index\":" << dbg.control_target.selected_index
                  << ",\"real_selected_index\":" << dbg.control_target.real_selected_index
                  << ",\"prediction_time_s\":" << dbg.control_target.prediction_time_s
                  << ",\"yaw_velocity_rad_s\":" << dbg.control_target.yaw_velocity
                  << ",\"current_center\":[" << dbg.control_target.current_center.x() << ','
                  << dbg.control_target.current_center.y() << ','
                  << dbg.control_target.current_center.z() << ']'
                  << ",\"predicted_center\":[" << dbg.control_target.predicted_center.x() << ','
                  << dbg.control_target.predicted_center.y() << ','
                  << dbg.control_target.predicted_center.z() << ']'
                  << ",\"linear_velocity\":[" << dbg.control_target.linear_velocity.x() << ','
                  << dbg.control_target.linear_velocity.y() << ','
                  << dbg.control_target.linear_velocity.z() << ']'
                  << ",\"current_selected_armor\":["
                  << dbg.control_target.current_selected_armor.x() << ','
                  << dbg.control_target.current_selected_armor.y() << ','
                  << dbg.control_target.current_selected_armor.z() << ']'
                  << ",\"control_target_position\":["
                  << dbg.control_target.control_target_position.x() << ','
                  << dbg.control_target.control_target_position.y() << ','
                  << dbg.control_target.control_target_position.z() << ']'
                  << ",\"current_armor_positions\":[";
      for (size_t index = 0;
           index < dbg.control_target.current_armor_positions.size(); ++index) {
        if (index > 0) diagnostics << ',';
        const auto& position = dbg.control_target.current_armor_positions[index];
        diagnostics << '[' << position.x() << ',' << position.y() << ','
                    << position.z() << ']';
      }
      diagnostics << "],\"predicted_armor_positions\":[";
      for (size_t index = 0;
           index < dbg.control_target.predicted_armor_positions.size(); ++index) {
        if (index > 0) diagnostics << ',';
        const auto& position = dbg.control_target.predicted_armor_positions[index];
        diagnostics << '[' << position.x() << ',' << position.y() << ','
                    << position.z() << ']';
      }
      diagnostics << ']'
                  << "}"
                  << ",\"delay\":{\"prediction_s\":"
                  << dbg.delay_audit.total_prediction_time_s
                  << ",\"flight_time_s\":" << dbg.delay_audit.flight_time_s
                  << ",\"processing_delay_s\":" << dbg.delay_audit.processing_delay_s
                  << ",\"control_latency_s\":" << dbg.delay_audit.control_latency_s
                  << ",\"fire_control_compensation_s\":"
                  << dbg.delay_audit.fire_control_compensation_s
                  << ",\"control_delay_steps\":" << dbg.delay_audit.control_delay_steps
                  << ",\"uses_delayed_b\":"
                  << (dbg.delay_audit.uses_delayed_b ? "true" : "false")
                  << ",\"double_compensation_risk\":"
                  << (dbg.delay_audit.double_compensation_risk ? "true" : "false")
                  << ",\"strategy\":\""
                  << escapeJsonString(dbg.delay_audit.strategy_name) << "\""
                  << "}"
                  << ",\"body_attitude\":{\"valid\":"
                  << (dbg.selected_attitude_valid ? "true" : "false")
                  << ",\"pitch_rad\":" << dbg.selected_body_pitch_rad
                  << ",\"roll_rad\":" << dbg.selected_body_roll_rad << "}"
                  << ",\"tracker_structure\":{\"valid\":"
                  << (dbg.tracker_structure_valid ? "true" : "false")
                  << ",\"r1\":" << dbg.tracker_r1
                  << ",\"r2\":" << dbg.tracker_r2
                  << ",\"dza\":" << dbg.tracker_dza << "}"
                  << ",\"state_estimate\":{\"valid\":"
                  << (dbg.selected_state_valid ? "true" : "false");
      if (dbg.selected_state_valid) {
        const auto& state = dbg.selected_state;
        diagnostics << ",\"full_state_valid\":"
                    << (state.full_state_valid ? "true" : "false")
                    << ",\"layout_attitude_valid\":"
                    << (state.layout_attitude_valid ? "true" : "false")
                    << ",\"position\":[" << state.center_position.x << ','
                    << state.center_position.y << ',' << state.center_position.z << ']'
                    << ",\"velocity\":[" << state.center_velocity.x << ','
                    << state.center_velocity.y << ',' << state.center_velocity.z << ']'
                    << ",\"acceleration\":[" << state.center_acceleration.x << ','
                    << state.center_acceleration.y << ','
                    << state.center_acceleration.z << ']'
                    << ",\"yaw\":" << state.yaw
                    << ",\"yaw_velocity\":" << state.yaw_velocity
                    << ",\"yaw_acceleration\":" << state.yaw_acceleration
                    << ",\"center_quaternion_wxyz\":["
                    << state.center_pose.orientation.w << ','
                    << state.center_pose.orientation.x << ','
                    << state.center_pose.orientation.y << ','
                    << state.center_pose.orientation.z << ']'
                    << ",\"angular_velocity\":["
                    << state.center_twist.angular.x << ','
                    << state.center_twist.angular.y << ','
                    << state.center_twist.angular.z << ']'
                    << ",\"angular_acceleration\":["
                    << state.center_accel.angular.x << ','
                    << state.center_accel.angular.y << ','
                    << state.center_accel.angular.z << ']'
                    << ",\"r1\":" << state.radius
                    << ",\"r2\":" << state.radius_2
                    << ",\"dza\":" << state.d_za
                    << ",\"covariance_dim\":"
                    << static_cast<int>(state.covariance_dim)
                    << ",\"covariance_diag\":[";
        const size_t covariance_dim = state.covariance_dim;
        for (size_t index = 0; index < covariance_dim; ++index) {
          if (index > 0) diagnostics << ',';
          const size_t flat_index = index * covariance_dim + index;
          diagnostics << (flat_index < state.state_covariance.size()
                              ? state.state_covariance[flat_index]
                              : 0.0);
        }
        diagnostics << "],\"armor_offsets\":[";
        for (size_t index = 0; index < state.armors_offset.size(); ++index) {
          if (index > 0) diagnostics << ',';
          const auto& offset = state.armors_offset[index];
          diagnostics << "{\"position\":[" << offset.position.x << ','
                      << offset.position.y << ',' << offset.position.z
                      << "],\"quaternion_wxyz\":[" << offset.orientation.w << ','
                      << offset.orientation.x << ',' << offset.orientation.y << ','
                      << offset.orientation.z << "]}";
        }
        diagnostics << ']';
      }
      diagnostics << '}'
                  << ",\"tracked_armors\":[";
      for (size_t index = 0; index < dbg.tracked_armor_poses.size(); ++index) {
        if (index > 0) diagnostics << ',';
        const auto& armor = dbg.tracked_armor_poses[index];
        diagnostics << "{\"position\":[" << armor.position.x() << ','
                    << armor.position.y() << ',' << armor.position.z()
                    << "],\"normal\":[" << armor.normal.x() << ','
                    << armor.normal.y() << ',' << armor.normal.z()
                    << "],\"width_axis\":[" << armor.width_axis.x() << ','
                    << armor.width_axis.y() << ',' << armor.width_axis.z()
                    << "],\"height_axis\":[" << armor.height_axis.x() << ','
                    << armor.height_axis.y() << ',' << armor.height_axis.z()
                    << "]}";
      }
      diagnostics << ']'
                  << ",\"detections\":[";
      for (size_t detection_index = 0; detection_index < dets.size(); ++detection_index) {
        if (detection_index > 0) diagnostics << ',';
        const auto& detection = dets[detection_index];
        diagnostics << "{\"number\":\"" << detection.publish_number
                    << "\",\"type\":\"" << detection.publish_type
                    << "\",\"track_id\":" << detection.track_id
                    << ",\"confidence\":" << detection.confidence
                    << ",\"keypoints\":[";
        for (size_t corner_index = 0; corner_index < detection.keypoints.size(); ++corner_index) {
          if (corner_index > 0) diagnostics << ',';
          diagnostics << '[' << detection.keypoints[corner_index].x << ','
                      << detection.keypoints[corner_index].y << ']';
        }
        diagnostics << ']';
        if (detection_index < pose_detections.size()) {
          diagnostics << ",\"pose_keypoints\":[";
          const auto& pose_detection = pose_detections[detection_index];
          for (size_t corner_index = 0;
               corner_index < pose_detection.keypoints.size(); ++corner_index) {
            if (corner_index > 0) diagnostics << ',';
            diagnostics << '[' << pose_detection.keypoints[corner_index].x << ','
                        << pose_detection.keypoints[corner_index].y << ']';
          }
          diagnostics << ']';
        }
        if (detection_index < poses.size() && poses[detection_index].valid) {
          const auto& pose = poses[detection_index];
          diagnostics << ",\"pose\":{\"translation\":["
                      << pose.translation.x() << ',' << pose.translation.y() << ','
                      << pose.translation.z() << "],\"rotation_wxyz\":["
                      << pose.rotation.w() << ',' << pose.rotation.x() << ','
                      << pose.rotation.y() << ',' << pose.rotation.z()
                      << "],\"yaw_pitch_roll\":[" << pose.yaw << ',' << pose.pitch
                      << ',' << pose.roll << "],\"mode\":"
                      << static_cast<int>(pose.mode) << ",\"quality_score\":"
                      << pose.quality_score << ",\"reprojection_error\":"
                      << pose.reprojection_error << '}';
        } else {
          diagnostics << ",\"pose\":null";
        }
        diagnostics << '}';
      }
      diagnostics << "]}\n";
      if ((frames & 0x0fU) == 0) diagnostics.flush();
    }

    const auto frame_done = std::chrono::steady_clock::now();
    const double frame_interval =
        std::chrono::duration<double>(frame_done - last_frame_done).count();
    if (frame_interval > 1e-6) {
      const double instantaneous_fps = 1.0 / frame_interval;
      smoothed_fps = smoothed_fps <= 0.0 ? instantaneous_fps :
          0.9 * smoothed_fps + 0.1 * instantaneous_fps;
    }
    last_frame_done = frame_done;
    const double loop_latency_ms = std::chrono::duration<double, std::milli>(
        frame_done - processing_started).count();

    // ── Debug: stream scalars to PlotJuggler + show detection overlay ──
    if (debug) {
      const auto& dbg = pipeline.lastDebug();
      plotter->plot({
          {"sim_t", frame.sim_time_s},
          {"source_fps", smoothed_source_fps},
          {"control_fps", smoothed_fps},
          {"gimbal_yaw_deg", frame.gimbal_yaw * kRadToDeg},
          {"gimbal_pitch_deg", frame.gimbal_pitch * kRadToDeg},
          {"gimbal_yaw_vel", frame.gimbal_yaw_vel},
          {"cmd_yaw_deg", command_yaw_deg},
          {"cmd_pitch_deg", command_pitch_deg},
          {"cmd_yaw_vel", command_yaw_velocity_dps},
          {"cmd_yaw_acc", out.yaw_acc * kRadToDeg},
          {"distance", out.distance},
          {"armors", static_cast<double>(armors.armors.size())},
          {"fire", out.fire_advice ? 1.0 : 0.0},
          {"mode", static_cast<double>(out.mode)},
          {"gestalt_idle_scan", idle_scan_override ? 1.0 : 0.0},
      });

      cv::Mat vis = frame.image.empty()
          ? cv::Mat(frame.intrinsics.height, frame.intrinsics.width, CV_8UC3, cv::Scalar(12, 15, 18))
          : frame.image.clone();
      drawer->drawDetections(vis, dets, /*show_confidence=*/true);
      drawer->drawCrosshair(vis);

      std::vector<InputArmorOverlay> input_armor_overlays;
      if (input_mode == hfut::io::BridgeInputMode::vision) {
        const size_t count = std::min(dets.size(), poses.size());
        input_armor_overlays.reserve(count);
        for (size_t index = 0; index < count; ++index) {
          const auto& pose = poses[index];
          if (!pose.valid || !pose.translation.allFinite() ||
              !pose.rotation.coeffs().allFinite() || pose.rotation.norm() <= 1e-9) {
            continue;
          }

          const Eigen::Matrix3d armor_rotation_control =
              frame.R_cam2world() * pose.rotation.normalized().toRotationMatrix();
          const Eigen::Vector3d armor_center_control =
              frame.R_cam2world() * pose.translation + frame.t_cam2world;
          const double armor_width = dets[index].publish_type == "large"
              ? dcfg.pose.large_armor_width
              : dcfg.pose.small_armor_width;
          const double armor_height = dets[index].publish_type == "large"
              ? dcfg.pose.large_armor_height
              : dcfg.pose.small_armor_height;

          InputArmorOverlay overlay;
          if (!projectArmorOutline(
                  frame, armor_center_control,
                  armor_rotation_control.col(1), armor_rotation_control.col(2),
                  armor_width, armor_height, vis, overlay.outline) ||
              !projectControlPoint(frame, armor_center_control, overlay.center)) {
            continue;
          }
          overlay.has_normal_tip = projectControlPoint(
              frame,
              armor_center_control + armor_rotation_control.col(0) * 0.10,
              overlay.normal_tip);
          overlay.label = "VISION " + dets[index].publish_number;
          input_armor_overlays.push_back(std::move(overlay));
        }
      } else {
        input_armor_overlays.reserve(frame.direct_armors.size());
        for (const auto& armor : frame.direct_armors) {
          Eigen::Matrix3d armor_rotation_control;
          if (armor.surface_orientation_valid &&
              armor.surface_orientation_control.coeffs().allFinite() &&
              armor.surface_orientation_control.norm() > 1e-9) {
            armor_rotation_control =
                armor.surface_orientation_control.normalized().toRotationMatrix();
          } else {
            const double normal_yaw = armor.radial_yaw - M_PI;
            armor_rotation_control = Eigen::AngleAxisd(
                normal_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          }
          const double armor_width = armor.type == "large"
              ? dcfg.pose.large_armor_width
              : dcfg.pose.small_armor_width;
          const double armor_height = armor.type == "large"
              ? dcfg.pose.large_armor_height
              : dcfg.pose.small_armor_height;

          InputArmorOverlay overlay;
          if (!projectArmorOutline(
                  frame, armor.position_control,
                  armor_rotation_control.col(1), armor_rotation_control.col(2),
                  armor_width, armor_height, vis, overlay.outline) ||
              !projectControlPoint(frame, armor.position_control, overlay.center)) {
            continue;
          }
          overlay.has_normal_tip = projectControlPoint(
              frame,
              armor.position_control + armor_rotation_control.col(0) * 0.10,
              overlay.normal_tip);
          overlay.label = "ARMOR_POSE " + armor.number;
          input_armor_overlays.push_back(std::move(overlay));
        }
      }

      DebugOverlayGeometry geometry;
      const auto& control = dbg.control_target;
      // Estimator's full-vehicle plates: visible for the whole track lifetime.
      // While the controller is aiming (control.valid) the AIM-time plates
      // below carry the same geometry; outside AIM (tracking / temp-lost)
      // draw the tracked robot's own estimate instead of showing nothing.
      if (!control.valid && !dbg.tracked_armor_poses.empty()) {
        for (const auto& pose : dbg.tracked_armor_poses) {
          std::array<cv::Point2f, 4> outline;
          if (projectArmorOutline(
                frame, pose.position, pose.width_axis, pose.height_axis,
                dbg.tracked_armor_width_m, dbg.tracked_armor_height_m,
                vis, outline)) {
            geometry.current_armor_outlines.push_back(outline);
          }
        }
      }
      if (control.valid) {
        cv::Point2f point;
        if (projectControlPoint(frame, control.current_center, point) && pointInImage(point, vis)) {
          geometry.has_center = true;
          geometry.center = point;
        }
        if (projectControlPoint(frame, control.predicted_center, point) && pointInImage(point, vis)) {
          geometry.has_predicted_center = true;
          geometry.predicted_center = point;
        }
        const Eigen::Vector3d velocity_tip =
            control.current_center + control.linear_velocity * 0.35;
        if (projectControlPoint(frame, velocity_tip, point) && pointInImage(point, vis)) {
          geometry.has_velocity_tip = true;
          geometry.velocity_tip = point;
        }
        for (size_t armor_index = 0;
             armor_index < control.current_armor_positions.size();
             ++armor_index) {
          const auto& armor = control.current_armor_positions[armor_index];
          Eigen::Vector3d width_axis = Eigen::Vector3d::UnitY();
          Eigen::Vector3d height_axis = Eigen::Vector3d::UnitZ();
          if (armor_index < control.current_armor_width_axes.size() &&
              armor_index < control.current_armor_height_axes.size()) {
            width_axis = control.current_armor_width_axes[armor_index];
            height_axis = control.current_armor_height_axes[armor_index];
          } else {
            // Legacy/single-armor fallback: keep the normal Z-dominant
            // geometry when no complete armor surface frame is available.
            Eigen::Vector3d radial = armor - control.current_center;
            radial.z() = 0.0;
            if (radial.norm() > 1e-6) {
              radial.normalize();
              width_axis = Eigen::Vector3d(-radial.y(), radial.x(), 0.0);
            }
          }
          std::array<cv::Point2f, 4> outline;
          static bool printed_axes = false;
          if (!printed_axes && std::getenv("HFUT_DEBUG_AXES") != nullptr) {
            std::fprintf(stderr,
                "[axes] i=%zu width=(%+.3f,%+.3f,%+.3f) height=(%+.3f,%+.3f,%+.3f) "
                "tilt_height_z=%.3f n_axes=%zu/%zu\n",
                armor_index, width_axis.x(), width_axis.y(), width_axis.z(),
                height_axis.x(), height_axis.y(), height_axis.z(),
                height_axis.z(),
                control.current_armor_width_axes.size(),
                control.current_armor_height_axes.size());
            if (armor_index + 1 >= control.current_armor_positions.size())
              printed_axes = true;
          }
          if (projectArmorOutline(
                frame, armor, width_axis, height_axis,
                control.armor_width_m, control.armor_height_m, vis, outline)) {
            geometry.current_armor_outlines.push_back(outline);
          }
        }
        if (!control.tracks_center &&
            projectControlPoint(frame, control.current_selected_armor, point) &&
            pointInImage(point, vis)) {
          geometry.has_current_selected_armor = true;
          geometry.current_selected_armor = point;
        }
        if (projectControlPoint(frame, control.control_target_position, point) &&
            pointInImage(point, vis)) {
          if (!control.tracks_center) {
            geometry.locked_detection_index = findLockedDetection(dets, dbg.selected_id, point);
          }
        }
      }

      if (dbg.fire_advice.evaluated && dbg.fire_advice.valid) {
        cv::Point2f point;
        if (projectControlPoint(frame, dbg.fire_advice.armor_center, point) &&
            pointInImage(point, vis)) {
          geometry.has_fire_target = true;
          geometry.fire_target = point;
        }
      }

      if (control.valid && std::isfinite(out.yaw) && std::isfinite(out.pitch)) {
        const double yaw = out.yaw;
        const double pitch = out.pitch;
        const Eigen::Vector3d command_direction(
            std::cos(pitch) * std::cos(yaw),
            std::cos(pitch) * std::sin(yaw),
            std::sin(pitch));
        const Eigen::Vector3d camera_ray_aim_point =
            frame.t_cam2world + command_direction * 2.0;
        cv::Point2f point;
        if (projectControlPoint(frame, camera_ray_aim_point, point) && pointInImage(point, vis)) {
          geometry.has_control_target = true;
          geometry.control_target = point;
        }
      }

      DebugOverlayStatus status;
      status.fire_advice = out.fire_advice;
      status.fire_evaluated = dbg.fire_advice.evaluated;
      status.fire_valid = dbg.fire_advice.valid;
      status.fire_facing_ok = dbg.fire_advice.best_candidate_facing_ok;
      status.probability_enabled = dbg.fire_advice.probability_enabled;
      status.tracking = dbg.context_is_tracking;
      status.temp_lost = dbg.context_is_temp_lost;
      status.stale = dbg.stale;
      status.tracks_center = control.tracks_center;
      status.virtual_target = control.is_virtual_target;
      status.mode = static_cast<int8_t>(out.mode);
      status.track_state = dbg.selected_track_state;
      status.detections = static_cast<int>(armors.armors.size());
      status.tracked_targets = dbg.num_tracked;
      status.fire_candidates = dbg.fire_advice.candidate_count_total;
      status.target_id = dbg.selected_id;
      status.strategy = dbg.delay_audit.strategy_name;
      status.gimbal_yaw_rad = frame.gimbal_yaw;
      status.gimbal_pitch_rad = frame.gimbal_pitch;
      status.command_yaw_rad = out.yaw;
      status.command_pitch_rad = out.pitch;
      status.yaw_diff_rad = out.yaw_diff;
      status.pitch_diff_rad = out.pitch_diff;
      status.distance_m = out.distance;
      status.target_speed_mps = control.linear_velocity.norm();
      status.target_yaw_rate_radps = control.yaw_velocity;
      if (dbg.tracker_structure_valid) {
        status.r1_m = dbg.tracker_r1;
        status.r2_m = dbg.tracker_r2;
      }
      status.prediction_ms = dbg.delay_audit.total_prediction_time_s * 1e3;
      status.flight_ms = dbg.delay_audit.flight_time_s * 1e3;
      status.data_age_ms = dbg.data_age * 1e3;
      status.body_attitude_valid = dbg.selected_attitude_valid;
      status.body_pitch_rad = dbg.selected_body_pitch_rad;
      status.body_roll_rad = dbg.selected_body_roll_rad;
      status.fire_yaw_error_rad = dbg.fire_advice.yaw_error;
      status.fire_pitch_error_rad = dbg.fire_advice.pitch_error;
      status.hit_probability = dbg.fire_advice.p_hit_window;
      status.fps = smoothed_fps;
      status.loop_latency_ms = loop_latency_ms;
      status.inference_ms = detector ? detector->lastProfiler().infer_ms : 0.0;
      status.pose_ms = pose_latency_ms;
      if (!status.fire_advice) {
        if (status.temp_lost) status.fire_hold_reason = "NO OBS";
        else if (!status.fire_evaluated) status.fire_hold_reason = "NOT EVAL";
        else if (!status.fire_valid) status.fire_hold_reason = "INVALID";
        else if (!status.fire_facing_ok) status.fire_hold_reason = "BAD FACE";
        else status.fire_hold_reason = status.probability_enabled ? "PROB GATE" : "AIM GATE";
      }
      drawer->drawControlOverlay(vis, dets, status, geometry);
      drawInputArmorOverlays(
          vis, input_armor_overlays,
          input_mode == hfut::io::BridgeInputMode::vision
              ? cv::Scalar(0, 165, 255)
              : cv::Scalar(255, 255, 0));

      if (!debug_snapshot_path.empty() && (frames % 30U) == 0U) {
        cv::imwrite(debug_snapshot_path, vis);
      }

      // WSLg/XWayland presents X11 apps at 96 DPI regardless of the Windows
      // display scaling, so on high-DPI screens the imshow window can come up
      // tiny and resizeWindow is unreliable there. Scale the shown image
      // directly instead; HFUT_DEBUG_WINDOW_SCALE (default 1.0) compensates.
      if (debug_window_scale != 1.0) {
        cv::Mat scaled;
        cv::resize(vis, scaled, cv::Size(), debug_window_scale,
                   debug_window_scale, cv::INTER_LINEAR);
        cv::imshow("hfut_auto_aim", scaled);
      } else {
        cv::imshow("hfut_auto_aim", vis);
      }
      if (cv::waitKey(1) == 'q') g_stop.store(true);
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::milliseconds(500)) {
      const auto& dbg = pipeline.lastDebug();
      std::printf("[bringup_sim] seq=%llu t=%.2f armors=%zu cmd(yaw=%.2f pitch=%.2f deg) "
                  "fire=%d mode=%d scan=%d | tracked=%d sel=%s state=%d "
                  "ctx_track=%d age=%.3f stale=%d | source=%.1fHz loop=%.1fHz "
                  "process=%.1fms infer=%.1fms dropped=%llu\n",
                  (unsigned long long)frame.seq, frame.sim_time_s, armors.armors.size(),
                  command_yaw_deg, command_pitch_deg, (int)out.fire_advice, (int)out.mode,
                  (int)idle_scan_override,
                  dbg.num_tracked, dbg.selected_id.c_str(), dbg.selected_track_state,
                  (int)dbg.context_is_tracking, dbg.data_age, (int)dbg.stale,
                  smoothed_source_fps, smoothed_fps, loop_latency_ms,
                  detector ? detector->lastProfiler().infer_ms : 0.0,
                  (unsigned long long)(gestalt_frame_receiver
                      ? gestalt_frame_receiver->droppedFrames() : 0));
      std::fflush(stdout);
      last_log = now;
    }
  }
  std::printf("[bringup_sim] stopped after %llu frames\n", (unsigned long long)frames);
  return 0;
}
