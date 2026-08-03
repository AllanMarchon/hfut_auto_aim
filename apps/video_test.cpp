// Offline real-camera validation path:
// video -> detector -> PnP -> tracker -> prediction -> JSONL/annotated video.
#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

#include "hfut_auto_aim/video_calibration.hpp"

#include "config_loader.hpp"
#include "armor_detector_nn/core/armor_detector_nn.hpp"
#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"
#include "armor_detector_nn/core/pose_refine/pose_refiner.hpp"
#include "armor_detector_nn/debug/debug_drawer.hpp"
#include "pipeline.hpp"
#include "rm_utils/logger/log.hpp"

#include <rm_interfaces/msg/armors.hpp>

namespace {

using fyt::auto_aim::ArmorDetection;
using fyt::auto_aim::ArmorDetectorNN;
using fyt::auto_aim::ArmorPoseEstimatorAdapter;
using fyt::auto_aim::DebugDrawer;
using fyt::auto_aim::DetectorConfig;
using fyt::auto_aim::PoseEstimate;
using fyt::auto_aim::SingleYawRefiner;

constexpr double kDegToRad = M_PI / 180.0;

// Camera optical frame (x right, y down, z forward) to the fixed control frame
// (x forward, y left, z up). The recording is treated as a stationary camera.
Eigen::Matrix3d opticalToControl() {
  Eigen::Matrix3d rotation;
  rotation << 0.0, 0.0, 1.0,
             -1.0, 0.0, 0.0,
              0.0,-1.0, 0.0;
  return rotation;
}

std::string escapeJson(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          escaped += '?';
        } else {
          escaped += ch;
        }
    }
  }
  return escaped;
}

struct Options {
  std::string video_path{"test_video/output.avi"};
  std::string camera_info_path{"test_video/camera_info.yaml"};
  std::string config_dir{"configs"};
  std::string overlay_path{"/tmp/hfut_video_test_overlay.avi"};
  std::string diagnostics_path{"/tmp/hfut_video_test.jsonl"};
  std::string summary_path{"/tmp/hfut_video_test_summary.json"};
  std::string calibration_mode{"center_crop"};
  std::string enemy_color{"red"};
  int start_frame{0};
  int max_frames{-1};
  int frame_step{1};
  bool display{false};
  bool realtime{false};
  bool help{false};
};

std::string optionValue(
    int argc, char** argv, int& index, const std::string& arg,
    const std::string& name) {
  const std::string prefix = name + "=";
  if (arg.compare(0, prefix.size(), prefix) == 0) {
    return arg.substr(prefix.size());
  }
  if (arg == name && index + 1 < argc) {
    return argv[++index];
  }
  return {};
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      options.help = true;
    } else if (arg == "--display") {
      options.display = true;
    } else if (arg == "--realtime") {
      options.realtime = true;
    } else if (arg == "--no-overlay") {
      options.overlay_path.clear();
    } else if (auto value = optionValue(argc, argv, i, arg, "--video"); !value.empty()) {
      options.video_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-info"); !value.empty()) {
      options.camera_info_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--config-dir"); !value.empty()) {
      options.config_dir = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--output"); !value.empty()) {
      options.overlay_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--diagnostics"); !value.empty()) {
      options.diagnostics_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--summary"); !value.empty()) {
      options.summary_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--calibration-mode"); !value.empty()) {
      options.calibration_mode = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--enemy-color"); !value.empty()) {
      options.enemy_color = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--start-frame"); !value.empty()) {
      options.start_frame = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--max-frames"); !value.empty()) {
      options.max_frames = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--frame-step"); !value.empty()) {
      options.frame_step = std::stoi(value);
    } else {
      throw std::invalid_argument("unknown or incomplete option: " + arg);
    }
  }
  if (options.start_frame < 0 || options.frame_step <= 0 || options.max_frames == 0) {
    throw std::invalid_argument(
        "start-frame must be >= 0, frame-step > 0, and max-frames != 0");
  }
  return options;
}

void printUsage() {
  std::printf(
      "Usage: video_test [options]\n"
      "  --video PATH                 input video (default test_video/output.avi)\n"
      "  --camera-info PATH           ROS camera_info YAML\n"
      "  --config-dir PATH            directory containing detector/tracker/controller configs\n"
      "  --calibration-mode MODE      strict | scale | center_crop (default)\n"
      "  --enemy-color COLOR          red | blue | white\n"
      "  --output PATH                annotated AVI/MP4 output\n"
      "  --diagnostics PATH           per-frame JSONL output\n"
      "  --summary PATH               aggregate JSON output\n"
      "  --start-frame N              first source frame\n"
      "  --max-frames N               processed frame limit (-1 = all)\n"
      "  --frame-step N               process every Nth source frame\n"
      "  --display                    show the annotated video\n"
      "  --realtime                   pace processing from source timestamps\n"
      "  --no-overlay                 disable annotated video output\n");
}

hfut::video::CameraCalibration loadCalibration(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  hfut::video::CameraCalibration calibration;
  if (!root["image_width"] || !root["image_height"] ||
      !root["camera_matrix"] || !root["camera_matrix"]["data"]) {
    throw std::invalid_argument("camera_info YAML is missing image dimensions or camera_matrix.data");
  }
  calibration.width = root["image_width"].as<int>();
  calibration.height = root["image_height"].as<int>();
  const auto matrix = root["camera_matrix"]["data"];
  if (!matrix.IsSequence() || matrix.size() != 9) {
    throw std::invalid_argument("camera_matrix.data must contain 9 values");
  }
  for (size_t i = 0; i < 9; ++i) calibration.k[i] = matrix[i].as<double>();
  const auto distortion = root["distortion_coefficients"];
  if (distortion && distortion["data"]) {
    calibration.d = distortion["data"].as<std::vector<double>>();
  }
  if (calibration.d.empty()) calibration.d.assign(5, 0.0);
  for (double value : calibration.d) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("distortion coefficients contain a non-finite value");
    }
  }
  return calibration;
}

sensor_msgs::msg::CameraInfo toCameraInfo(
    const hfut::video::CameraCalibration& calibration) {
  sensor_msgs::msg::CameraInfo info;
  info.width = calibration.width;
  info.height = calibration.height;
  info.k = calibration.k;
  info.d = calibration.d;
  return info;
}

fyt::EnemyColor parseEnemyColor(const std::string& value) {
  if (value == "red") return fyt::EnemyColor::RED;
  if (value == "blue") return fyt::EnemyColor::BLUE;
  if (value == "white") return fyt::EnemyColor::WHITE;
  throw std::invalid_argument("enemy color must be red, blue, or white, got: " + value);
}

rm_interfaces::msg::Armors buildValidArmors(
    const std::vector<ArmorDetection>& detections,
    const std::vector<PoseEstimate>& poses) {
  rm_interfaces::msg::Armors message;
  const size_t count = std::min(detections.size(), poses.size());
  for (size_t i = 0; i < count; ++i) {
    if (!poses[i].valid || !poses[i].translation.allFinite() ||
        poses[i].translation.z() <= 0.0) {
      continue;
    }
    const auto& detection = detections[i];
    const auto& pose = poses[i];
    rm_interfaces::msg::Armor armor;
    armor.number = detection.publish_number;
    armor.type = detection.publish_type;
    armor.detection_confidence = detection.confidence;
    armor.track_id = detection.track_id;
    armor.has_image_geometry = true;
    armor.bbox_xywh = {detection.bbox.x, detection.bbox.y,
                       detection.bbox.width, detection.bbox.height};
    for (size_t corner = 0; corner < 4; ++corner) {
      armor.image_corners[corner].x = detection.keypoints[corner].x;
      armor.image_corners[corner].y = detection.keypoints[corner].y;
    }
    armor.pose.position.x = pose.translation.x();
    armor.pose.position.y = pose.translation.y();
    armor.pose.position.z = pose.translation.z();
    armor.pose.orientation.x = pose.rotation.x();
    armor.pose.orientation.y = pose.rotation.y();
    armor.pose.orientation.z = pose.rotation.z();
    armor.pose.orientation.w = pose.rotation.w();
    armor.pose_estimate_mode = static_cast<uint8_t>(pose.mode);
    armor.pose_quality_score = static_cast<float>(pose.quality_score);
    armor.reproj_error_raw = static_cast<float>(pose.reproj_error_raw);
    armor.reproj_error_refined = static_cast<float>(pose.reproj_error_refined);
    armor.pose_condition_number = static_cast<float>(pose.condition_number);
    armor.pose_num_points = static_cast<uint16_t>(std::max(pose.num_points, 0));
    armor.pose_num_inliers = static_cast<uint16_t>(std::max(pose.num_inliers, 0));
    armor.pose_covariance_valid = pose.covariance_valid;
    armor.ippe_yaw_ambiguity = static_cast<float>(pose.ippe_yaw_ambiguity);
    if (pose.covariance_valid) {
      for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
          armor.pose_covariance_xyz_yaw[row * 4 + column] =
              pose.covariance_xyz_yaw(row, column);
        }
      }
    }
    message.armors.push_back(std::move(armor));
  }
  return message;
}

bool projectControlPoint(
    const Eigen::Vector3d& point_control, const Eigen::Matrix3d& camera_to_control,
    const hfut::video::CameraCalibration& calibration, cv::Point2f& pixel) {
  if (!point_control.allFinite()) return false;
  const Eigen::Vector3d point_camera = camera_to_control.transpose() * point_control;
  if (!point_camera.allFinite() || point_camera.z() <= 0.02) return false;
  pixel.x = static_cast<float>(
      calibration.k[0] * point_camera.x() / point_camera.z() + calibration.k[2]);
  pixel.y = static_cast<float>(
      calibration.k[4] * point_camera.y() / point_camera.z() + calibration.k[5]);
  return std::isfinite(pixel.x) && std::isfinite(pixel.y);
}

bool projectArmorOutline(
    const fyt::auto_aim::robot_description::ArmorWorldPose& armor,
    double width, double height, const Eigen::Matrix3d& camera_to_control,
    const hfut::video::CameraCalibration& calibration,
    std::array<cv::Point2f, 4>& pixels) {
  Eigen::Vector3d width_axis = armor.width_axis;
  Eigen::Vector3d height_axis = armor.height_axis;
  if (!armor.position.allFinite() || width_axis.norm() < 1e-6 ||
      height_axis.norm() < 1e-6) {
    return false;
  }
  width_axis.normalize();
  height_axis -= width_axis * height_axis.dot(width_axis);
  if (height_axis.norm() < 1e-6) return false;
  height_axis.normalize();
  const std::array<Eigen::Vector3d, 4> corners = {
      armor.position - width_axis * width * 0.5 + height_axis * height * 0.5,
      armor.position + width_axis * width * 0.5 + height_axis * height * 0.5,
      armor.position + width_axis * width * 0.5 - height_axis * height * 0.5,
      armor.position - width_axis * width * 0.5 - height_axis * height * 0.5};
  for (size_t i = 0; i < corners.size(); ++i) {
    if (!projectControlPoint(corners[i], camera_to_control, calibration, pixels[i])) {
      return false;
    }
  }
  return true;
}

void putOutlined(
    cv::Mat& image, const std::string& text, cv::Point origin,
    double scale, const cv::Scalar& color) {
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
              cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
              color, 1, cv::LINE_AA);
}

void drawPipelineOverlay(
    cv::Mat& image, const std::vector<ArmorDetection>& detections,
    const std::vector<PoseEstimate>& poses,
    const hfut::pipeline::Pipeline::DebugSnapshot& debug,
    const Eigen::Matrix3d& camera_to_control,
    const hfut::video::CameraCalibration& calibration,
    double source_time_s, double processing_ms) {
  const cv::Scalar observed_color(30, 170, 255);
  const cv::Scalar state_color(40, 255, 255);
  const cv::Scalar predicted_color(255, 100, 255);
  for (size_t i = 0; i < detections.size() && i < poses.size(); ++i) {
    if (!poses[i].valid) continue;
    const cv::Point center(cvRound(detections[i].center.x),
                           cvRound(detections[i].center.y));
    cv::circle(image, center, 4, observed_color, cv::FILLED, cv::LINE_AA);
    std::ostringstream label;
    label << "PnP " << std::fixed << std::setprecision(2)
          << poses[i].translation.z() << "m e="
          << poses[i].reproj_error_refined << "px";
    putOutlined(image, label.str(), center + cv::Point(8, 17), 0.38, observed_color);
  }

  if (debug.selected_state_valid) {
    const auto& state = debug.selected_state;
    const Eigen::Vector3d center(
        state.center_position.x, state.center_position.y, state.center_position.z);
    const Eigen::Vector3d velocity(
        state.center_velocity.x, state.center_velocity.y, state.center_velocity.z);
    cv::Point2f center_pixel;
    if (projectControlPoint(center, camera_to_control, calibration, center_pixel)) {
      cv::circle(image, center_pixel, 7, state_color, 2, cv::LINE_AA);
      cv::Point2f velocity_pixel;
      if (projectControlPoint(center + velocity * 0.25, camera_to_control,
                              calibration, velocity_pixel)) {
        cv::arrowedLine(image, center_pixel, velocity_pixel, state_color, 2,
                        cv::LINE_AA, 0, 0.25);
      }
    }
  }

  for (const auto& armor : debug.tracked_armor_poses) {
    std::array<cv::Point2f, 4> outline;
    if (!projectArmorOutline(armor, debug.tracked_armor_width_m,
                             debug.tracked_armor_height_m, camera_to_control,
                             calibration, outline)) {
      continue;
    }
    std::vector<cv::Point> polygon;
    for (const auto& point : outline) polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    cv::polylines(image, polygon, true, state_color, 1, cv::LINE_AA);
  }

  const auto& prediction = debug.control_target;
  if (prediction.valid) {
    cv::Point2f current_pixel;
    cv::Point2f predicted_pixel;
    if (projectControlPoint(prediction.current_center, camera_to_control,
                            calibration, current_pixel) &&
        projectControlPoint(prediction.predicted_center, camera_to_control,
                            calibration, predicted_pixel)) {
      cv::line(image, current_pixel, predicted_pixel, predicted_color, 2, cv::LINE_AA);
      cv::circle(image, predicted_pixel, 8, predicted_color, 2, cv::LINE_AA);
    }
    for (const auto& armor : prediction.predicted_armor_positions) {
      cv::Point2f point;
      if (projectControlPoint(armor, camera_to_control, calibration, point)) {
        cv::drawMarker(image, point, predicted_color, cv::MARKER_DIAMOND, 12, 2, cv::LINE_AA);
      }
    }
  }

  const int panel_height = std::min(120, image.rows);
  const int panel_width = std::min(570, image.cols);
  cv::Mat panel = image(cv::Rect(0, 0, panel_width, panel_height));
  cv::Mat tint(panel.size(), panel.type(), cv::Scalar(15, 18, 20));
  cv::addWeighted(tint, 0.70, panel, 0.30, 0.0, panel);
  std::ostringstream line1;
  line1 << "VIDEO t=" << std::fixed << std::setprecision(2) << source_time_s
        << "s  detect=" << detections.size() << " pose="
        << std::count_if(poses.begin(), poses.end(),
                         [](const PoseEstimate& pose) { return pose.valid; })
        << "  process=" << std::setprecision(1) << processing_ms << "ms";
  putOutlined(image, line1.str(), cv::Point(12, 25), 0.55, cv::Scalar(240, 240, 240));
  std::ostringstream line2;
  line2 << "target=" << (debug.selected_id.empty() ? "none" : debug.selected_id)
        << " state=" << debug.selected_track_state
        << " tracked=" << debug.num_tracked
        << " update=" << (debug.tracker_update_committed ? "commit" : "hold")
        << " cmd=" << static_cast<int>(debug.command.mode);
  putOutlined(image, line2.str(), cv::Point(12, 51), 0.52, state_color);
  if (debug.selected_state_valid) {
    const auto& state = debug.selected_state;
    const double speed = std::hypot(
        std::hypot(state.center_velocity.x, state.center_velocity.y),
        state.center_velocity.z);
    std::ostringstream line3;
    line3 << "xyz=[" << std::setprecision(2) << state.center_position.x << ','
          << state.center_position.y << ',' << state.center_position.z << "]m"
          << "  speed=" << speed << "m/s  yaw_rate="
          << state.yaw_velocity << "rad/s";
    putOutlined(image, line3.str(), cv::Point(12, 77), 0.48, state_color);
    std::ostringstream line4;
    line4 << "r1=" << debug.tracker_r1 << " r2=" << debug.tracker_r2
          << " dza=" << debug.tracker_dza << " prediction="
          << debug.control_target.prediction_time_s * 1000.0 << "ms";
    putOutlined(image, line4.str(), cv::Point(12, 103), 0.48, predicted_color);
  }
}

void writeVector(std::ostream& output, const Eigen::Vector3d& value) {
  output << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

template <typename Point>
void writePoint(std::ostream& output, const Point& value) {
  output << '[' << value.x << ',' << value.y << ',' << value.z << ']';
}

void writeQuaternion(std::ostream& output, const Eigen::Quaterniond& value) {
  const Eigen::Quaterniond normalized = value.normalized();
  output << '[' << normalized.w() << ',' << normalized.x() << ','
         << normalized.y() << ',' << normalized.z() << ']';
}

template <typename Quaternion>
void writeMessageQuaternion(std::ostream& output, const Quaternion& value) {
  output << '[' << value.w << ',' << value.x << ',' << value.y << ',' << value.z << ']';
}

void writeFrameJson(
    std::ostream& output, int64_t sequence, double time_s, int source_frame,
    const hfut::video::CameraCalibration& calibration,
    const Eigen::Matrix3d& camera_to_control,
    const std::vector<ArmorDetection>& detections,
    const std::vector<PoseEstimate>& poses,
    const hfut::pipeline::Pipeline::DebugSnapshot& debug,
    double detection_ms, double pose_ms, double processing_ms) {
  output << std::setprecision(10)
         << "{\"schema_version\":1,\"seq\":" << sequence
         << ",\"source_frame\":" << source_frame
         << ",\"time_s\":" << time_s
         << ",\"coordinate_frame\":\"control_x_forward_y_left_z_up\""
         << ",\"camera\":{\"width\":" << calibration.width
         << ",\"height\":" << calibration.height << ",\"k\":[";
  for (size_t i = 0; i < calibration.k.size(); ++i) {
    if (i) output << ',';
    output << calibration.k[i];
  }
  output << "],\"d\":[";
  for (size_t i = 0; i < calibration.d.size(); ++i) {
    if (i) output << ',';
    output << calibration.d[i];
  }
  output << "],\"position\":[0,0,0],\"orientation_wxyz\":";
  writeQuaternion(output, Eigen::Quaterniond(camera_to_control));
  output << "},\"detections\":[";
  for (size_t i = 0; i < detections.size(); ++i) {
    if (i) output << ',';
    const auto& detection = detections[i];
    const bool valid = i < poses.size() && poses[i].valid;
    output << "{\"number\":\"" << escapeJson(detection.publish_number)
           << "\",\"type\":\"" << escapeJson(detection.publish_type)
           << "\",\"confidence\":" << detection.confidence
           << ",\"track_id\":" << detection.track_id
           << ",\"bbox_xywh\":[" << detection.bbox.x << ',' << detection.bbox.y
           << ',' << detection.bbox.width << ',' << detection.bbox.height
           << "],\"keypoints\":[";
    for (size_t corner = 0; corner < 4; ++corner) {
      if (corner) output << ',';
      output << '[' << detection.keypoints[corner].x << ','
             << detection.keypoints[corner].y << ']';
    }
    output << "],\"pose_valid\":" << (valid ? "true" : "false");
    if (valid) {
      const auto& pose = poses[i];
      const Eigen::Vector3d position_control = camera_to_control * pose.translation;
      const Eigen::Quaterniond orientation_control(
          camera_to_control * pose.rotation.toRotationMatrix());
      output << ",\"position_camera\":";
      writeVector(output, pose.translation);
      output << ",\"position_control\":";
      writeVector(output, position_control);
      output << ",\"orientation_control_wxyz\":";
      writeQuaternion(output, orientation_control);
      output << ",\"yaw\":" << pose.yaw
             << ",\"pitch\":" << pose.pitch
             << ",\"roll\":" << pose.roll
             << ",\"reprojection_error_raw\":" << pose.reproj_error_raw
             << ",\"reprojection_error_refined\":" << pose.reproj_error_refined
             << ",\"quality_score\":" << pose.quality_score
             << ",\"ippe_yaw_ambiguity\":" << pose.ippe_yaw_ambiguity
             << ",\"pose_mode\":" << static_cast<int>(pose.mode);
    }
    output << '}';
  }
  output << "],\"state\":{\"valid\":"
         << (debug.selected_state_valid ? "true" : "false")
         << ",\"selected_id\":\"" << escapeJson(debug.selected_id)
         << "\",\"track_state\":" << debug.selected_track_state
         << ",\"tracked_count\":" << debug.num_tracked
         << ",\"tracker_update_committed\":"
         << (debug.tracker_update_committed ? "true" : "false")
         << ",\"tracker_update_reason\":\""
         << escapeJson(debug.tracker_decision_reason) << "\"";
  if (debug.selected_state_valid) {
    const auto& state = debug.selected_state;
    output << ",\"position\":";
    writePoint(output, state.center_position);
    output << ",\"velocity\":";
    writePoint(output, state.center_velocity);
    output << ",\"acceleration\":";
    writePoint(output, state.center_acceleration);
    output << ",\"orientation_wxyz\":";
    writeMessageQuaternion(output, state.center_pose.orientation);
    output << ",\"yaw\":" << state.yaw
           << ",\"yaw_rate\":" << state.yaw_velocity
           << ",\"yaw_acceleration\":" << state.yaw_acceleration
           << ",\"r1\":" << debug.tracker_r1
           << ",\"r2\":" << debug.tracker_r2
           << ",\"dza\":" << debug.tracker_dza
           << ",\"body_attitude_valid\":"
           << (debug.selected_attitude_valid ? "true" : "false")
           << ",\"body_pitch\":" << debug.selected_body_pitch_rad
           << ",\"body_roll\":" << debug.selected_body_roll_rad;
  }
  output << "},\"tracked_armors\":[";
  for (size_t i = 0; i < debug.tracked_armor_poses.size(); ++i) {
    if (i) output << ',';
    const auto& armor = debug.tracked_armor_poses[i];
    output << "{\"position\":";
    writeVector(output, armor.position);
    output << ",\"normal\":";
    writeVector(output, armor.normal);
    output << ",\"width_axis\":";
    writeVector(output, armor.width_axis);
    output << ",\"height_axis\":";
    writeVector(output, armor.height_axis);
    output << '}';
  }
  const auto& prediction = debug.control_target;
  output << "],\"prediction\":{\"valid\":"
         << (prediction.valid ? "true" : "false")
         << ",\"time_s\":" << prediction.prediction_time_s;
  if (prediction.valid) {
    output << ",\"current_center\":";
    writeVector(output, prediction.current_center);
    output << ",\"center\":";
    writeVector(output, prediction.predicted_center);
    output << ",\"linear_velocity\":";
    writeVector(output, prediction.linear_velocity);
    output << ",\"yaw_rate\":" << prediction.yaw_velocity
           << ",\"armors\":[";
    for (size_t i = 0; i < prediction.predicted_armor_positions.size(); ++i) {
      if (i) output << ',';
      writeVector(output, prediction.predicted_armor_positions[i]);
    }
    output << ']';
  }
  output << "},\"command\":{\"mode\":" << static_cast<int>(debug.command.mode)
         << ",\"yaw_deg\":" << debug.command.yaw
         << ",\"pitch_deg\":" << debug.command.pitch
         << ",\"yaw_velocity_deg_s\":" << debug.command.yaw_v
         << ",\"pitch_velocity_deg_s\":" << debug.command.pitch_v
         << ",\"distance_m\":" << debug.command.distance
         << ",\"bullet_speed_mps\":" << debug.bullet_speed
         << "},\"timing\":{\"detection_ms\":" << detection_ms
         << ",\"pose_ms\":" << pose_ms
         << ",\"processing_ms\":" << processing_ms << "}}\n";
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double index = std::clamp(fraction, 0.0, 1.0) * (values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(index));
  const size_t upper = static_cast<size_t>(std::ceil(index));
  const double weight = index - lower;
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

double mean(const std::vector<double>& values) {
  if (values.empty()) return 0.0;
  return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

struct RunStats {
  int source_frames{0};
  int processed_frames{0};
  int detection_frames{0};
  int total_detections{0};
  int valid_poses{0};
  int selected_state_frames{0};
  int tracking_frames{0};
  int prediction_frames{0};
  int committed_update_frames{0};
  int track_state_transitions{0};
  int longest_detection_gap{0};
  int current_detection_gap{0};
  int previous_track_state{-99};
  std::unordered_map<int, Eigen::Vector3d> previous_pose_by_track;
  std::vector<double> confidences;
  std::vector<double> reprojection_errors;
  std::vector<double> pose_jumps;
  std::vector<double> processing_times;
  std::vector<double> detection_times;
  std::vector<double> pose_times;
};

void updateStats(
    RunStats& stats, const std::vector<ArmorDetection>& detections,
    const std::vector<PoseEstimate>& poses,
    const hfut::pipeline::Pipeline::DebugSnapshot& debug,
    double detection_ms, double pose_ms, double processing_ms) {
  ++stats.processed_frames;
  stats.total_detections += static_cast<int>(detections.size());
  if (!detections.empty()) {
    ++stats.detection_frames;
    stats.current_detection_gap = 0;
  } else {
    ++stats.current_detection_gap;
    stats.longest_detection_gap =
        std::max(stats.longest_detection_gap, stats.current_detection_gap);
  }
  for (const auto& detection : detections) stats.confidences.push_back(detection.confidence);
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i];
    if (!pose.valid) continue;
    ++stats.valid_poses;
    stats.reprojection_errors.push_back(pose.reproj_error_refined);
    const int track_id = i < detections.size() ? detections[i].track_id : -1;
    if (track_id >= 0) {
      const auto previous = stats.previous_pose_by_track.find(track_id);
      if (previous != stats.previous_pose_by_track.end()) {
        stats.pose_jumps.push_back((pose.translation - previous->second).norm());
      }
      stats.previous_pose_by_track[track_id] = pose.translation;
    }
  }
  if (debug.selected_state_valid) ++stats.selected_state_frames;
  if (debug.selected_track_state == rm_interfaces::msg::TrackedRobot::TRACKING) {
    ++stats.tracking_frames;
  }
  if (debug.control_target.valid) ++stats.prediction_frames;
  if (debug.tracker_update_committed) ++stats.committed_update_frames;
  if (stats.previous_track_state != -99 &&
      stats.previous_track_state != debug.selected_track_state) {
    ++stats.track_state_transitions;
  }
  stats.previous_track_state = debug.selected_track_state;
  stats.detection_times.push_back(detection_ms);
  stats.pose_times.push_back(pose_ms);
  stats.processing_times.push_back(processing_ms);
}

void writeSummary(
    const std::string& path, const Options& options, const RunStats& stats,
    const hfut::video::CameraCalibration& source_calibration,
    const hfut::video::CameraCalibration& effective_calibration,
    double fps, int declared_frame_count, const std::string& backend_name) {
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output) throw std::runtime_error("failed to open summary output: " + path);
  auto ratio = [&stats](int count) {
    return stats.processed_frames > 0
        ? static_cast<double>(count) / stats.processed_frames : 0.0;
  };
  output << std::setprecision(10)
         << "{\n  \"schema_version\": 1,\n"
         << "  \"video\": {\"path\": \"" << escapeJson(options.video_path)
         << "\", \"fps\": " << fps << ", \"declared_frames\": "
         << declared_frame_count << "},\n"
         << "  \"calibration\": {\"path\": \""
         << escapeJson(options.camera_info_path) << "\", \"mode\": \""
         << escapeJson(options.calibration_mode) << "\", \"source_size\": ["
         << source_calibration.width << ',' << source_calibration.height
         << "], \"effective_size\": [" << effective_calibration.width << ','
         << effective_calibration.height << "], \"effective_k\": [";
  for (size_t i = 0; i < effective_calibration.k.size(); ++i) {
    if (i) output << ',';
    output << effective_calibration.k[i];
  }
  output << "], \"scale\": [" << effective_calibration.scale_x << ','
         << effective_calibration.scale_y << "], \"offset\": ["
         << effective_calibration.offset_x << ',' << effective_calibration.offset_y
         << "]},\n  \"detector_backend\": \"" << escapeJson(backend_name) << "\",\n"
         << "  \"absolute_accuracy_available\": false,\n"
         << "  \"accuracy_note\": \"The video has no synchronized ground truth; metrics measure detection, reprojection, continuity, and tracker response only.\",\n"
         << "  \"counts\": {\"source_frames_read\": " << stats.source_frames
         << ", \"processed_frames\": " << stats.processed_frames
         << ", \"detection_frames\": " << stats.detection_frames
         << ", \"detections\": " << stats.total_detections
         << ", \"valid_poses\": " << stats.valid_poses
         << ", \"selected_state_frames\": " << stats.selected_state_frames
         << ", \"tracking_frames\": " << stats.tracking_frames
         << ", \"prediction_frames\": " << stats.prediction_frames
         << ", \"committed_update_frames\": " << stats.committed_update_frames
         << ", \"track_state_transitions\": " << stats.track_state_transitions
         << ", \"longest_detection_gap_frames\": " << stats.longest_detection_gap
         << "},\n"
         << "  \"ratios\": {\"detection_frame\": " << ratio(stats.detection_frames)
         << ", \"pose_per_detection\": "
         << (stats.total_detections > 0
                ? static_cast<double>(stats.valid_poses) / stats.total_detections : 0.0)
         << ", \"selected_state_frame\": " << ratio(stats.selected_state_frames)
         << ", \"tracking_frame\": " << ratio(stats.tracking_frames)
         << ", \"prediction_frame\": " << ratio(stats.prediction_frames) << "},\n"
         << "  \"metrics\": {\n"
         << "    \"confidence_mean\": " << mean(stats.confidences)
         << ", \"confidence_p10\": " << percentile(stats.confidences, 0.10) << ",\n"
         << "    \"reprojection_error_px_mean\": " << mean(stats.reprojection_errors)
         << ", \"reprojection_error_px_p95\": "
         << percentile(stats.reprojection_errors, 0.95) << ",\n"
         << "    \"pose_jump_m_p50\": " << percentile(stats.pose_jumps, 0.50)
         << ", \"pose_jump_m_p95\": " << percentile(stats.pose_jumps, 0.95)
         << ", \"pose_jump_m_max\": " << percentile(stats.pose_jumps, 1.0) << ",\n"
         << "    \"detection_ms_mean\": " << mean(stats.detection_times)
         << ", \"pose_ms_mean\": " << mean(stats.pose_times)
         << ", \"processing_ms_mean\": " << mean(stats.processing_times)
         << ", \"processing_ms_p95\": " << percentile(stats.processing_times, 0.95)
         << "\n  }\n}\n";
}

cv::VideoWriter openVideoWriter(
    const std::string& path, double fps, const cv::Size& size) {
  cv::VideoWriter writer;
  if (path.empty()) return writer;
  const std::string lower = [&path]() {
    std::string value = path;
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
  }();
  const int preferred_fourcc =
      lower.size() >= 4 && lower.substr(lower.size() - 4) == ".mp4"
          ? cv::VideoWriter::fourcc('m', 'p', '4', 'v')
          : cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  writer.open(path, preferred_fourcc, fps, size, true);
  if (!writer.isOpened()) {
    throw std::runtime_error("failed to open annotated video output: " + path);
  }
  return writer;
}

int run(const Options& options) {
  const auto mode = hfut::video::parseCalibrationMode(options.calibration_mode);
  cv::VideoCapture capture(options.video_path);
  if (!capture.isOpened()) {
    throw std::runtime_error("failed to open video: " + options.video_path);
  }
  const int width = cvRound(capture.get(cv::CAP_PROP_FRAME_WIDTH));
  const int height = cvRound(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
  const double source_fps = capture.get(cv::CAP_PROP_FPS);
  const double fps = source_fps > 0.0 && std::isfinite(source_fps) ? source_fps : 30.0;
  const int declared_frames = cvRound(capture.get(cv::CAP_PROP_FRAME_COUNT));
  const auto source_calibration = loadCalibration(options.camera_info_path);
  const auto calibration = hfut::video::adaptCalibration(
      source_calibration, width, height, mode);
  const sensor_msgs::msg::CameraInfo camera_info = toCameraInfo(calibration);

  std::printf(
      "[video_test] video=%s %dx%d %.3f FPS frames=%d\n",
      options.video_path.c_str(), width, height, fps, declared_frames);
  std::printf(
      "[video_test] calibration=%s source=%dx%d mode=%s scale=[%.6f,%.6f] "
      "offset=[%.3f,%.3f] effective K=[%.3f,%.3f,%.3f,%.3f]\n",
      options.camera_info_path.c_str(), source_calibration.width,
      source_calibration.height, hfut::video::calibrationModeName(mode),
      calibration.scale_x, calibration.scale_y, calibration.offset_x,
      calibration.offset_y, calibration.k[0], calibration.k[4],
      calibration.k[2], calibration.k[5]);

  const std::string detector_config_path = options.config_dir + "/detector.yaml";
  const std::string tracker_config_path = options.config_dir + "/tracker.yaml";
  const std::string controller_config_path = options.config_dir + "/controller.yaml";
  const std::string master_config_path = options.config_dir + "/gimbal_pipeline.yaml";

  DetectorConfig detector_config =
      hfut::detector::loadDetectorConfigFile(detector_config_path);
  ArmorDetectorNN detector(detector_config);
  detector.setTargetColor(parseEnemyColor(options.enemy_color));
  if (!detector.initialize()) throw std::runtime_error("detector initialization failed");
  const std::string backend_name = detector.backendInfo().backend_name;
  ArmorPoseEstimatorAdapter pose_estimator(detector_config.pose);
  if (detector_config.pose.refiner.mode == "single_yaw") {
    pose_estimator.setRefiner(std::make_shared<SingleYawRefiner>(
        detector_config.pose.single_yaw, detector_config.pose.gate));
  }

  hfut::pipeline::Pipeline pipeline(
      {tracker_config_path, controller_config_path, master_config_path});
  pipeline.updateFov(
      std::atan(static_cast<double>(width) / (2.0 * calibration.k[0])),
      std::atan(static_cast<double>(height) / (2.0 * calibration.k[4])));

  std::ofstream diagnostics(options.diagnostics_path, std::ios::out | std::ios::trunc);
  if (!diagnostics) {
    throw std::runtime_error("failed to open diagnostics output: " + options.diagnostics_path);
  }
  const double output_fps = fps / options.frame_step;
  cv::VideoWriter writer = openVideoWriter(
      options.overlay_path, output_fps, cv::Size(width, height));
  if (options.display) {
    cv::namedWindow("hfut video test", cv::WINDOW_NORMAL);
    cv::resizeWindow("hfut video test", 960, 720);
  }

  const Eigen::Matrix3d camera_to_control = opticalToControl();
  DebugDrawer drawer;
  RunStats stats;
  cv::Mat frame;
  int source_frame = -1;
  int64_t sequence = 0;
  const auto playback_start = std::chrono::steady_clock::now();
  while (options.max_frames < 0 || sequence < options.max_frames) {
    if (!capture.read(frame)) break;
    ++source_frame;
    ++stats.source_frames;
    if (source_frame < options.start_frame) continue;
    if ((source_frame - options.start_frame) % options.frame_step != 0) continue;
    const double time_s = static_cast<double>(source_frame) / fps;
    const auto processing_start = std::chrono::steady_clock::now();

    std_msgs::msg::Header header;
    const int64_t time_ns = static_cast<int64_t>(std::llround(time_s * 1e9));
    header.stamp.sec = static_cast<int32_t>(time_ns / 1000000000LL);
    header.stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000LL);
    const auto detection_start = std::chrono::steady_clock::now();
    const auto detected = detector.detectBatch({frame}, {header});
    const double detection_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - detection_start).count();
    std::vector<ArmorDetection> detections;
    if (!detected.empty()) detections = detected.front().detections;

    const auto pose_start = std::chrono::steady_clock::now();
    const std::vector<PoseEstimate> poses =
        pose_estimator.estimateBatch(detections, camera_info, camera_to_control);
    const double pose_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - pose_start).count();
    const auto armors = buildValidArmors(detections, poses);
    pipeline.updateTracking(armors, camera_to_control, Eigen::Vector3d::Zero(), time_s);
    (void)pipeline.computeCommand(0.0, 0.0, time_s);
    const double processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - processing_start).count();
    const auto& debug = pipeline.lastDebug();

    writeFrameJson(diagnostics, sequence, time_s, source_frame, calibration,
                   camera_to_control, detections, poses, debug,
                   detection_ms, pose_ms, processing_ms);
    updateStats(stats, detections, poses, debug,
                detection_ms, pose_ms, processing_ms);

    if (writer.isOpened() || options.display) {
      cv::Mat annotated = frame.clone();
      drawer.drawDetections(annotated, detections, true);
      drawPipelineOverlay(annotated, detections, poses, debug, camera_to_control,
                          calibration, time_s, processing_ms);
      if (writer.isOpened()) writer.write(annotated);
      if (options.display) {
        cv::imshow("hfut video test", annotated);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
      }
    }

    ++sequence;
    if (sequence % 100 == 0) {
      std::printf(
          "[video_test] processed=%lld source=%d detections=%d poses=%d "
          "tracking=%d prediction=%d\n",
          static_cast<long long>(sequence), source_frame, stats.total_detections,
          stats.valid_poses, stats.tracking_frames, stats.prediction_frames);
      std::fflush(stdout);
    }
    if (options.realtime) {
      const double relative_source_time =
          static_cast<double>(source_frame - options.start_frame) / fps;
      std::this_thread::sleep_until(
          playback_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(relative_source_time)));
    }
  }
  diagnostics.flush();
  writer.release();
  if (options.display) cv::destroyAllWindows();
  writeSummary(options.summary_path, options, stats, source_calibration,
               calibration, fps, declared_frames, backend_name);

  std::printf(
      "[video_test] complete: processed=%d detections=%d valid_poses=%d "
      "tracking_frames=%d prediction_frames=%d\n",
      stats.processed_frames, stats.total_detections, stats.valid_poses,
      stats.tracking_frames, stats.prediction_frames);
  std::printf("[video_test] diagnostics=%s\n", options.diagnostics_path.c_str());
  std::printf("[video_test] summary=%s\n", options.summary_path.c_str());
  if (!options.overlay_path.empty()) {
    std::printf("[video_test] overlay=%s\n", options.overlay_path.c_str());
  }
  return stats.processed_frames > 0 ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  try { FYT_REGISTER_LOGGER("armor_detector", "/tmp/hfut_auto_aim_log", INFO); } catch (...) {}
  try { FYT_REGISTER_LOGGER("armor_detector_nn", "/tmp/hfut_auto_aim_log", INFO); } catch (...) {}
  try {
    const Options options = parseOptions(argc, argv);
    if (options.help) {
      printUsage();
      return 0;
    }
    return run(options);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[video_test] ERROR: %s\n", error.what());
    return 1;
  }
}
