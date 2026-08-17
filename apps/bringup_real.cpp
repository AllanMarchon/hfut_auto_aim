// 真实车运行入口：
// OpenCV 相机源 + infantry32 串口传输 + 现有 detector/PnP/
// tracker/controller 链路。默认关闭开火输出。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <yaml-cpp/yaml.h>

#include "hfut_auto_aim/camera_frame.hpp"
#include "hfut_auto_aim/gimbal_command.hpp"
#include "hfut_auto_aim/video_calibration.hpp"
#include "io/camera/camera_source.hpp"
#ifdef HFUT_HAS_HIK_CAMERA
#include "io/camera/hik_camera_source.hpp"
#endif
#ifdef HFUT_HAS_MINDVISION_CAMERA
#include "io/camera/mindvision_camera_source.hpp"
#endif
#include "io/camera/opencv_camera_source.hpp"
#include "io/serial/infantry_serial.hpp"
#include "io/web/debug_mjpeg_server.hpp"

#include "config_loader.hpp"
#include "armor_detector_nn/core/armor_detector_nn.hpp"
#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"
#include "armor_detector_nn/core/pose_refine/pose_refiner.hpp"
#include "armor_detector_nn/debug/debug_drawer.hpp"
#include "pipeline.hpp"
#include "rm_utils/logger/log.hpp"

#include <rm_interfaces/msg/armors.hpp>

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

struct Options {
  std::string config_dir{"configs"};
  std::string hardware_config{"configs/hardware.yaml"};
  std::string camera_backend{"opencv"};
  std::string camera_source;
  std::string camera_sn;
  int camera_index{0};
  int camera_width{0};
  int camera_height{0};
  double camera_fps{0.0};
  double camera_exposure_time_us{0.0};
  double camera_gain{0.0};
  int camera_analog_gain{-1};
  int camera_frame_speed{-1};
  bool camera_flip_image{false};
  std::string camera_info_path;
  std::string calibration_mode{"strict"};
  hfut::CameraToBarrelExtrinsics camera_to_barrel;

  std::string serial_port{"/dev/ttyACM0"};
  int serial_baudrate{115200};
  std::string serial_protocol{"infantry"};
  std::string serial_tx_protocol{"infantry"};
  std::string serial_rx_protocol{"infantry"};
  std::string infantry32_tail_fields{"duplicate_velocity"};
  bool command_angles_in_degrees{true};
  bool feedback_angles_in_degrees{true};
  int serial_read_timeout_ms{2};
  int feedback_timeout_ms{100};
  bool require_feedback{true};

  std::string enemy_color{"red"};
  double bullet_speed{23.0};
  std::string controller_strategy;
  bool dry_run{false};
  bool enable_fire{false};
  bool display{false};
  bool web_view{false};
  std::string web_host{"0.0.0.0"};
  int web_port{8080};
  int web_jpeg_quality{80};
  int web_max_width{960};
  int web_frame_step{2};
  int max_frames{-1};
};

std::string optionValue(int argc, char** argv, int& index,
                        const std::string& arg, const std::string& name) {
  const std::string prefix = name + "=";
  if (arg.compare(0, prefix.size(), prefix) == 0) {
    return arg.substr(prefix.size());
  }
  if (arg == name && index + 1 < argc) return argv[++index];
  return {};
}

bool parseBool(const YAML::Node& node, bool fallback) {
  return node ? node.as<bool>() : fallback;
}

int parseInt(const YAML::Node& node, int fallback) {
  return node ? node.as<int>() : fallback;
}

double parseDouble(const YAML::Node& node, double fallback) {
  return node ? node.as<double>() : fallback;
}

std::string parseString(const YAML::Node& node, const std::string& fallback) {
  return node ? node.as<std::string>() : fallback;
}

Eigen::Vector3d readVector3(const YAML::Node& node,
                            const Eigen::Vector3d& fallback,
                            const std::string& name) {
  if (!node) return fallback;
  if (!node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(name + " must be a three-element sequence");
  }
  Eigen::Vector3d value(node[0].as<double>(), node[1].as<double>(),
                        node[2].as<double>());
  if (!value.allFinite()) {
    throw std::invalid_argument(name + " values must be finite");
  }
  return value;
}

void loadHardwareConfig(Options& options) {
  const YAML::Node file_root = YAML::LoadFile(options.hardware_config);
  const YAML::Node root = file_root["hardware"] ? file_root["hardware"]
                          : (file_root["real_vehicle"] ? file_root["real_vehicle"]
                                                        : file_root);

  const YAML::Node camera = root["camera"];
  if (camera) {
    options.camera_backend = parseString(camera["backend"], options.camera_backend);
    options.camera_source = parseString(camera["source"], options.camera_source);
    options.camera_sn = parseString(camera["camera_sn"], options.camera_sn);
    options.camera_index = parseInt(camera["device_index"], options.camera_index);
    options.camera_width = parseInt(camera["width"], options.camera_width);
    options.camera_height = parseInt(camera["height"], options.camera_height);
    options.camera_fps = parseDouble(camera["fps"], options.camera_fps);
    options.camera_exposure_time_us =
        parseDouble(camera["exposure_time_us"], options.camera_exposure_time_us);
    options.camera_gain = parseDouble(camera["gain"], options.camera_gain);
    options.camera_analog_gain =
        parseInt(camera["analog_gain"], options.camera_analog_gain);
    options.camera_frame_speed =
        parseInt(camera["frame_speed"], options.camera_frame_speed);
    options.camera_flip_image =
        parseBool(camera["flip_image"], options.camera_flip_image);
    options.camera_info_path =
        parseString(camera["camera_info"], options.camera_info_path);
    options.calibration_mode =
        parseString(camera["calibration_mode"], options.calibration_mode);
    const YAML::Node extrinsics = camera["camera_to_barrel"];
    if (extrinsics) {
      options.camera_to_barrel.xyz = readVector3(
          extrinsics["xyz"], options.camera_to_barrel.xyz,
          "camera.camera_to_barrel.xyz");
      options.camera_to_barrel.rpy = readVector3(
          extrinsics["rpy"], options.camera_to_barrel.rpy,
          "camera.camera_to_barrel.rpy");
    }
  }

  const YAML::Node serial = root["serial"];
  if (serial) {
    options.serial_port = parseString(serial["port"], options.serial_port);
    options.serial_baudrate = parseInt(serial["baudrate"], options.serial_baudrate);
    options.serial_protocol = parseString(serial["protocol"], options.serial_protocol);
    options.serial_tx_protocol = options.serial_protocol;
    options.serial_rx_protocol = options.serial_protocol;
    options.serial_tx_protocol = parseString(serial["tx_protocol"], options.serial_tx_protocol);
    options.serial_rx_protocol = parseString(serial["rx_protocol"], options.serial_rx_protocol);
    options.infantry32_tail_fields = parseString(
        serial["infantry32_tail_fields"], options.infantry32_tail_fields);
    options.command_angles_in_degrees = parseBool(
        serial["command_angles_in_degrees"], options.command_angles_in_degrees);
    options.feedback_angles_in_degrees = parseBool(
        serial["feedback_angles_in_degrees"], options.feedback_angles_in_degrees);
    options.serial_read_timeout_ms =
        parseInt(serial["read_timeout_ms"], options.serial_read_timeout_ms);
    options.feedback_timeout_ms =
        parseInt(serial["feedback_timeout_ms"], options.feedback_timeout_ms);
    options.require_feedback =
        parseBool(serial["require_feedback"], options.require_feedback);
  }

  const YAML::Node detector = root["detector"];
  if (detector) {
    options.enemy_color = parseString(detector["enemy_color"], options.enemy_color);
  }

  const YAML::Node controller = root["controller"];
  if (controller) {
    options.bullet_speed = parseDouble(controller["bullet_speed"], options.bullet_speed);
    options.controller_strategy =
        parseString(controller["strategy"], options.controller_strategy);
  }

  const YAML::Node safety = root["safety"];
  if (safety) {
    options.dry_run = parseBool(safety["dry_run"], options.dry_run);
    options.enable_fire = parseBool(safety["enable_fire"], options.enable_fire);
  }
}

Options parseOptions(int argc, char** argv) {
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (auto value = optionValue(argc, argv, i, arg, "--hardware-config"); !value.empty()) {
      options.hardware_config = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--real-config"); !value.empty()) {
      options.hardware_config = value;
    }
  }

  loadHardwareConfig(options);

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      std::printf(
          "Usage: bringup_real [options]\n"
          "  --hardware-config PATH   hardware YAML (default configs/hardware.yaml)\n"
          "  --real-config PATH       legacy alias for --hardware-config\n"
          "  --config-dir PATH        detector/tracker/controller config directory\n"
          "  --camera-backend NAME    opencv | hik | mindvision\n"
          "  --camera-source PATH     OpenCV source path/URL/video file\n"
          "  --camera-sn SN           industrial camera serial number\n"
          "  --camera-index N         OpenCV camera index when source is empty\n"
          "  --camera-info PATH       ROS camera_info YAML, required\n"
          "  --exposure-time-us US    industrial camera exposure override\n"
          "  --gain VALUE             Hik/OpenCV gain override\n"
          "  --flip-image             flip industrial camera image 180 degrees\n"
          "  --serial-port PATH       serial device path\n"
          "  --baudrate N             serial baudrate\n"
          "  --serial-protocol NAME   set both TX/RX protocol\n"
          "  --serial-tx-protocol NAME  infantry | infantry_16 | infantry_32\n"
          "  --serial-rx-protocol NAME  infantry | infantry_16 | infantry_32\n"
          "  --infantry32-tail-fields acceleration | duplicate_velocity\n"
          "  --enemy-color COLOR      red | blue | white\n"
          "  --bullet-speed MPS       controller bullet speed override\n"
          "  --strategy NAME          controller strategy override\n"
          "  --dry-run                do not open/send serial\n"
          "  --enable-fire            allow fire_advice to reach serial packet\n"
          "  --display                show detector overlay\n"
          "  --web-view               serve detector overlay at http://HOST:PORT/\n"
          "  --web-host ADDR          web bind address (default 0.0.0.0)\n"
          "  --web-port N             web port (default 8080)\n"
          "  --web-jpeg-quality N     JPEG quality 30-95 (default 80)\n"
          "  --web-max-width PX       stream downscale width (default 960)\n"
          "  --web-frame-step N       publish every N processed frames (default 2)\n"
          "  --max-frames N           stop after N processed frames\n");
      std::exit(0);
    } else if (arg == "--dry-run") {
      options.dry_run = true;
    } else if (arg == "--enable-fire") {
      options.enable_fire = true;
    } else if (arg == "--display") {
      options.display = true;
    } else if (arg == "--web-view") {
      options.web_view = true;
    } else if (arg == "--flip-image") {
      options.camera_flip_image = true;
    } else if (arg == "--hardware-config" || arg == "--real-config") {
      ++i;
    } else if (arg.rfind("--hardware-config=", 0) == 0 ||
               arg.rfind("--real-config=", 0) == 0) {
      continue;
    } else if (auto value = optionValue(argc, argv, i, arg, "--config-dir"); !value.empty()) {
      options.config_dir = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-backend"); !value.empty()) {
      options.camera_backend = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-source"); !value.empty()) {
      options.camera_source = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-sn"); !value.empty()) {
      options.camera_sn = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-index"); !value.empty()) {
      options.camera_index = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-info"); !value.empty()) {
      options.camera_info_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--exposure-time-us"); !value.empty()) {
      options.camera_exposure_time_us = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--gain"); !value.empty()) {
      options.camera_gain = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-port"); !value.empty()) {
      options.serial_port = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--baudrate"); !value.empty()) {
      options.serial_baudrate = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-protocol"); !value.empty()) {
      options.serial_protocol = value;
      options.serial_tx_protocol = value;
      options.serial_rx_protocol = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-tx-protocol"); !value.empty()) {
      options.serial_tx_protocol = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-rx-protocol"); !value.empty()) {
      options.serial_rx_protocol = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--infantry32-tail-fields"); !value.empty()) {
      options.infantry32_tail_fields = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--enemy-color"); !value.empty()) {
      options.enemy_color = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--bullet-speed"); !value.empty()) {
      options.bullet_speed = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--strategy"); !value.empty()) {
      options.controller_strategy = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-host"); !value.empty()) {
      options.web_host = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-port"); !value.empty()) {
      options.web_port = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-jpeg-quality"); !value.empty()) {
      options.web_jpeg_quality = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-max-width"); !value.empty()) {
      options.web_max_width = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-frame-step"); !value.empty()) {
      options.web_frame_step = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--max-frames"); !value.empty()) {
      options.max_frames = std::stoi(value);
    } else {
      throw std::invalid_argument("unknown or incomplete option: " + arg);
    }
  }

  if (options.camera_info_path.empty()) {
    throw std::invalid_argument(
        "camera_info is required; set hardware.camera.camera_info or --camera-info");
  }
  if (options.camera_backend != "opencv" && options.camera_backend != "hik" &&
      options.camera_backend != "mindvision") {
    throw std::invalid_argument(
        "camera backend must be opencv, hik, or mindvision, got: " +
        options.camera_backend);
  }
  if (options.serial_read_timeout_ms < 0) {
    throw std::invalid_argument("serial read timeout must be >= 0");
  }
  if (options.feedback_timeout_ms <= 0) {
    throw std::invalid_argument("feedback timeout must be > 0");
  }
  if (options.web_port <= 0 || options.web_port > 65535) {
    throw std::invalid_argument("web port must be in 1..65535");
  }
  if (options.web_jpeg_quality < 30 || options.web_jpeg_quality > 95) {
    throw std::invalid_argument("web JPEG quality must be in 30..95");
  }
  if (options.web_max_width <= 0) {
    throw std::invalid_argument("web max width must be > 0");
  }
  if (options.web_frame_step <= 0) {
    throw std::invalid_argument("web frame step must be > 0");
  }
  hfut::io::InfantryPacketLayout tx_layout;
  if (!hfut::io::parseInfantryPacketLayout(options.serial_tx_protocol, tx_layout)) {
    throw std::invalid_argument(
        "serial tx_protocol must be infantry, infantry_16, or infantry_32, got: " +
        options.serial_tx_protocol);
  }
  hfut::io::InfantryPacketLayout rx_layout;
  if (!hfut::io::parseInfantryPacketLayout(options.serial_rx_protocol, rx_layout)) {
    throw std::invalid_argument(
        "serial rx_protocol must be infantry, infantry_16, or infantry_32, got: " +
        options.serial_rx_protocol);
  }
  hfut::io::Infantry32TailFields fields;
  if (!hfut::io::parseInfantry32TailFields(options.infantry32_tail_fields, fields)) {
    throw std::invalid_argument(
        "infantry32_tail_fields must be acceleration or duplicate_velocity, got: " +
        options.infantry32_tail_fields);
  }
  return options;
}

hfut::video::CameraCalibration loadCalibration(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  hfut::video::CameraCalibration calibration;
  if (!root["image_width"] || !root["image_height"] ||
      !root["camera_matrix"] || !root["camera_matrix"]["data"]) {
    throw std::invalid_argument(
        "camera_info YAML is missing image dimensions or camera_matrix.data");
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
  return calibration;
}

hfut::CameraIntrinsics toIntrinsics(
    const hfut::video::CameraCalibration& calibration) {
  hfut::CameraIntrinsics intrinsics;
  intrinsics.width = calibration.width;
  intrinsics.height = calibration.height;
  intrinsics.fx = calibration.k[0];
  intrinsics.fy = calibration.k[4];
  intrinsics.cx = calibration.k[2];
  intrinsics.cy = calibration.k[5];
  for (size_t i = 0; i < 5 && i < calibration.d.size(); ++i) {
    intrinsics.distortion[i] = calibration.d[i];
  }
  return intrinsics;
}

sensor_msgs::msg::CameraInfo toCameraInfo(
    const hfut::CameraIntrinsics& intrinsics) {
  sensor_msgs::msg::CameraInfo info;
  info.width = static_cast<uint32_t>(intrinsics.width);
  info.height = static_cast<uint32_t>(intrinsics.height);
  info.k = {intrinsics.fx, 0.0, intrinsics.cx,
            0.0, intrinsics.fy, intrinsics.cy,
            0.0, 0.0, 1.0};
  info.d.assign(intrinsics.distortion, intrinsics.distortion + 5);
  return info;
}

fyt::EnemyColor parseEnemyColor(const std::string& value) {
  if (value == "red") return fyt::EnemyColor::RED;
  if (value == "blue") return fyt::EnemyColor::BLUE;
  if (value == "white") return fyt::EnemyColor::WHITE;
  throw std::invalid_argument("enemy color must be red, blue, or white, got: " + value);
}

std::unique_ptr<hfut::io::CameraSource> createCameraSource(
    const Options& options, const hfut::CameraIntrinsics& intrinsics) {
  if (options.camera_backend == "opencv") {
    hfut::io::OpenCvCameraSourceConfig config;
    config.source = options.camera_source;
    config.device_index = options.camera_index;
    config.width = options.camera_width;
    config.height = options.camera_height;
    config.fps = options.camera_fps;
    config.gain = options.camera_gain;
    config.set_gain = options.camera_gain > 0.0;
    config.intrinsics = intrinsics;
    return std::make_unique<hfut::io::OpenCvCameraSource>(config);
  }

  if (options.camera_backend == "hik") {
#ifdef HFUT_HAS_HIK_CAMERA
    hfut::io::HikCameraSourceConfig config;
    config.camera_sn = options.camera_sn;
    config.width = options.camera_width;
    config.height = options.camera_height;
    config.fps = options.camera_fps;
    config.exposure_time_us = options.camera_exposure_time_us;
    config.gain = options.camera_gain;
    config.flip_image = options.camera_flip_image;
    config.intrinsics = intrinsics;
    return std::make_unique<hfut::io::HikCameraSource>(config);
#else
    throw std::runtime_error(
        "camera.backend=hik requires rebuilding with HFUT_ENABLE_HIK_CAMERA=ON");
#endif
  }

  if (options.camera_backend == "mindvision") {
#ifdef HFUT_HAS_MINDVISION_CAMERA
    hfut::io::MindvisionCameraSourceConfig config;
    config.camera_sn = options.camera_sn;
    config.width = options.camera_width;
    config.height = options.camera_height;
    config.fps = options.camera_fps;
    config.exposure_time_us = options.camera_exposure_time_us;
    config.analog_gain = options.camera_analog_gain;
    config.frame_speed = options.camera_frame_speed;
    config.flip_image = options.camera_flip_image;
    config.intrinsics = intrinsics;
    return std::make_unique<hfut::io::MindvisionCameraSource>(config);
#else
    throw std::runtime_error(
        "camera.backend=mindvision requires rebuilding with "
        "HFUT_ENABLE_MINDVISION_CAMERA=ON");
#endif
  }

  throw std::invalid_argument("unsupported camera backend: " + options.camera_backend);
}

hfut::io::InfantrySerialConfig makeSerialConfig(const Options& options) {
  hfut::io::InfantrySerialConfig config;
  config.port = options.serial_port;
  config.baudrate = options.serial_baudrate;
  if (!hfut::io::parseInfantryPacketLayout(options.serial_tx_protocol, config.tx_layout)) {
    throw std::invalid_argument("unsupported serial tx_protocol: " + options.serial_tx_protocol);
  }
  if (!hfut::io::parseInfantryPacketLayout(options.serial_rx_protocol, config.rx_layout)) {
    throw std::invalid_argument("unsupported serial rx_protocol: " + options.serial_rx_protocol);
  }
  if (!hfut::io::parseInfantry32TailFields(
          options.infantry32_tail_fields, config.tail_fields)) {
    throw std::invalid_argument(
        "unsupported infantry32_tail_fields: " + options.infantry32_tail_fields);
  }
  config.command_angles_in_degrees = options.command_angles_in_degrees;
  config.feedback_angles_in_degrees = options.feedback_angles_in_degrees;
  config.read_timeout_ms = options.serial_read_timeout_ms;
  config.allow_fire = options.enable_fire;
  return config;
}

std_msgs::msg::Header makeHeader(double time_s) {
  std_msgs::msg::Header header;
  const auto time_ns = static_cast<int64_t>(std::llround(time_s * 1.0e9));
  header.stamp.sec = static_cast<int32_t>(time_ns / 1000000000LL);
  header.stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000LL);
  header.frame_id = "camera_optical_frame";
  return header;
}

rm_interfaces::msg::Armors buildValidArmors(
    const std::vector<fyt::auto_aim::ArmorDetection>& detections,
    const std::vector<fyt::auto_aim::PoseEstimate>& poses) {
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
        for (int col = 0; col < 4; ++col) {
          armor.pose_covariance_xyz_yaw[row * 4 + col] =
              pose.covariance_xyz_yaw(row, col);
        }
      }
    }
    message.armors.push_back(std::move(armor));
  }
  return message;
}

const char* trackStateName(int state) {
  using Robot = rm_interfaces::msg::TrackedRobot;
  switch (state) {
    case Robot::DETECTING:
      return "DETECTING";
    case Robot::TRACKING:
      return "TRACKING";
    case Robot::TEMP_LOST:
      return "TEMP_LOST";
    default:
      return "none";
  }
}

std::string firstArmorSummary(const rm_interfaces::msg::Armors& armors) {
  if (armors.armors.empty()) return "none";

  const auto& armor = armors.armors.front();
  const double x = armor.pose.position.x;
  const double y = armor.pose.position.y;
  const double z = armor.pose.position.z;
  const double distance = std::sqrt(x * x + y * y + z * z);

  char buffer[192];
  std::snprintf(buffer, sizeof(buffer), "%s conf=%.2f z=%.2f dist=%.2f err=%.2f",
                armor.number.c_str(), armor.detection_confidence, z, distance,
                armor.reproj_error_refined);
  return buffer;
}

std::string firstPoseSummary(
    const std::vector<fyt::auto_aim::ArmorDetection>& detections,
    const std::vector<fyt::auto_aim::PoseEstimate>& poses) {
  if (poses.empty()) return "none";

  const auto& pose = poses.front();
  const std::string id = detections.empty() ? "?" : detections.front().publish_number;
  const bool finite = pose.translation.allFinite();
  std::string reject_reason = "ok";
  if (!pose.valid) {
    reject_reason = "invalid";
  } else if (!finite) {
    reject_reason = "nonfinite";
  } else if (pose.translation.z() <= 0.0) {
    reject_reason = "z<=0";
  }

  char buffer[256];
  std::snprintf(
      buffer, sizeof(buffer),
      "%s valid=%d xyz=(%.2f,%.2f,%.2f) raw=%.2f refined=%.2f reason=%s",
      id.c_str(), pose.valid ? 1 : 0, pose.translation.x(),
      pose.translation.y(), pose.translation.z(), pose.reproj_error_raw,
      pose.reproj_error_refined, reject_reason.c_str());
  return buffer;
}

void applyFeedbackPose(hfut::CameraFrame& frame,
                       const hfut::io::SerialFeedback& feedback,
                       const hfut::CameraToBarrelExtrinsics& extrinsics) {
  const Eigen::Matrix3d barrel_to_control =
      (Eigen::AngleAxisd(feedback.yaw_rad, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-feedback.pitch_rad, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(feedback.roll_rad, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const Eigen::Matrix3d camera_to_barrel =
      extrinsics.cameraOpticalToBarrelRotation();
  frame.q_cam2world =
      Eigen::Quaterniond(barrel_to_control * camera_to_barrel).normalized();
  frame.t_cam2world = barrel_to_control * extrinsics.xyz;
  frame.gimbal_yaw = feedback.yaw_rad;
  frame.gimbal_pitch = feedback.pitch_rad;
}

hfut::GimbalCommand toRadiansCommand(const rm_interfaces::msg::GimbalCmd& cmd_deg) {
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
  out.target_id = cmd_deg.target_id;
  out.mode = static_cast<hfut::GimbalMode>(cmd_deg.mode);
  return out;
}

void drawDebugLine(cv::Mat& image, int& y, const std::string& text,
                   const cv::Scalar& color = cv::Scalar(80, 255, 80)) {
  if (image.empty()) return;
  constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;
  constexpr double kScale = 0.55;
  constexpr int kThickness = 1;
  int baseline = 0;
  const cv::Size size = cv::getTextSize(text, kFont, kScale, kThickness, &baseline);
  const int x = 10;
  const int top = std::max(0, y - size.height - 5);
  cv::rectangle(image, cv::Rect(x - 4, top, size.width + 8, size.height + baseline + 8),
                cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(image, text, cv::Point(x, y), kFont, kScale, color, kThickness,
              cv::LINE_AA);
  y += size.height + baseline + 10;
}

void drawWebOverlay(cv::Mat& image, const hfut::io::DebugMjpegStatus& status) {
  int y = 24;
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer),
                "frames=%llu fps=%.1f det=%d poses=%d armors=%d tracked=%d",
                static_cast<unsigned long long>(status.frames), status.fps,
                status.detections, status.poses, status.armors, status.tracked);
  drawDebugLine(image, y, buffer);

  std::snprintf(buffer, sizeof(buffer),
                "fb yaw=%.2f pitch=%.2f cmd yaw=%.2f pitch=%.2f mode=%d",
                status.feedback_yaw_deg, status.feedback_pitch_deg,
                status.command_yaw_deg, status.command_pitch_deg, status.mode);
  drawDebugLine(image, y, buffer, cv::Scalar(255, 220, 120));

  std::snprintf(buffer, sizeof(buffer),
                "target_dist=%.2fm cmd_dist=%.2fm fire_advice=%d",
                status.target_distance_m, status.command_distance_m,
                status.fire_advice ? 1 : 0);
  drawDebugLine(image, y, buffer, cv::Scalar(120, 220, 255));

  std::snprintf(buffer, sizeof(buffer),
                "latency=%.1fms fb_age=%.0fms dry=%d fire_enabled=%d",
                status.latency_ms, status.feedback_age_ms, status.dry_run ? 1 : 0,
                status.fire_enabled ? 1 : 0);
  drawDebugLine(image, y, buffer, status.fire_advice ? cv::Scalar(80, 255, 80)
                                                     : cv::Scalar(180, 180, 180));

  const std::string line = "selected=" + status.selected_id +
      " state=" + status.track_state;
  drawDebugLine(image, y, line, cv::Scalar(220, 220, 220));
}

int run(const Options& options) {
  using namespace fyt::auto_aim;

  const std::string detector_cfg = options.config_dir + "/detector.yaml";
  const std::string tracker_cfg = options.config_dir + "/tracker.yaml";
  const std::string controller_cfg = options.config_dir + "/controller.yaml";
  const std::string master_cfg = options.config_dir + "/gimbal_pipeline.yaml";

  const auto source_calibration = loadCalibration(options.camera_info_path);
  const auto calibration_mode =
      hfut::video::parseCalibrationMode(options.calibration_mode);

  auto camera = createCameraSource(options, toIntrinsics(source_calibration));
  if (!camera->open()) {
    throw std::runtime_error("camera open failed: " + camera->errorMessage());
  }

  const auto serial_config = makeSerialConfig(options);
  hfut::io::InfantrySerialTransport serial(serial_config);
  if (!options.dry_run && !serial.open()) {
    throw std::runtime_error("serial open failed: " + serial.errorMessage());
  }

  DetectorConfig detector_config =
      hfut::detector::loadDetectorConfigFile(detector_cfg);
  ArmorDetectorNN detector(detector_config);
  detector.setTargetColor(parseEnemyColor(options.enemy_color));
  if (!detector.initialize()) {
    throw std::runtime_error("detector initialization failed");
  }

  ArmorPoseEstimatorAdapter pose_estimator(detector_config.pose);
  if (detector_config.pose.refiner.mode == "single_yaw") {
    pose_estimator.setRefiner(std::make_shared<SingleYawRefiner>(
        detector_config.pose.single_yaw, detector_config.pose.gate));
  }

  hfut::pipeline::PipelineOverrides pipeline_overrides;
  pipeline_overrides.bullet_speed = options.bullet_speed;
  pipeline_overrides.controller_strategy = options.controller_strategy;
  hfut::pipeline::Pipeline pipeline(
      {tracker_cfg, controller_cfg, master_cfg},
      "gimbal_pipeline", pipeline_overrides);

  std::unique_ptr<DebugDrawer> drawer;
  if (options.display || options.web_view) {
    drawer = std::make_unique<DebugDrawer>();
  }
  if (options.display) {
    cv::namedWindow("hfut bringup real", cv::WINDOW_NORMAL);
    cv::resizeWindow("hfut bringup real", 960, 720);
  }

  std::unique_ptr<hfut::io::DebugMjpegServer> web_server;
  if (options.web_view) {
    hfut::io::DebugMjpegServerConfig web_config;
    web_config.host = options.web_host;
    web_config.port = static_cast<uint16_t>(options.web_port);
    web_config.jpeg_quality = options.web_jpeg_quality;
    web_config.max_width = options.web_max_width;
    web_server = std::make_unique<hfut::io::DebugMjpegServer>(web_config);
    if (!web_server->start()) {
      throw std::runtime_error("web debug server failed: " + web_server->errorMessage());
    }
    std::printf("[bringup_real] web_view=%s stream=/stream.mjpg status=/status.json\n",
                web_server->url().c_str());
  }

  if (options.camera_backend == "opencv" && options.camera_source.empty()) {
    std::printf(
        "[bringup_real] camera=%s:index %d serial=%s:%s@%d dry_run=%s fire=%s\n",
        options.camera_backend.c_str(), options.camera_index,
        hfut::io::infantryPacketLayoutName(serial_config.tx_layout),
        options.serial_port.c_str(), options.serial_baudrate,
        options.dry_run ? "true" : "false",
        options.enable_fire ? "enabled" : "disabled");
  } else {
    std::printf(
        "[bringup_real] camera=%s:%s serial=%s:%s@%d dry_run=%s fire=%s\n",
        options.camera_backend.c_str(),
        (options.camera_backend == "opencv" ? options.camera_source : options.camera_sn).c_str(),
        hfut::io::infantryPacketLayoutName(serial_config.tx_layout),
        options.serial_port.c_str(), options.serial_baudrate,
        options.dry_run ? "true" : "false",
        options.enable_fire ? "enabled" : "disabled");
  }
  if (serial_config.tx_layout != serial_config.rx_layout) {
    std::printf("[bringup_real] serial_rx=%s\n",
                hfut::io::infantryPacketLayoutName(serial_config.rx_layout));
  }
  if (serial_config.tx_layout == hfut::io::InfantryPacketLayout::kInfantry32) {
    std::printf("[bringup_real] infantry32_tail_fields=%s\n",
                hfut::io::infantry32TailFieldsName(serial_config.tail_fields));
  }
  std::printf(
      "[bringup_real] camera_info=%s calibration_mode=%s enemy=%s bullet=%.2f\n",
      options.camera_info_path.c_str(),
      hfut::video::calibrationModeName(calibration_mode),
      options.enemy_color.c_str(), options.bullet_speed);

  hfut::io::SerialFeedback latest_feedback;
  bool have_feedback = options.dry_run || !options.require_feedback;
  int processed = 0;
  uint64_t frames = 0;
  const auto run_start = std::chrono::steady_clock::now();
  auto last_log = std::chrono::steady_clock::now();
  auto last_feedback_wait_log = std::chrono::steady_clock::now();
  auto last_feedback_time = std::chrono::steady_clock::now();
  auto last_feedback_heartbeat_time = std::chrono::steady_clock::now();
  const auto elapsedMs = [](const auto& start, const auto& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };

  while (!g_stop.load() &&
         (options.max_frames < 0 || processed < options.max_frames)) {
    const auto loop_start = std::chrono::steady_clock::now();
    const auto serial_rx_start = loop_start;
    if (!options.dry_run) {
      hfut::io::SerialFeedback feedback;
      if (serial.readFeedback(feedback)) {
        latest_feedback = feedback;
        latest_feedback.bullet_speed = options.bullet_speed;
        have_feedback = true;
        last_feedback_time = std::chrono::steady_clock::now();
      }
    }
    const auto serial_rx_end = std::chrono::steady_clock::now();
    const double serial_rx_ms = elapsedMs(serial_rx_start, serial_rx_end);

    const auto feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_feedback_time);
    const bool feedback_ready =
        options.dry_run || !options.require_feedback ||
        (have_feedback && feedback_age.count() <= options.feedback_timeout_ms);
    if (!feedback_ready) {
      const auto now = std::chrono::steady_clock::now();
      if (!options.dry_run &&
          now - last_feedback_heartbeat_time >= std::chrono::milliseconds(20)) {
        hfut::GimbalCommand heartbeat;
        heartbeat.yaw = latest_feedback.yaw_rad;
        heartbeat.pitch = latest_feedback.pitch_rad;
        heartbeat.fire_advice = false;
        heartbeat.mode = hfut::GimbalMode::no_valid_measurement;
        serial.sendCommand(heartbeat);
        last_feedback_heartbeat_time = now;
      }
      if (now - last_feedback_wait_log > std::chrono::seconds(1)) {
        std::printf(
            "[bringup_real] waiting for fresh serial feedback on %s "
            "(last_age=%lldms)\n",
            options.serial_port.c_str(),
            static_cast<long long>(have_feedback ? feedback_age.count() : -1));
        std::fflush(stdout);
        last_feedback_wait_log = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    hfut::CameraFrame frame;
    const auto capture_start = std::chrono::steady_clock::now();
    if (!camera->read(frame, std::chrono::milliseconds(200))) {
      std::fprintf(stderr, "[bringup_real] camera read timeout: %s\n",
                   camera->errorMessage().c_str());
      continue;
    }
    const auto capture_end = std::chrono::steady_clock::now();

    const auto adapted_calibration = hfut::video::adaptCalibration(
        source_calibration, frame.image.cols, frame.image.rows, calibration_mode);
    frame.intrinsics = toIntrinsics(adapted_calibration);
    applyFeedbackPose(frame, latest_feedback, options.camera_to_barrel);
    pipeline.updateFov(
        std::atan(static_cast<double>(frame.intrinsics.width) /
                  (2.0 * frame.intrinsics.fx)),
        std::atan(static_cast<double>(frame.intrinsics.height) /
                  (2.0 * frame.intrinsics.fy)));
    const auto setup_end = std::chrono::steady_clock::now();

    const auto processing_start = std::chrono::steady_clock::now();
    auto detections = detector.detectBatch({frame.image}, {makeHeader(frame.sim_time_s)});
    std::vector<ArmorDetection> dets;
    if (!detections.empty()) dets = detections.front().detections;
    const auto detect_end = std::chrono::steady_clock::now();

    const auto poses = pose_estimator.estimateBatch(
        dets, toCameraInfo(frame.intrinsics), frame.R_cam2world());
    const auto armors = buildValidArmors(dets, poses);
    const auto pnp_end = std::chrono::steady_clock::now();
    pipeline.updateTracking(armors, frame.R_cam2world(), frame.t_cam2world,
                            frame.sim_time_s);
    const auto track_end = std::chrono::steady_clock::now();

    const double latency_s = std::chrono::duration<double>(track_end - processing_start).count();
    auto command_deg = pipeline.computeCommand(
        frame.gimbal_yaw, frame.gimbal_pitch, frame.sim_time_s + latency_s);
    const hfut::GimbalCommand algorithm_command = toRadiansCommand(command_deg);
    hfut::GimbalCommand command = algorithm_command;
    if (!options.enable_fire) command.fire_advice = false;
    const auto control_end = std::chrono::steady_clock::now();

    const auto serial_tx_start = control_end;
    if (!options.dry_run) {
      serial.sendCommand(command);
    }
    const auto serial_tx_end = std::chrono::steady_clock::now();
    const double serial_tx_ms = elapsedMs(serial_tx_start, serial_tx_end);

    ++frames;
    ++processed;

    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - run_start).count();
    const double runtime_fps = elapsed_s > 0.0 ? static_cast<double>(frames) / elapsed_s : 0.0;
    const auto& debug = pipeline.lastDebug();
    hfut::io::DebugMjpegStatus web_status;
    if (web_server) {
      double target_distance_m = 0.0;
      if (!armors.armors.empty()) {
        const auto& position = armors.armors.front().pose.position;
        target_distance_m = std::sqrt(position.x * position.x +
                                      position.y * position.y +
                                      position.z * position.z);
      }
      web_status.frames = frames;
      web_status.fps = runtime_fps;
      web_status.latency_ms = latency_s * 1000.0;
      web_status.detections = static_cast<int>(dets.size());
      web_status.poses = static_cast<int>(poses.size());
      web_status.armors = static_cast<int>(armors.armors.size());
      web_status.tracked = debug.num_tracked;
      web_status.selected_id = debug.selected_id.empty() ? "none" : debug.selected_id;
      web_status.track_state = trackStateName(debug.selected_track_state);
      web_status.reason = debug.tracker_decision_reason.empty()
          ? "none"
          : debug.tracker_decision_reason;
      web_status.mode = static_cast<int>(command.mode);
      web_status.feedback_yaw_deg = frame.gimbal_yaw * kRadToDeg;
      web_status.feedback_pitch_deg = frame.gimbal_pitch * kRadToDeg;
      web_status.command_yaw_deg = command.yaw * kRadToDeg;
      web_status.command_pitch_deg = command.pitch * kRadToDeg;
      web_status.command_yaw_vel_rad_s = command.yaw_vel;
      web_status.command_pitch_vel_rad_s = command.pitch_vel;
      web_status.command_yaw_acc_rad_s2 = command.yaw_acc;
      web_status.command_pitch_acc_rad_s2 = command.pitch_acc;
      web_status.target_distance_m = target_distance_m;
      web_status.command_distance_m = command.distance;
      web_status.yaw_error_deg = web_status.command_yaw_deg - web_status.feedback_yaw_deg;
      web_status.pitch_error_deg = web_status.command_pitch_deg - web_status.feedback_pitch_deg;
      web_status.distance_error_m = web_status.command_distance_m - web_status.target_distance_m;
      web_status.feedback_age_ms = options.dry_run ? 0.0 : static_cast<double>(feedback_age.count());
      web_status.fire_advice = algorithm_command.fire_advice;
      web_status.fire = command.fire_advice;
      web_status.dry_run = options.dry_run;
      web_status.fire_enabled = options.enable_fire;
      web_status.enemy_color = options.enemy_color;
      web_status.camera_backend = options.camera_backend;
      web_status.serial_tx = hfut::io::infantryPacketLayoutName(serial_config.tx_layout);
      web_status.serial_rx = hfut::io::infantryPacketLayoutName(serial_config.rx_layout);
    }

    const auto visual_start = std::chrono::steady_clock::now();
    if (drawer && (options.display || web_server)) {
      cv::Mat visual = frame.image.clone();
      drawer->drawDetections(visual, dets, true);
      drawer->drawCrosshair(visual);
      if (web_server) {
        drawWebOverlay(visual, web_status);
        if (frames % static_cast<uint64_t>(options.web_frame_step) == 0U) {
          web_server->publish(visual, web_status);
        }
      }
      if (options.display) {
        cv::imshow("hfut bringup real", visual);
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q') break;
      }
    }
    const auto visual_end = std::chrono::steady_clock::now();
    const double setup_ms = elapsedMs(capture_end, setup_end);
    const double detect_ms = elapsedMs(processing_start, detect_end);
    const double pnp_ms = elapsedMs(detect_end, pnp_end);
    const double track_ms = elapsedMs(pnp_end, track_end);
    const double control_ms = elapsedMs(track_end, control_end);
    const double visual_ms = elapsedMs(visual_start, visual_end);
    const double total_ms = elapsedMs(loop_start, visual_end);

    if (visual_end - last_log > std::chrono::seconds(1)) {
      std::printf(
          "[bringup_real] frames=%llu fps=%.1f det=%zu poses=%zu armors=%zu "
          "first=%s pose0=%s tracked=%d selected=%s state=%s mode=%d "
          "yaw=%.2f pitch=%.2f yaw_vel=%.3f pitch_vel=%.3f "
          "yaw_acc=%.3f pitch_acc=%.3f distance=%.3f fire=%d "
          "latency=%.1fms reason=%s\n",
          static_cast<unsigned long long>(frames), runtime_fps,
          dets.size(), poses.size(),
          armors.armors.size(), firstArmorSummary(armors).c_str(),
          firstPoseSummary(dets, poses).c_str(),
          debug.num_tracked,
          debug.selected_id.empty() ? "none" : debug.selected_id.c_str(),
          trackStateName(debug.selected_track_state),
          static_cast<int>(command.mode),
          command.yaw * kRadToDeg, command.pitch * kRadToDeg,
          command.yaw_vel, command.pitch_vel,
          command.yaw_acc, command.pitch_acc,
          command.distance,
          command.fire_advice ? 1 : 0, latency_s * 1000.0,
          debug.tracker_decision_reason.empty()
              ? "none"
              : debug.tracker_decision_reason.c_str());
      std::printf(
          "[bringup_real] timing_ms total=%.1f serial_rx=%.2f capture=%.2f "
          "setup=%.2f detect=%.2f pnp=%.2f track=%.2f control=%.2f "
          "serial_tx=%.2f visual=%.2f\n",
          total_ms, serial_rx_ms, elapsedMs(capture_start, capture_end),
          setup_ms, detect_ms, pnp_ms, track_ms, control_ms,
          serial_tx_ms, visual_ms);
      std::fflush(stdout);
      last_log = visual_end;
    }
  }

  if (options.display) cv::destroyAllWindows();
  return processed > 0 ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif
  try { FYT_REGISTER_LOGGER("armor_detector", "/tmp/hfut_auto_aim_log", INFO); } catch (...) {}
  try { FYT_REGISTER_LOGGER("armor_detector_nn", "/tmp/hfut_auto_aim_log", INFO); } catch (...) {}

  try {
    return run(parseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[bringup_real] fatal: %s\n", error.what());
    return 1;
  }
}
