#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "hfut_auto_aim/camera_frame.hpp"
#include "io/camera/camera_source.hpp"
#ifdef HFUT_HAS_HIK_CAMERA
#include "io/camera/hik_camera_source.hpp"
#endif
#include "io/camera/opencv_camera_source.hpp"

namespace {

struct Options {
  std::string hardware_config{"configs/hardware.yaml"};
  std::string camera_backend{"hik"};
  std::string camera_source;
  std::string camera_sn;
  int camera_index{0};
  int camera_width{0};
  int camera_height{0};
  double camera_fps{0.0};
  double exposure_time_us{0.0};
  double gain{0.0};
  bool flip_image{false};

  std::string output_dir{"calibration/images"};
  std::string prefix{"calib"};
  int save_interval{20};
  int max_images{40};
  bool display{false};

  bool auto_chessboard{false};
  int pattern_cols{9};
  int pattern_rows{6};
  int min_save_gap_frames{15};
  double min_sharpness{30.0};
  double min_pose_delta{0.08};
  double min_board_area{0.001};
  double max_board_area{0.85};
};

struct BoardObservation {
  bool found{false};
  double center_x{0.0};
  double center_y{0.0};
  double area{0.0};
  double skew{0.0};
  double sharpness{0.0};
  std::string reject_reason;
  std::vector<cv::Point2f> corners;
};

std::string optionValue(int argc, char** argv, int& index,
                        const std::string& arg, const std::string& name) {
  const std::string prefix = name + "=";
  if (arg.compare(0, prefix.size(), prefix) == 0) return arg.substr(prefix.size());
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

void loadHardwareConfig(Options& options) {
  const YAML::Node file_root = YAML::LoadFile(options.hardware_config);
  const YAML::Node root = file_root["hardware"] ? file_root["hardware"]
                          : (file_root["real_vehicle"] ? file_root["real_vehicle"] : file_root);
  const YAML::Node camera = root["camera"];
  if (!camera) return;
  options.camera_backend = parseString(camera["backend"], options.camera_backend);
  options.camera_source = parseString(camera["source"], options.camera_source);
  options.camera_sn = parseString(camera["camera_sn"], options.camera_sn);
  options.camera_index = parseInt(camera["device_index"], options.camera_index);
  options.camera_width = parseInt(camera["width"], options.camera_width);
  options.camera_height = parseInt(camera["height"], options.camera_height);
  options.camera_fps = parseDouble(camera["fps"], options.camera_fps);
  options.exposure_time_us = parseDouble(camera["exposure_time_us"], options.exposure_time_us);
  options.gain = parseDouble(camera["gain"], options.gain);
  options.flip_image = parseBool(camera["flip_image"], options.flip_image);
}

void printUsage(const char* argv0) {
  std::fprintf(
      stderr,
      "用法: %s [options]\n"
      "  --hardware-config PATH   硬件配置，默认 configs/hardware.yaml\n"
      "  --camera-backend NAME    hik | opencv\n"
      "  --camera-source PATH     OpenCV 视频源或设备路径\n"
      "  --camera-index N         OpenCV 设备序号\n"
      "  --camera-sn SN           海康相机序列号\n"
      "  --exposure-time-us US    曝光时间覆盖\n"
      "  --gain VALUE             增益覆盖\n"
      "  --output-dir PATH        图片输出目录，默认 calibration/images\n"
      "  --prefix NAME            图片名前缀，默认 calib\n"
      "  --save-interval N        每 N 帧自动保存一次；0 表示只按 s 保存\n"
      "  --max-images N           保存 N 张后退出；-1 表示手动退出\n"
      "  --display                显示采图窗口，按 s 保存，q/Esc 退出\n"
      "  --auto-chessboard        自动识别棋盘并筛选非重复样本后保存\n"
      "  --pattern-cols N         棋盘格内角点列数，默认 9\n"
      "  --pattern-rows N         棋盘格内角点行数，默认 6\n"
      "  --min-sharpness VALUE    自动采图最小清晰度，0 表示关闭，默认 30\n"
      "  --min-save-gap-frames N  自动采图两次保存的最小帧间隔，默认 15\n"
      "  --min-pose-delta VALUE   自动采图最小姿态差异，默认 0.08\n"
      "  --min-board-area VALUE   棋盘外接框最小画面占比，默认 0.001\n"
      "  --max-board-area VALUE   棋盘外接框最大画面占比，默认 0.85\n",
      argv0);
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (auto value = optionValue(argc, argv, i, arg, "--hardware-config"); !value.empty()) {
      options.hardware_config = value;
    }
  }

  loadHardwareConfig(options);

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      continue;
    } else if (arg == "--display") {
      options.display = true;
    } else if (arg == "--auto-chessboard") {
      options.auto_chessboard = true;
    } else if (arg == "--hardware-config") {
      ++i;
    } else if (arg.rfind("--hardware-config=", 0) == 0) {
      continue;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-backend"); !value.empty()) {
      options.camera_backend = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-source"); !value.empty()) {
      options.camera_source = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-index"); !value.empty()) {
      options.camera_index = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-sn"); !value.empty()) {
      options.camera_sn = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--exposure-time-us"); !value.empty()) {
      options.exposure_time_us = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--gain"); !value.empty()) {
      options.gain = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--output-dir"); !value.empty()) {
      options.output_dir = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--prefix"); !value.empty()) {
      options.prefix = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--save-interval"); !value.empty()) {
      options.save_interval = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--max-images"); !value.empty()) {
      options.max_images = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--pattern-cols"); !value.empty()) {
      options.pattern_cols = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--pattern-rows"); !value.empty()) {
      options.pattern_rows = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--min-sharpness"); !value.empty()) {
      options.min_sharpness = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--min-save-gap-frames"); !value.empty()) {
      options.min_save_gap_frames = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--min-pose-delta"); !value.empty()) {
      options.min_pose_delta = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--min-board-area"); !value.empty()) {
      options.min_board_area = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--max-board-area"); !value.empty()) {
      options.max_board_area = std::stod(value);
    } else {
      throw std::invalid_argument("不支持或不完整的参数: " + arg);
    }
  }

  if (options.camera_backend != "hik" && options.camera_backend != "opencv") {
    throw std::invalid_argument("camera-backend 必须是 hik 或 opencv");
  }
  if (options.save_interval < 0) throw std::invalid_argument("save-interval 必须 >= 0");
  if (options.max_images == 0) throw std::invalid_argument("max-images 不能为 0");
  if (options.pattern_cols <= 0 || options.pattern_rows <= 0) {
    throw std::invalid_argument("pattern-cols / pattern-rows 必须大于 0");
  }
  if (options.min_save_gap_frames < 0) {
    throw std::invalid_argument("min-save-gap-frames 必须 >= 0");
  }
  if (options.min_sharpness < 0.0 || options.min_pose_delta < 0.0 ||
      options.min_board_area < 0.0 || options.max_board_area <= 0.0 ||
      options.min_board_area >= options.max_board_area) {
    throw std::invalid_argument("自动采图阈值非法");
  }
  return options;
}

std::unique_ptr<hfut::io::CameraSource> createCamera(const Options& options) {
  if (options.camera_backend == "opencv") {
    hfut::io::OpenCvCameraSourceConfig config;
    config.source = options.camera_source;
    config.device_index = options.camera_index;
    config.width = options.camera_width;
    config.height = options.camera_height;
    config.fps = options.camera_fps;
    config.gain = options.gain;
    config.set_gain = options.gain > 0.0;
    return std::make_unique<hfut::io::OpenCvCameraSource>(config);
  }

  if (options.camera_backend == "hik") {
#ifdef HFUT_HAS_HIK_CAMERA
    hfut::io::HikCameraSourceConfig config;
    config.camera_sn = options.camera_sn;
    config.width = options.camera_width;
    config.height = options.camera_height;
    config.fps = options.camera_fps;
    config.exposure_time_us = options.exposure_time_us;
    config.gain = options.gain;
    config.flip_image = options.flip_image;
    return std::make_unique<hfut::io::HikCameraSource>(config);
#else
    throw std::runtime_error("当前二进制未开启 HFUT_ENABLE_HIK_CAMERA，无法使用海康相机");
#endif
  }
  throw std::runtime_error("未知相机后端: " + options.camera_backend);
}

std::string imagePath(const Options& options, int index) {
  std::ostringstream name;
  name << options.prefix << '_' << std::setw(4) << std::setfill('0') << index << ".png";
  return (std::filesystem::path(options.output_dir) / name.str()).string();
}

bool saveImage(const Options& options, int index, const cv::Mat& image) {
  const std::string path = imagePath(options, index);
  if (!cv::imwrite(path, image)) {
    std::fprintf(stderr, "保存失败: %s\n", path.c_str());
    return false;
  }
  std::printf("已保存: %s\n", path.c_str());
  return true;
}

double laplacianSharpness(const cv::Mat& gray) {
  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_64F);
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  return stddev[0] * stddev[0];
}

double pointDistance(const cv::Point2f& lhs, const cv::Point2f& rhs) {
  const double dx = static_cast<double>(lhs.x - rhs.x);
  const double dy = static_cast<double>(lhs.y - rhs.y);
  return std::hypot(dx, dy);
}

BoardObservation detectChessboard(const Options& options, const cv::Mat& image) {
  BoardObservation observation;
  if (image.empty()) {
    observation.reject_reason = "empty";
    return observation;
  }

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  observation.sharpness = laplacianSharpness(gray);
  if (observation.sharpness < options.min_sharpness) {
    observation.reject_reason = "blur";
    return observation;
  }

  const cv::Size pattern_size(options.pattern_cols, options.pattern_rows);
  const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(gray, pattern_size, corners, flags);
  if (!found) {
    observation.reject_reason = "not_found";
    return observation;
  }

  const auto criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                        30, 1e-4);
  cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
  observation.found = true;
  observation.corners = std::move(corners);

  const cv::Rect bounds = cv::boundingRect(observation.corners);
  const double image_area = static_cast<double>(image.cols) * static_cast<double>(image.rows);
  observation.area = static_cast<double>(bounds.area()) / std::max(image_area, 1.0);
  observation.center_x = (bounds.x + bounds.width * 0.5) / std::max(image.cols, 1);
  observation.center_y = (bounds.y + bounds.height * 0.5) / std::max(image.rows, 1);

  const int cols = options.pattern_cols;
  const int rows = options.pattern_rows;
  const cv::Point2f top_left = observation.corners.front();
  const cv::Point2f top_right = observation.corners[cols - 1];
  const cv::Point2f bottom_right = observation.corners[cols * rows - 1];
  const cv::Point2f bottom_left = observation.corners[(rows - 1) * cols];
  const double top = pointDistance(top_left, top_right);
  const double bottom = pointDistance(bottom_left, bottom_right);
  const double left = pointDistance(top_left, bottom_left);
  const double right = pointDistance(top_right, bottom_right);
  observation.skew = std::abs(top - bottom) / std::max((top + bottom) * 0.5, 1.0) +
                     std::abs(left - right) / std::max((left + right) * 0.5, 1.0);

  constexpr double kMinMarginPx = 6.0;
  if (bounds.x < kMinMarginPx || bounds.y < kMinMarginPx ||
      bounds.x + bounds.width > image.cols - kMinMarginPx ||
      bounds.y + bounds.height > image.rows - kMinMarginPx) {
    observation.reject_reason = "edge";
  } else if (observation.area < options.min_board_area) {
    observation.reject_reason = "small";
  } else if (observation.area > options.max_board_area) {
    observation.reject_reason = "large";
  } else {
    observation.reject_reason = "ok";
  }
  return observation;
}

double poseDelta(const BoardObservation& lhs, const BoardObservation& rhs) {
  const double dx = lhs.center_x - rhs.center_x;
  const double dy = lhs.center_y - rhs.center_y;
  const double area_ratio = std::log(std::max(lhs.area, 1e-9) / std::max(rhs.area, 1e-9));
  const double skew_delta = lhs.skew - rhs.skew;
  return std::sqrt(dx * dx + dy * dy) + 0.25 * std::abs(area_ratio) +
         0.5 * std::abs(skew_delta);
}

double nearestPoseDelta(const BoardObservation& observation,
                        const std::vector<BoardObservation>& saved_observations) {
  if (saved_observations.empty()) return std::numeric_limits<double>::infinity();
  double best = std::numeric_limits<double>::infinity();
  for (const auto& saved : saved_observations) {
    best = std::min(best, poseDelta(observation, saved));
  }
  return best;
}

bool shouldAutoSave(const Options& options, const BoardObservation& observation,
                    const std::vector<BoardObservation>& saved_observations,
                    int frame_count, int last_saved_frame, double& delta) {
  delta = nearestPoseDelta(observation, saved_observations);
  if (!observation.found || observation.reject_reason != "ok") return false;
  if (last_saved_frame >= 0 && frame_count - last_saved_frame < options.min_save_gap_frames) {
    return false;
  }
  return saved_observations.empty() || delta >= options.min_pose_delta;
}

void drawStatus(cv::Mat& preview, const Options& options,
                const BoardObservation& observation, int saved, double delta) {
  if (observation.found) {
    cv::drawChessboardCorners(preview, cv::Size(options.pattern_cols, options.pattern_rows),
                              observation.corners, true);
  }
  std::ostringstream text;
  text << "saved=" << saved;
  if (options.auto_chessboard) {
    text << " found=" << (observation.found ? 1 : 0)
         << " reason=" << observation.reject_reason
         << " sharp=" << std::fixed << std::setprecision(0) << observation.sharpness
         << " area=" << std::setprecision(3) << observation.area
         << " delta=" << std::setprecision(3) << delta;
  } else {
    text << "  s save  q quit";
  }
  cv::putText(preview, text.str(), cv::Point(18, 32), cv::FONT_HERSHEY_SIMPLEX,
              0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  if (options.auto_chessboard) {
    cv::putText(preview, "move board across X/Y/size/skew; s saves manually; q quits",
                cv::Point(18, 62), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
}

int run(const Options& options) {
  std::filesystem::create_directories(options.output_dir);
  auto camera = createCamera(options);
  if (!camera->open()) throw std::runtime_error("相机打开失败: " + camera->errorMessage());
  bool display_enabled = options.display;
  if (display_enabled) {
    try {
      cv::namedWindow("capture_calibration_images", cv::WINDOW_NORMAL);
    } catch (const cv::Exception& error) {
      std::fprintf(stderr, "显示窗口初始化失败，切换为无窗口自动采图: %s\n", error.what());
      display_enabled = false;
    }
  }

  std::printf("标定采图启动: camera=%s output=%s interval=%d max=%d auto=%s pattern=%dx%d\n",
              options.camera_backend.c_str(), options.output_dir.c_str(),
              options.save_interval, options.max_images,
              options.auto_chessboard ? "on" : "off",
              options.pattern_cols, options.pattern_rows);
  int saved = 0;
  int frame_count = 0;
  int last_saved_frame = -1;
  std::vector<BoardObservation> saved_observations;
  while (options.max_images < 0 || saved < options.max_images) {
    hfut::CameraFrame frame;
    if (!camera->read(frame, std::chrono::milliseconds(500))) {
      std::fprintf(stderr, "相机读帧超时: %s\n", camera->errorMessage().c_str());
      continue;
    }
    ++frame_count;
    BoardObservation observation;
    double delta = 0.0;
    bool request_save = false;
    if (options.auto_chessboard) {
      observation = detectChessboard(options, frame.image);
      request_save = shouldAutoSave(options, observation, saved_observations,
                                    frame_count, last_saved_frame, delta);
    } else {
      request_save = options.save_interval > 0 && frame_count % options.save_interval == 0;
    }

    if (display_enabled) {
      cv::Mat preview = frame.image.clone();
      drawStatus(preview, options, observation, saved, delta);
      try {
        cv::imshow("capture_calibration_images", preview);
        const int key = cv::waitKey(1) & 0xff;
        if (key == 'q' || key == 27) break;
        if (key == 's' || key == 'S') request_save = true;
      } catch (const cv::Exception& error) {
        std::fprintf(stderr, "显示窗口失效，切换为无窗口自动采图: %s\n", error.what());
        display_enabled = false;
      }
    } else if (options.auto_chessboard && frame_count % 30 == 0) {
      std::printf("自动采图状态: frame=%d saved=%d found=%d reason=%s sharp=%.0f area=%.3f delta=%.3f\n",
                  frame_count, saved, observation.found ? 1 : 0,
                  observation.reject_reason.c_str(), observation.sharpness,
                  observation.area, delta);
    }

    if (request_save && saveImage(options, saved, frame.image)) {
      if (options.auto_chessboard && observation.found && observation.reject_reason == "ok") {
        saved_observations.push_back(observation);
      }
      last_saved_frame = frame_count;
      ++saved;
    }
  }
  if (display_enabled) cv::destroyAllWindows();
  return saved > 0 ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(parseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[capture_calibration_images] fatal: %s\n", error.what());
    return 1;
  }
}
