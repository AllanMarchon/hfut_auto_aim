#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

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
      "  --display                显示采图窗口，按 s 保存，q/Esc 退出\n",
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
    } else {
      throw std::invalid_argument("不支持或不完整的参数: " + arg);
    }
  }

  if (options.camera_backend != "hik" && options.camera_backend != "opencv") {
    throw std::invalid_argument("camera-backend 必须是 hik 或 opencv");
  }
  if (options.save_interval < 0) throw std::invalid_argument("save-interval 必须 >= 0");
  if (options.max_images == 0) throw std::invalid_argument("max-images 不能为 0");
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

int run(const Options& options) {
  std::filesystem::create_directories(options.output_dir);
  auto camera = createCamera(options);
  if (!camera->open()) throw std::runtime_error("相机打开失败: " + camera->errorMessage());
  if (options.display) cv::namedWindow("capture_calibration_images", cv::WINDOW_NORMAL);

  std::printf("标定采图启动: camera=%s output=%s interval=%d max=%d\n",
              options.camera_backend.c_str(), options.output_dir.c_str(),
              options.save_interval, options.max_images);
  int saved = 0;
  int frame_count = 0;
  while (options.max_images < 0 || saved < options.max_images) {
    hfut::CameraFrame frame;
    if (!camera->read(frame, std::chrono::milliseconds(500))) {
      std::fprintf(stderr, "相机读帧超时: %s\n", camera->errorMessage().c_str());
      continue;
    }
    ++frame_count;
    bool request_save = options.save_interval > 0 && frame_count % options.save_interval == 0;

    if (options.display) {
      cv::Mat preview = frame.image.clone();
      const std::string text = "saved=" + std::to_string(saved) + "  s保存  q退出";
      cv::putText(preview, text, cv::Point(18, 32), cv::FONT_HERSHEY_SIMPLEX,
                  0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::imshow("capture_calibration_images", preview);
      const int key = cv::waitKey(1) & 0xff;
      if (key == 'q' || key == 27) break;
      if (key == 's' || key == 'S') request_save = true;
    }

    if (request_save && saveImage(options, saved, frame.image)) ++saved;
  }
  if (options.display) cv::destroyAllWindows();
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
