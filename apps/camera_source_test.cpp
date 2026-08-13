#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "io/camera/camera_source.hpp"
#include "io/camera/opencv_camera_source.hpp"

#if defined(HFUT_HAS_HIK_CAMERA)
#include "io/camera/hik_camera_source.hpp"
#endif

#if defined(HFUT_HAS_MINDVISION_CAMERA)
#include "io/camera/mindvision_camera_source.hpp"
#endif

namespace {

struct Options {
  std::string backend{"opencv"};
  std::string source;
  std::string camera_sn;
  std::string save_path;
  int camera_index{0};
  int width{0};
  int height{0};
  int frames{100};
  int timeout_ms{500};
  double fps{0.0};
  double exposure_time_us{0.0};
  double gain{0.0};
  int analog_gain{-1};
  int frame_speed{-1};
  bool flip_image{false};
  bool display{false};
};

void printUsage(const char* argv0) {
  std::fprintf(
      stderr,
      "Usage: %s [options]\n"
      "  --backend NAME          opencv | hik | mindvision\n"
      "  --source PATH           OpenCV source path/video/URL\n"
      "  --camera-index N        OpenCV device index when source is empty\n"
      "  --camera-sn SN          industrial camera serial number\n"
      "  --width N --height N    requested image size\n"
      "  --fps VALUE             requested frame rate\n"
      "  --exposure-time-us US   requested exposure time\n"
      "  --gain VALUE            Hik/OpenCV gain\n"
      "  --analog-gain N         MindVision analog gain\n"
      "  --frame-speed N         MindVision frame speed mode\n"
      "  --flip-image            flip industrial camera image 180 degrees\n"
      "  --frames N              frames to read, default 100\n"
      "  --timeout-ms N          per-frame timeout, default 500\n"
      "  --save PATH             save the first captured frame\n"
      "  --display               show live frames\n"
      "Example: %s --backend hik --width 1280 --height 1024 --frames 100 --save hik.jpg\n",
      argv0, argv0);
}

std::string optionValue(int argc, char** argv, int& index, const std::string& arg,
                        const char* name) {
  const std::string key{name};
  if (arg == key) {
    if (index + 1 >= argc) return {};
    ++index;
    return argv[index];
  }
  const std::string prefix = key + "=";
  if (arg.rfind(prefix, 0) == 0) return arg.substr(prefix.size());
  return {};
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--display") {
      options.display = true;
    } else if (arg == "--flip-image") {
      options.flip_image = true;
    } else if (auto value = optionValue(argc, argv, i, arg, "--backend"); !value.empty()) {
      options.backend = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--source"); !value.empty()) {
      options.source = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-sn"); !value.empty()) {
      options.camera_sn = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--save"); !value.empty()) {
      options.save_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-index"); !value.empty()) {
      options.camera_index = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--width"); !value.empty()) {
      options.width = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--height"); !value.empty()) {
      options.height = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--frames"); !value.empty()) {
      options.frames = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--timeout-ms"); !value.empty()) {
      options.timeout_ms = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--fps"); !value.empty()) {
      options.fps = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--exposure-time-us"); !value.empty()) {
      options.exposure_time_us = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--gain"); !value.empty()) {
      options.gain = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--analog-gain"); !value.empty()) {
      options.analog_gain = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--frame-speed"); !value.empty()) {
      options.frame_speed = std::stoi(value);
    } else {
      throw std::invalid_argument("unknown or incomplete option: " + arg);
    }
  }
  if (options.frames <= 0) throw std::invalid_argument("--frames must be > 0");
  if (options.timeout_ms <= 0) throw std::invalid_argument("--timeout-ms must be > 0");
  if (options.backend != "opencv" && options.backend != "hik" &&
      options.backend != "mindvision") {
    throw std::invalid_argument("--backend must be opencv, hik, or mindvision");
  }
  return options;
}

std::unique_ptr<hfut::io::CameraSource> createCamera(const Options& options) {
  if (options.backend == "opencv") {
    hfut::io::OpenCvCameraSourceConfig config;
    config.source = options.source;
    config.device_index = options.camera_index;
    config.width = options.width;
    config.height = options.height;
    config.fps = options.fps;
    config.exposure = options.exposure_time_us;
    config.gain = options.gain;
    config.set_exposure = options.exposure_time_us > 0.0;
    config.set_gain = options.gain > 0.0;
    return std::make_unique<hfut::io::OpenCvCameraSource>(config);
  }
  if (options.backend == "hik") {
#if defined(HFUT_HAS_HIK_CAMERA)
    hfut::io::HikCameraSourceConfig config;
    config.camera_sn = options.camera_sn;
    config.width = options.width;
    config.height = options.height;
    config.fps = options.fps;
    config.exposure_time_us = options.exposure_time_us;
    config.gain = options.gain;
    config.flip_image = options.flip_image;
    return std::make_unique<hfut::io::HikCameraSource>(config);
#else
    throw std::runtime_error("Hik backend was not built; rebuild with HFUT_ENABLE_HIK_CAMERA=ON");
#endif
  }

#if defined(HFUT_HAS_MINDVISION_CAMERA)
  hfut::io::MindvisionCameraSourceConfig config;
  config.camera_sn = options.camera_sn;
  config.width = options.width;
  config.height = options.height;
  config.fps = options.fps;
  config.exposure_time_us = options.exposure_time_us;
  config.analog_gain = options.analog_gain;
  config.frame_speed = options.frame_speed;
  config.flip_image = options.flip_image;
  return std::make_unique<hfut::io::MindvisionCameraSource>(config);
#else
  throw std::runtime_error("MindVision backend was not built; rebuild with HFUT_ENABLE_MINDVISION_CAMERA=ON");
#endif
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    auto camera = createCamera(options);
    if (!camera->open()) {
      std::fprintf(stderr, "camera open failed: %s\n", camera->errorMessage().c_str());
      return 1;
    }

    std::printf(
        "camera opened: backend=%s frames=%d timeout=%dms requested=%dx%d fps=%.1f\n",
        options.backend.c_str(), options.frames, options.timeout_ms,
        options.width, options.height, options.fps);

    const auto start = std::chrono::steady_clock::now();
    int captured = 0;
    int saved = 0;
    for (; captured < options.frames; ++captured) {
      hfut::CameraFrame frame;
      if (!camera->read(frame, std::chrono::milliseconds(options.timeout_ms))) {
        std::fprintf(stderr, "camera read failed after %d frames: %s\n", captured,
                     camera->errorMessage().c_str());
        return 2;
      }
      if (frame.image.empty()) {
        std::fprintf(stderr, "camera returned an empty image at frame %d\n", captured);
        return 3;
      }
      if (captured == 0) {
        std::printf("first frame: %dx%d channels=%d seq=%llu time=%.6f\n",
                    frame.image.cols, frame.image.rows, frame.image.channels(),
                    static_cast<unsigned long long>(frame.seq), frame.sim_time_s);
        if (!options.save_path.empty()) {
          if (!cv::imwrite(options.save_path, frame.image)) {
            std::fprintf(stderr, "failed to save frame: %s\n", options.save_path.c_str());
            return 4;
          }
          saved = 1;
          std::printf("saved first frame: %s\n", options.save_path.c_str());
        }
      }
      if (options.display) {
        cv::imshow("camera_source_test", frame.image);
        if (cv::waitKey(1) == 27) break;
      }
    }
    const auto end = std::chrono::steady_clock::now();
    const double seconds =
        std::chrono::duration<double>(end - start).count();
    const double fps = seconds > 0.0 ? static_cast<double>(captured) / seconds : 0.0;
    std::printf("captured=%d saved=%d elapsed=%.3fs avg_fps=%.1f\n",
                captured, saved, seconds, fps);
    return captured > 0 ? 0 : 5;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "camera_source_test failed: %s\n", e.what());
    return 10;
  }
}
