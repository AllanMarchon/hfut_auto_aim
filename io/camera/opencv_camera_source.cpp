#include "io/camera/opencv_camera_source.hpp"

#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace hfut::io {

OpenCvCameraSource::OpenCvCameraSource(OpenCvCameraSourceConfig config)
    : config_(std::move(config)),
      start_time_(std::chrono::steady_clock::now()) {}

OpenCvCameraSource::~OpenCvCameraSource() { close(); }

bool OpenCvCameraSource::open() {
  if (isOpen()) return true;

  const bool opened = config_.source.empty()
      ? capture_.open(config_.device_index, config_.api_preference)
      : capture_.open(config_.source, config_.api_preference);
  if (!opened) {
    error_message_ = config_.source.empty()
        ? "failed to open camera device index " + std::to_string(config_.device_index)
        : "failed to open camera source " + config_.source;
    return false;
  }

  applyCaptureOptions();
  start_time_ = std::chrono::steady_clock::now();
  seq_ = 0;
  error_message_.clear();
  return true;
}

void OpenCvCameraSource::close() {
  if (capture_.isOpened()) capture_.release();
}

bool OpenCvCameraSource::isOpen() const { return capture_.isOpened(); }

void OpenCvCameraSource::applyCaptureOptions() {
  if (config_.width > 0) {
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(config_.width));
  }
  if (config_.height > 0) {
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(config_.height));
  }
  if (config_.fps > 0.0) {
    capture_.set(cv::CAP_PROP_FPS, config_.fps);
  }
  if (config_.set_exposure) {
    capture_.set(cv::CAP_PROP_EXPOSURE, config_.exposure);
  }
  if (config_.set_gain) {
    capture_.set(cv::CAP_PROP_GAIN, config_.gain);
  }
}

bool OpenCvCameraSource::read(CameraFrame& frame,
                              std::chrono::milliseconds timeout) {
  if (!isOpen() && !open()) return false;

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  cv::Mat image;
  for (;;) {
    if (capture_.read(image) && !image.empty()) {
      const auto timestamp = std::chrono::steady_clock::now();
      if (image.channels() == 1) {
        cv::cvtColor(image, frame.image, cv::COLOR_GRAY2BGR);
      } else if (image.channels() == 4) {
        cv::cvtColor(image, frame.image, cv::COLOR_BGRA2BGR);
      } else {
        frame.image = std::move(image);
      }
      fillFrameMetadata(frame, timestamp);
      return true;
    }

    if (std::chrono::steady_clock::now() >= deadline) {
      error_message_ = "timed out waiting for camera frame";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void OpenCvCameraSource::fillFrameMetadata(
    CameraFrame& frame, std::chrono::steady_clock::time_point timestamp) {
  frame.input_mode = FrameInputMode::vision;
  frame.seq = ++seq_;
  frame.timestamp = timestamp;
  frame.sim_time_s = std::chrono::duration<double>(timestamp - start_time_).count();
  frame.direct_armors.clear();
  frame.direct_position_noise_std_m = 0.0;
  frame.direct_yaw_noise_std_rad = 0.0;
  frame.gimbal_yaw = 0.0;
  frame.gimbal_pitch = 0.0;
  frame.gimbal_yaw_vel = 0.0;
  frame.gimbal_pitch_vel = 0.0;
  frame.q_cam2world = Eigen::Quaterniond::Identity();
  frame.t_cam2world = Eigen::Vector3d::Zero();

  frame.intrinsics = config_.intrinsics;
  if (frame.intrinsics.width <= 0) frame.intrinsics.width = frame.image.cols;
  if (frame.intrinsics.height <= 0) frame.intrinsics.height = frame.image.rows;
}

}  // namespace hfut::io
