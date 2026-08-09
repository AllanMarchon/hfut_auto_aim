#include "io/camera/mindvision_camera_source.hpp"

#include <cstdio>
#include <cstring>
#include <utility>

#include <opencv2/imgproc.hpp>

#include <CameraApi.h>

namespace hfut::io {
namespace {

std::string statusString(int status) {
  char buffer[32]{};
  std::snprintf(buffer, sizeof(buffer), "%d", status);
  return buffer;
}

}  // namespace

MindvisionCameraSource::MindvisionCameraSource(
    MindvisionCameraSourceConfig config)
    : config_(std::move(config)),
      start_time_(std::chrono::steady_clock::now()) {}

MindvisionCameraSource::~MindvisionCameraSource() { close(); }

void MindvisionCameraSource::setError(const std::string& action, int status) {
  error_message_ = action + " failed, status=" + statusString(status);
}

bool MindvisionCameraSource::open() {
  if (isOpen()) return true;

  int status = CameraSdkInit(1);
  if (status != CAMERA_STATUS_SUCCESS) {
    setError("CameraSdkInit", status);
    return false;
  }
  if (!selectAndOpenDevice()) return false;
  if (!applyOptions()) {
    close();
    return false;
  }

  status = CameraPlay(handle_);
  if (status != CAMERA_STATUS_SUCCESS) {
    setError("CameraPlay", status);
    close();
    return false;
  }

  start_time_ = std::chrono::steady_clock::now();
  seq_ = 0;
  error_message_.clear();
  return true;
}

void MindvisionCameraSource::close() {
  if (handle_ >= 0) {
    CameraUnInit(handle_);
    handle_ = -1;
  }
  rgb_buffer_.clear();
}

bool MindvisionCameraSource::isOpen() const { return handle_ >= 0; }

bool MindvisionCameraSource::selectAndOpenDevice() {
  constexpr int kMaxCameras = 10;
  int camera_count = kMaxCameras;
  tSdkCameraDevInfo devices[kMaxCameras]{};
  int status = CameraEnumerateDevice(devices, &camera_count);
  if (status != CAMERA_STATUS_SUCCESS) {
    setError("CameraEnumerateDevice", status);
    return false;
  }
  if (camera_count <= 0) {
    error_message_ = "no MindVision camera found";
    return false;
  }

  tSdkCameraDevInfo* selected = nullptr;
  if (config_.camera_sn.empty()) {
    selected = &devices[0];
  } else {
    for (int i = 0; i < camera_count; ++i) {
      if (std::string(devices[i].acSn) == config_.camera_sn) {
        selected = &devices[i];
        break;
      }
    }
  }

  if (selected == nullptr) {
    error_message_ = "MindVision camera_sn not found: " + config_.camera_sn;
    return false;
  }

  status = CameraInit(selected, -1, -1, &handle_);
  if (status != CAMERA_STATUS_SUCCESS) {
    setError("CameraInit", status);
    handle_ = -1;
    return false;
  }
  return true;
}

bool MindvisionCameraSource::applyOptions() {
  tSdkCameraCapbility capability{};
  int status = CameraGetCapability(handle_, &capability);
  if (status != CAMERA_STATUS_SUCCESS) {
    setError("CameraGetCapability", status);
    return false;
  }

  CameraSetAeState(handle_, false);
  CameraSetTriggerMode(handle_, 0);
  CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_RGB8);

  if (config_.exposure_time_us > 0.0) {
    status = CameraSetExposureTime(handle_, config_.exposure_time_us);
    if (status != CAMERA_STATUS_SUCCESS) setError("CameraSetExposureTime", status);
  }

  if (config_.analog_gain >= 0) {
    status = CameraSetAnalogGain(handle_, config_.analog_gain);
    if (status != CAMERA_STATUS_SUCCESS) setError("CameraSetAnalogGain", status);
  }

  if (config_.fps > 0.0) {
    status = CameraSetFrameRate(handle_, static_cast<int>(config_.fps));
    if (status != CAMERA_STATUS_SUCCESS && config_.frame_speed >= 0) {
      CameraSetFrameSpeed(handle_, config_.frame_speed);
    }
  } else if (config_.frame_speed >= 0) {
    CameraSetFrameSpeed(handle_, config_.frame_speed);
  }

  if (config_.width > 0 && config_.height > 0) {
    tSdkImageResolution resolution{};
    resolution.iIndex = 0xFF;
    resolution.iWidth = config_.width;
    resolution.iHeight = config_.height;
    resolution.iWidthFOV = config_.width;
    resolution.iHeightFOV = config_.height;
    resolution.iHOffsetFOV = 0;
    resolution.iVOffsetFOV = 0;
    status = CameraSetImageResolution(handle_, &resolution);
    if (status != CAMERA_STATUS_SUCCESS) {
      setError("CameraSetImageResolution", status);
    }
  }

  const int max_width = capability.sResolutionRange.iWidthMax;
  const int max_height = capability.sResolutionRange.iHeightMax;
  if (max_width > 0 && max_height > 0) {
    rgb_buffer_.assign(static_cast<std::size_t>(max_width) * max_height * 3, 0);
  }
  return true;
}

bool MindvisionCameraSource::read(CameraFrame& frame,
                                  std::chrono::milliseconds timeout) {
  if (!isOpen() && !open()) return false;

  tSdkFrameHead frame_info{};
  unsigned char* raw_buffer = nullptr;
  int status = CameraGetImageBuffer(
      handle_, &frame_info, &raw_buffer, static_cast<UINT>(timeout.count()));
  if (status != CAMERA_STATUS_SUCCESS) {
    setError("CameraGetImageBuffer", status);
    return false;
  }

  if (frame_info.iWidth <= 0 || frame_info.iHeight <= 0) {
    CameraReleaseImageBuffer(handle_, raw_buffer);
    error_message_ = "MindVision returned an empty frame";
    return false;
  }

  const std::size_t need_size =
      static_cast<std::size_t>(frame_info.iWidth) * frame_info.iHeight * 3;
  if (rgb_buffer_.size() < need_size) rgb_buffer_.assign(need_size, 0);

  status = CameraImageProcess(handle_, raw_buffer, rgb_buffer_.data(), &frame_info);
  CameraReleaseImageBuffer(handle_, raw_buffer);
  if (status != CAMERA_STATUS_SUCCESS) {
    setError("CameraImageProcess", status);
    return false;
  }

  cv::Mat rgb(frame_info.iHeight, frame_info.iWidth, CV_8UC3, rgb_buffer_.data());
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  if (config_.flip_image) {
    cv::flip(bgr, frame.image, -1);
  } else {
    frame.image = bgr.clone();
  }
  fillFrameMetadata(frame);
  error_message_.clear();
  return true;
}

void MindvisionCameraSource::fillFrameMetadata(CameraFrame& frame) {
  frame.input_mode = FrameInputMode::vision;
  frame.seq = ++seq_;
  frame.sim_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    start_time_)
          .count();
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
