#include "io/camera/hik_camera_source.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>

#include <MvCameraControl.h>
#include <PixelType.h>

namespace hfut::io {
namespace {

std::string deviceSerial(const MV_CC_DEVICE_INFO* info) {
  if (info == nullptr) return {};
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char*>(
        info->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  if (info->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char*>(
        info->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  return {};
}

std::string hexStatus(int status) {
  char buffer[32]{};
  std::snprintf(buffer, sizeof(buffer), "0x%x", status);
  return buffer;
}

}  // namespace

HikCameraSource::HikCameraSource(HikCameraSourceConfig config)
    : config_(std::move(config)),
      start_time_(std::chrono::steady_clock::now()) {}

HikCameraSource::~HikCameraSource() { close(); }

void HikCameraSource::setError(const std::string& action, int status) {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  error_message_ = action + " failed, status=" + hexStatus(status);
}

const std::string& HikCameraSource::errorMessage() const {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  error_message_snapshot_ = error_message_;
  return error_message_snapshot_;
}

bool HikCameraSource::open() {
  if (isOpen() && running_.load()) return true;
  if (isOpen()) close();
  if (!selectAndOpenDevice()) return false;
  if (!applyOptions()) {
    close();
    return false;
  }

  int status = MV_CC_StartGrabbing(handle_);
  if (status != MV_OK) {
    setError("MV_CC_StartGrabbing", status);
    close();
    return false;
  }
  // Some firmware accepts this command after StartGrabbing; if it fails,
  // continuous mode can still work, so keep it non-fatal.
  MV_CC_SetCommandValue(handle_, "AcquisitionStart");

  if (!updatePayloadBuffer()) {
    close();
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    start_time_ = std::chrono::steady_clock::now();
    seq_ = 0;
    has_frame_ = false;
    latest_frame_ = CameraFrame{};
    error_message_.clear();
  }
  running_.store(true);
  capture_thread_ = std::thread(&HikCameraSource::captureLoop, this);
  return true;
}

void HikCameraSource::close() {
  running_.store(false);
  frame_cv_.notify_all();
  if (capture_thread_.joinable()) capture_thread_.join();

  if (handle_ != nullptr) {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
  frame_buffer_.clear();
  bgr_buffer_.clear();
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    has_frame_ = false;
    latest_frame_ = CameraFrame{};
  }
}

bool HikCameraSource::isOpen() const { return handle_ != nullptr; }

bool HikCameraSource::selectAndOpenDevice() {
  MV_CC_DEVICE_INFO_LIST device_list{};
  int status = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (status != MV_OK) {
    setError("MV_CC_EnumDevices", status);
    return false;
  }
  if (device_list.nDeviceNum == 0) {
    error_message_ = "no Hik camera found";
    return false;
  }

  MV_CC_DEVICE_INFO* selected = nullptr;
  if (config_.camera_sn.empty()) {
    selected = device_list.pDeviceInfo[0];
  } else {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      MV_CC_DEVICE_INFO* candidate = device_list.pDeviceInfo[i];
      if (deviceSerial(candidate) == config_.camera_sn) {
        selected = candidate;
        break;
      }
    }
  }

  if (selected == nullptr) {
    error_message_ = "Hik camera_sn not found: " + config_.camera_sn;
    return false;
  }

  status = MV_CC_CreateHandle(&handle_, selected);
  if (status != MV_OK) {
    setError("MV_CC_CreateHandle", status);
    handle_ = nullptr;
    return false;
  }
  status = MV_CC_OpenDevice(handle_);
  if (status != MV_OK) {
    setError("MV_CC_OpenDevice", status);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }
  return true;
}

bool HikCameraSource::applyOptions() {
  int status = MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
  if (status != MV_OK) setError("MV_CC_SetEnumValue TriggerMode", status);

  status = MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous");
  if (status != MV_OK) setError("MV_CC_SetEnumValueByString AcquisitionMode", status);

  status = MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
  if (status != MV_OK) setError("MV_CC_SetEnumValue PixelFormat", status);

  if (config_.fps > 0.0) {
    MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    status = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", config_.fps);
    if (status != MV_OK) setError("MV_CC_SetFloatValue AcquisitionFrameRate", status);
  }

  if (config_.exposure_time_us > 0.0) {
    status = MV_CC_SetFloatValue(handle_, "ExposureTime", config_.exposure_time_us);
    if (status != MV_OK) setError("MV_CC_SetFloatValue ExposureTime", status);
  }

  if (config_.gain > 0.0) {
    status = MV_CC_SetFloatValue(handle_, "Gain", config_.gain);
    if (status != MV_OK) setError("MV_CC_SetFloatValue Gain", status);
  }

  if (config_.width > 0 && config_.height > 0) {
    MV_CC_SetIntValue(handle_, "OffsetX", 0);
    MV_CC_SetIntValue(handle_, "OffsetY", 0);
    status = MV_CC_SetIntValue(handle_, "Width", config_.width);
    if (status == MV_OK) status = MV_CC_SetIntValue(handle_, "Height", config_.height);
    if (status != MV_OK) setError("MV_CC_SetIntValue Width/Height", status);
  }

  MV_CC_SetBayerCvtQuality(handle_, 1);
  return true;
}

bool HikCameraSource::updatePayloadBuffer() {
  MVCC_INTVALUE payload_size{};
  const int status = MV_CC_GetIntValue(handle_, "PayloadSize", &payload_size);
  if (status != MV_OK || payload_size.nCurValue == 0) {
    setError("MV_CC_GetIntValue PayloadSize", status);
    return false;
  }
  frame_buffer_.assign(payload_size.nCurValue, 0);
  return true;
}

bool HikCameraSource::read(CameraFrame& frame,
                           std::chrono::milliseconds timeout) {
  if (!isOpen() && !open()) return false;

  std::unique_lock<std::mutex> lock(frame_mutex_);
  const bool ready = frame_cv_.wait_for(lock, timeout, [&] {
    return has_frame_ || !running_.load();
  });
  if (!ready) {
    if (error_message_.empty()) {
      error_message_ = "timed out waiting for Hik camera frame";
    }
    return false;
  }
  if (!has_frame_) {
    error_message_ = "Hik camera capture thread stopped";
    return false;
  }

  frame = latest_frame_;
  has_frame_ = false;
  error_message_.clear();
  return true;
}

void HikCameraSource::captureLoop() {
  while (running_.load()) {
    CameraFrame frame;
    if (captureOnce(frame)) {
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = std::move(frame);
        has_frame_ = true;
        error_message_.clear();
      }
      frame_cv_.notify_one();
    } else {
      frame_cv_.notify_all();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

bool HikCameraSource::captureOnce(CameraFrame& frame) {
  if (!isOpen()) return false;

  MV_FRAME_OUT_INFO_EX frame_info{};
  const int status = MV_CC_GetOneFrameTimeout(
      handle_, frame_buffer_.data(), static_cast<unsigned int>(frame_buffer_.size()),
      &frame_info, 50);
  const auto timestamp = std::chrono::steady_clock::now();
  if (status != MV_OK) {
    setError("MV_CC_GetOneFrameTimeout", status);
    return false;
  }
  if (frame_info.nWidth == 0 || frame_info.nHeight == 0 || frame_info.nFrameLen == 0) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    error_message_ = "Hik returned an empty frame";
    return false;
  }

  const unsigned int need_size = frame_info.nWidth * frame_info.nHeight * 3;
  if (bgr_buffer_.size() < need_size) bgr_buffer_.assign(need_size, 0);

  MV_CC_PIXEL_CONVERT_PARAM convert{};
  convert.nWidth = frame_info.nWidth;
  convert.nHeight = frame_info.nHeight;
  convert.pSrcData = frame_buffer_.data();
  convert.nSrcDataLen = frame_info.nFrameLen;
  convert.enSrcPixelType = frame_info.enPixelType;
  convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  convert.pDstBuffer = bgr_buffer_.data();
  convert.nDstBufferSize = static_cast<unsigned int>(bgr_buffer_.size());

  const int convert_status = MV_CC_ConvertPixelType(handle_, &convert);
  if (convert_status != MV_OK) {
    setError("MV_CC_ConvertPixelType", convert_status);
    return false;
  }

  cv::Mat image(static_cast<int>(frame_info.nHeight),
                static_cast<int>(frame_info.nWidth), CV_8UC3,
                bgr_buffer_.data());
  if (config_.flip_image) {
    cv::flip(image, frame.image, -1);
  } else {
    frame.image = image.clone();
  }
  fillFrameMetadata(frame, timestamp);
  return true;
}

void HikCameraSource::fillFrameMetadata(
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
