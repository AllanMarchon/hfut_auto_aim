#ifndef HFUT_AUTO_AIM_HIK_CAMERA_SOURCE_HPP
#define HFUT_AUTO_AIM_HIK_CAMERA_SOURCE_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "io/camera/camera_source.hpp"

namespace hfut::io {

struct HikCameraSourceConfig {
  std::string camera_sn;
  int width = 0;
  int height = 0;
  double fps = 0.0;
  double exposure_time_us = 0.0;
  double gain = 0.0;
  bool flip_image = false;
  CameraIntrinsics intrinsics;
};

class HikCameraSource final : public CameraSource {
 public:
  explicit HikCameraSource(HikCameraSourceConfig config = {});
  ~HikCameraSource() override;

  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool read(CameraFrame& frame,
            std::chrono::milliseconds timeout =
                std::chrono::milliseconds(200)) override;

  const std::string& errorMessage() const override { return error_message_; }

 private:
  bool selectAndOpenDevice();
  bool applyOptions();
  bool updatePayloadBuffer();
  void fillFrameMetadata(CameraFrame& frame);
  void setError(const std::string& action, int status);

  HikCameraSourceConfig config_;
  void* handle_ = nullptr;
  std::vector<std::uint8_t> frame_buffer_;
  std::vector<std::uint8_t> bgr_buffer_;
  std::string error_message_;
  std::chrono::steady_clock::time_point start_time_;
  std::uint64_t seq_ = 0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_HIK_CAMERA_SOURCE_HPP
