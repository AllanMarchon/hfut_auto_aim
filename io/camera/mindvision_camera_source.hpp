#ifndef HFUT_AUTO_AIM_MINDVISION_CAMERA_SOURCE_HPP
#define HFUT_AUTO_AIM_MINDVISION_CAMERA_SOURCE_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "io/camera/camera_source.hpp"

namespace hfut::io {

struct MindvisionCameraSourceConfig {
  std::string camera_sn;
  int width = 0;
  int height = 0;
  double fps = 0.0;
  double exposure_time_us = 0.0;
  int analog_gain = -1;
  int frame_speed = -1;
  bool flip_image = false;
  CameraIntrinsics intrinsics;
};

class MindvisionCameraSource final : public CameraSource {
 public:
  explicit MindvisionCameraSource(MindvisionCameraSourceConfig config = {});
  ~MindvisionCameraSource() override;

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
  void fillFrameMetadata(CameraFrame& frame);
  void setError(const std::string& action, int status);

  MindvisionCameraSourceConfig config_;
  int handle_ = -1;
  std::vector<std::uint8_t> rgb_buffer_;
  std::string error_message_;
  std::chrono::steady_clock::time_point start_time_;
  std::uint64_t seq_ = 0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_MINDVISION_CAMERA_SOURCE_HPP
