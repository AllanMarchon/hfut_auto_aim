#ifndef HFUT_AUTO_AIM_OPENCV_CAMERA_SOURCE_HPP
#define HFUT_AUTO_AIM_OPENCV_CAMERA_SOURCE_HPP

#include <chrono>
#include <cstdint>
#include <string>

#include <opencv2/videoio.hpp>

#include "io/camera/camera_source.hpp"

namespace hfut::io {

struct OpenCvCameraSourceConfig {
  // Empty source opens device_index. Non-empty source opens a file, stream URL,
  // or backend-specific device path accepted by cv::VideoCapture.
  std::string source;
  int device_index = 0;
  int api_preference = cv::CAP_ANY;

  int width = 0;
  int height = 0;
  double fps = 0.0;
  double exposure = 0.0;
  double gain = 0.0;
  bool set_exposure = false;
  bool set_gain = false;

  CameraIntrinsics intrinsics;
};

class OpenCvCameraSource final : public CameraSource {
 public:
  explicit OpenCvCameraSource(OpenCvCameraSourceConfig config = {});
  ~OpenCvCameraSource() override;

  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool read(CameraFrame& frame,
            std::chrono::milliseconds timeout =
                std::chrono::milliseconds(200)) override;

  const std::string& errorMessage() const override { return error_message_; }

 private:
  void applyCaptureOptions();
  void fillFrameMetadata(CameraFrame& frame,
                         std::chrono::steady_clock::time_point timestamp);

  OpenCvCameraSourceConfig config_;
  cv::VideoCapture capture_;
  std::string error_message_;
  std::chrono::steady_clock::time_point start_time_;
  std::uint64_t seq_ = 0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_OPENCV_CAMERA_SOURCE_HPP
