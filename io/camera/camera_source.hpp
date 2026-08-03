#ifndef HFUT_AUTO_AIM_CAMERA_SOURCE_HPP
#define HFUT_AUTO_AIM_CAMERA_SOURCE_HPP

#include <chrono>

#include "hfut_auto_aim/camera_frame.hpp"

namespace hfut::io {

class CameraSource {
 public:
  virtual ~CameraSource() = default;

  virtual bool read(
      CameraFrame& frame,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) = 0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_CAMERA_SOURCE_HPP
