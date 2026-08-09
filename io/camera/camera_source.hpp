#ifndef HFUT_AUTO_AIM_CAMERA_SOURCE_HPP
#define HFUT_AUTO_AIM_CAMERA_SOURCE_HPP

#include <chrono>
#include <string>

#include "hfut_auto_aim/camera_frame.hpp"

namespace hfut::io {

class CameraSource {
 public:
  virtual ~CameraSource() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  virtual bool read(
      CameraFrame& frame,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) = 0;

  virtual const std::string& errorMessage() const = 0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_CAMERA_SOURCE_HPP
