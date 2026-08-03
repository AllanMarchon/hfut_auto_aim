#ifndef HFUT_AUTO_AIM_VIDEO_CALIBRATION_HPP_
#define HFUT_AUTO_AIM_VIDEO_CALIBRATION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace hfut::video {

enum class CalibrationMode { strict, scale, center_crop };

inline CalibrationMode parseCalibrationMode(const std::string& value) {
  if (value == "strict") return CalibrationMode::strict;
  if (value == "scale") return CalibrationMode::scale;
  if (value == "center_crop" || value == "crop" || value == "cover") {
    return CalibrationMode::center_crop;
  }
  throw std::invalid_argument(
      "calibration mode must be strict, scale, or center_crop, got: " + value);
}

inline const char* calibrationModeName(CalibrationMode mode) {
  switch (mode) {
    case CalibrationMode::strict: return "strict";
    case CalibrationMode::scale: return "scale";
    case CalibrationMode::center_crop: return "center_crop";
  }
  return "unknown";
}

struct CameraCalibration {
  int width{0};
  int height{0};
  std::array<double, 9> k{1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0};
  std::vector<double> d;
  double scale_x{1.0};
  double scale_y{1.0};
  double offset_x{0.0};
  double offset_y{0.0};
};

inline CameraCalibration adaptCalibration(
    const CameraCalibration& source, int target_width, int target_height,
    CalibrationMode mode) {
  if (source.width <= 0 || source.height <= 0 || target_width <= 0 ||
      target_height <= 0) {
    throw std::invalid_argument("calibration and video dimensions must be positive");
  }
  for (double value : source.k) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("camera matrix contains a non-finite value");
    }
  }
  if (source.k[0] <= 0.0 || source.k[4] <= 0.0) {
    throw std::invalid_argument("camera focal lengths must be positive");
  }

  CameraCalibration result = source;
  result.width = target_width;
  result.height = target_height;

  if (mode == CalibrationMode::strict) {
    if (source.width != target_width || source.height != target_height) {
      throw std::invalid_argument(
          "strict calibration mode rejects the camera/video resolution mismatch");
    }
    return result;
  }

  double sx = static_cast<double>(target_width) / source.width;
  double sy = static_cast<double>(target_height) / source.height;
  double ox = 0.0;
  double oy = 0.0;
  if (mode == CalibrationMode::center_crop) {
    const double uniform_scale = std::max(sx, sy);
    sx = uniform_scale;
    sy = uniform_scale;
    ox = 0.5 * (target_width - source.width * uniform_scale);
    oy = 0.5 * (target_height - source.height * uniform_scale);
  }

  result.k[0] = source.k[0] * sx;
  result.k[1] = source.k[1] * sx;
  result.k[2] = source.k[2] * sx + ox;
  result.k[3] = source.k[3] * sy;
  result.k[4] = source.k[4] * sy;
  result.k[5] = source.k[5] * sy + oy;
  result.scale_x = sx;
  result.scale_y = sy;
  result.offset_x = ox;
  result.offset_y = oy;
  return result;
}

}  // namespace hfut::video

#endif  // HFUT_AUTO_AIM_VIDEO_CALIBRATION_HPP_
