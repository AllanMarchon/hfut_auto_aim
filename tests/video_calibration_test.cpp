#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "hfut_auto_aim/video_calibration.hpp"

namespace {

bool near(double lhs, double rhs) { return std::abs(lhs - rhs) < 1e-9; }

}  // namespace

int main() {
  hfut::video::CameraCalibration source;
  source.width = 1280;
  source.height = 1024;
  source.k = {1000.0, 0.0, 640.0, 0.0, 1000.0, 512.0, 0.0, 0.0, 1.0};

  const auto crop = hfut::video::adaptCalibration(
      source, 1440, 1080, hfut::video::CalibrationMode::center_crop);
  if (!near(crop.k[0], 1125.0) || !near(crop.k[4], 1125.0) ||
      !near(crop.k[2], 720.0) || !near(crop.k[5], 540.0) ||
      !near(crop.offset_y, -36.0)) {
    std::fprintf(stderr, "center_crop calibration is incorrect\n");
    return 1;
  }

  const auto scaled = hfut::video::adaptCalibration(
      source, 1440, 1080, hfut::video::CalibrationMode::scale);
  if (!near(scaled.k[0], 1125.0) || !near(scaled.k[4], 1054.6875) ||
      !near(scaled.k[2], 720.0) || !near(scaled.k[5], 540.0)) {
    std::fprintf(stderr, "scale calibration is incorrect\n");
    return 1;
  }

  bool strict_rejected = false;
  try {
    (void)hfut::video::adaptCalibration(
        source, 1440, 1080, hfut::video::CalibrationMode::strict);
  } catch (const std::invalid_argument&) {
    strict_rejected = true;
  }
  if (!strict_rejected) {
    std::fprintf(stderr, "strict mode accepted mismatched dimensions\n");
    return 1;
  }
  return 0;
}
