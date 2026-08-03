// ROS-free shim for sensor_msgs::msg::CameraInfo.
//
// The ported pose-estimator adapter reads only the K matrix (.k, 9 elements)
// and distortion coefficients (.d). This stand-in mirrors those two fields so
// the algorithm source compiles unmodified. Populate from hfut::CameraIntrinsics
// at the call site.
#ifndef HFUT_COMPAT_SENSOR_MSGS_CAMERA_INFO_HPP
#define HFUT_COMPAT_SENSOR_MSGS_CAMERA_INFO_HPP

#include <array>
#include <cstdint>
#include <vector>

namespace sensor_msgs {
namespace msg {

struct CameraInfo {
  uint32_t width = 0;
  uint32_t height = 0;
  std::array<double, 9> k{};   // row-major 3x3 intrinsics
  std::vector<double> d;       // distortion coefficients
};

}  // namespace msg
}  // namespace sensor_msgs

#endif  // HFUT_COMPAT_SENSOR_MSGS_CAMERA_INFO_HPP
