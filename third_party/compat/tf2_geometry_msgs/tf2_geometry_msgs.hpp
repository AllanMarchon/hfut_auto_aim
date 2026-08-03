// ROS-free shim for tf2_geometry_msgs/tf2_geometry_msgs.hpp.
// Provides the tf2 <-> geometry_msgs Quaternion conversions the ported
// tracker/controller use (tf2::toMsg / tf2::fromMsg). tf2::Quaternion comes
// from the vendored LinearMath headers.
#ifndef HFUT_COMPAT_TF2_GEOMETRY_MSGS_HPP
#define HFUT_COMPAT_TF2_GEOMETRY_MSGS_HPP

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/quaternion.hpp>

namespace tf2 {

inline geometry_msgs::msg::Quaternion toMsg(const tf2::Quaternion& in) {
  geometry_msgs::msg::Quaternion out;
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
  out.w = in.w();
  return out;
}

inline void fromMsg(const geometry_msgs::msg::Quaternion& in, tf2::Quaternion& out) {
  out.setValue(in.x, in.y, in.z, in.w);
}

}  // namespace tf2

#endif  // HFUT_COMPAT_TF2_GEOMETRY_MSGS_HPP
