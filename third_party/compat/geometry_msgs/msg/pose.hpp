// ROS-free shims for the geometry_msgs types used as plain data carriers by the
// ported tracker/selector/controller. Field-compatible with the ROS structs so
// the algorithm sources compile unmodified.
#ifndef HFUT_COMPAT_GEOMETRY_MSGS_HPP
#define HFUT_COMPAT_GEOMETRY_MSGS_HPP

namespace geometry_msgs {
namespace msg {

struct Point {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct Point32 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct Vector3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct Quaternion {
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
};

struct Pose {
  Point position;
  Quaternion orientation;
};

struct PoseStamped {
  Pose pose;
};

struct Twist {
  Vector3 linear;
  Vector3 angular;
};

struct Accel {
  Vector3 linear;
  Vector3 angular;
};

}  // namespace msg
}  // namespace geometry_msgs

#endif  // HFUT_COMPAT_GEOMETRY_MSGS_HPP
