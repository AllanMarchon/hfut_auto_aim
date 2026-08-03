// ROS-free shim for angles/angles.h. The ported controller uses only
// normalize_angle and shortest_angular_distance.
#ifndef HFUT_COMPAT_ANGLES_ANGLES_H
#define HFUT_COMPAT_ANGLES_ANGLES_H

#include <cmath>

namespace angles {

// Wrap to [-pi, pi].
inline double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Wrap to [0, 2pi).
inline double normalize_angle_positive(double angle) {
  const double two_pi = 2.0 * M_PI;
  double r = std::fmod(angle, two_pi);
  if (r < 0.0) r += two_pi;
  return r;
}

// Shortest signed difference (to - from) wrapped to [-pi, pi].
inline double shortest_angular_distance(double from, double to) {
  return normalize_angle(to - from);
}

inline double from_degrees(double deg) { return deg * M_PI / 180.0; }
inline double to_degrees(double rad) { return rad * 180.0 / M_PI; }

}  // namespace angles

#endif  // HFUT_COMPAT_ANGLES_ANGLES_H
