// ROS-free shim for rm_interfaces/msg/TrackedRobot. Field- and constant-
// compatible with the .msg so the ported tracker/controller compile unmodified.
#ifndef HFUT_COMPAT_RM_INTERFACES_TRACKED_ROBOT_HPP
#define HFUT_COMPAT_RM_INTERFACES_TRACKED_ROBOT_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace rm_interfaces {
namespace msg {

struct TrackedRobot {
  using SharedPtr = std::shared_ptr<TrackedRobot>;
  using ConstSharedPtr = std::shared_ptr<const TrackedRobot>;

  std_msgs::msg::Header header;
  std::string robot_id;

  // robot_type
  static constexpr uint8_t BALANCE_2 = 0;
  static constexpr uint8_t STANDARD_4 = 1;
  static constexpr uint8_t HERO_4 = 2;
  static constexpr uint8_t OUTPOST_3 = 3;
  static constexpr uint8_t SENTRY = 4;
  static constexpr uint8_t BASE = 5;
  static constexpr uint8_t UNKNOWN = 255;
  uint8_t robot_type = UNKNOWN;

  // track_state
  static constexpr uint8_t DETECTING = 0;
  static constexpr uint8_t TRACKING = 1;
  static constexpr uint8_t TEMP_LOST = 2;
  uint8_t track_state = DETECTING;

  // Full 6DoF kinematic state. Angular velocity/acceleration vectors are
  // expressed in header.frame_id (the world/target frame), not body-local axes.
  bool full_state_valid = false;
  geometry_msgs::msg::Pose center_pose;
  geometry_msgs::msg::Twist center_twist;
  geometry_msgs::msg::Accel center_accel;

  // Distinct from full_state_valid: this certifies that center_pose roll/pitch
  // describes the armor layout frame and may drive AUTO full-SE(3) geometry.
  bool layout_attitude_valid = false;

  // Legacy fields (kept in sync with the 6DoF state).
  geometry_msgs::msg::Point center_position;
  geometry_msgs::msg::Vector3 center_velocity;
  geometry_msgs::msg::Vector3 center_acceleration;

  double yaw = 0.0;
  double yaw_velocity = 0.0;
  double yaw_acceleration = 0.0;

  double radius = 0.0;
  double radius_2 = 0.0;
  double d_za = 0.0;
  double d_zc = 0.0;

  std::vector<geometry_msgs::msg::Pose> armors_offset;

  std::vector<double> state_covariance;
  uint8_t covariance_dim = 0;

  std::vector<std::string> bound_armor_ids;
  double confidence = 0.0;
  int32_t num_armors = 0;

  // representation_mode
  static constexpr uint8_t REP_STRUCTURED_ROBOT = 0;
  static constexpr uint8_t REP_AMBIGUOUS_SINGLE_ARMOR = 1;
  uint8_t representation_mode = REP_STRUCTURED_ROBOT;

  bool is_visible = false;
  int32_t visible_armor_count = 0;

  uint32_t engageable_mask = 0;
  int32_t engageable_count = 0;
};

}  // namespace msg
}  // namespace rm_interfaces

#endif  // HFUT_COMPAT_RM_INTERFACES_TRACKED_ROBOT_HPP
