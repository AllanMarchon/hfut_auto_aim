// ROS-free shim for rm_interfaces/msg/Armor. Field-compatible with the .msg so
// msg_converter.hpp (detector Armor -> tracker ObservationData) compiles
// unmodified. The Pipeline fills these from the detector's PoseEstimate.
#ifndef HFUT_COMPAT_RM_INTERFACES_ARMOR_HPP
#define HFUT_COMPAT_RM_INTERFACES_ARMOR_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace rm_interfaces {
namespace msg {

struct Armor {
  using SharedPtr = std::shared_ptr<Armor>;

  std::string number;
  std::string type;
  float distance_to_image_center = 0.0f;
  geometry_msgs::msg::Pose pose;

  // 2D image-domain metadata.
  float detection_confidence = 0.0f;
  int32_t track_id = -1;
  bool has_image_geometry = false;
  std::array<float, 4> bbox_xywh{};
  std::array<geometry_msgs::msg::Point32, 4> image_corners{};
  uint8_t corners_ordering = 0;

  // Pose / refiner quality metadata.
  uint8_t pose_estimate_mode = 0;
  float pose_quality_score = 0.0f;
  float reproj_error_raw = 0.0f;
  float reproj_error_refined = 0.0f;
  float pose_condition_number = 0.0f;
  uint16_t pose_num_points = 0;
  uint16_t pose_num_inliers = 0;

  // True only when the producer supplies a calibrated complete plate frame,
  // rather than a yaw-only or unconstrained planar-PnP orientation.
  bool pose_orientation_trusted = false;
  bool radial_yaw_valid = false;
  double radial_yaw = 0.0;  // header/control-frame center-to-armor yaw

  bool pose_covariance_valid = false;
  std::array<double, 16> pose_covariance_xyz_yaw{};

  // Equivalent 1-sigma yaw uncertainty (rad) from the IPPE twisted-pair
  // ambiguity; 0 when the minimum-error branch is clearly best or not IPPE.
  float ippe_yaw_ambiguity = 0.0f;
};

}  // namespace msg
}  // namespace rm_interfaces

#endif  // HFUT_COMPAT_RM_INTERFACES_ARMOR_HPP
