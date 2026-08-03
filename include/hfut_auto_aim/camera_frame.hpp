// IO-facing data types for the ros-free auto_aim program. These replace the
// sensor_msgs / geometry_msgs structs the ROS pipeline passed around at its
// boundary. Algorithm-domain types (Armor, TrackedRobot, ...) live in their
// own headers ported alongside the detector/tracker in later phases.
#ifndef HFUT_AUTO_AIM_CAMERA_FRAME_HPP
#define HFUT_AUTO_AIM_CAMERA_FRAME_HPP

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace hfut {

enum class FrameInputMode { vision, armor_pose };

struct DirectArmorPose {
  std::string number;
  std::string type;
  Eigen::Vector3d position_control = Eigen::Vector3d::Zero();
  double radial_yaw{0.0};
  double confidence{1.0};
  double position_noise_std_m{0.0};
  double yaw_noise_std_rad{0.0};
  double view_angle_rad{0.0};
  Eigen::Quaterniond surface_orientation_control =
      Eigen::Quaterniond::Identity();
  bool surface_orientation_valid{false};
};

// Pinhole intrinsics + plumb_bob distortion. Replaces sensor_msgs::CameraInfo
// at the algorithm boundary (e.g. ArmorPoseEstimator ctor).
struct CameraIntrinsics {
  int width = 0;
  int height = 0;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  double distortion[5] = {0, 0, 0, 0, 0};

  // 3x3 camera matrix K as a cv::Mat (CV_64F), built on demand.
  cv::Mat cameraMatrix() const {
    return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }
  cv::Mat distCoeffs() const {
    return (cv::Mat_<double>(1, 5) << distortion[0], distortion[1],
            distortion[2], distortion[3], distortion[4]);
  }
};

// Calibrated camera pose relative to the barrel frame. Both frames use the
// ROS convention X-forward, Y-left, Z-up before the camera optical rotation is
// applied. xyz is in meters; rpy is fixed-axis roll/pitch/yaw in radians.
struct CameraToBarrelExtrinsics {
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();

  Eigen::Matrix3d cameraOpticalToBarrelRotation() const {
    const Eigen::Matrix3d camera_link_to_barrel =
        (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    Eigen::Matrix3d optical_to_camera_link;
    optical_to_camera_link << 0, 0, 1,
                             -1, 0, 0,
                              0, -1, 0;
    return camera_link_to_barrel * optical_to_camera_link;
  }
};

// One camera frame plus everything the pipeline needs that the ROS version got
// from /camera_info, /joint_states and the tf2 tree.
struct CameraFrame {
  FrameInputMode input_mode = FrameInputMode::vision;
  uint64_t seq = 0;
  double sim_time_s = 0.0;

  cv::Mat image;  // BGR8, decoded from the wire encoding
  std::vector<DirectArmorPose> direct_armors;
  double direct_position_noise_std_m = 0.0;
  double direct_yaw_noise_std_rad = 0.0;
  CameraIntrinsics intrinsics;

  // Gimbal feedback (was /joint_states).
  double gimbal_yaw = 0.0;
  double gimbal_pitch = 0.0;
  double gimbal_yaw_vel = 0.0;
  double gimbal_pitch_vel = 0.0;

  // camera_optical_frame -> shooter-centered, odom-aligned control transform.
  Eigen::Quaterniond q_cam2world = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_cam2world = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_cam2world() const { return q_cam2world.toRotationMatrix(); }
};

// Make the YAML extrinsics authoritative for the camera origin and recover
// barrel feedback from the exact per-frame camera orientation. Direct-pose
// input still uses this transform for gimbal feedback and debug projection;
// its already-control-frame armor measurements remain unchanged.
inline bool applyCameraToBarrelExtrinsics(
    CameraFrame& frame, const CameraToBarrelExtrinsics& extrinsics) {
  if (!extrinsics.xyz.allFinite() || !extrinsics.rpy.allFinite() ||
      !std::isfinite(frame.q_cam2world.norm()) ||
      frame.q_cam2world.norm() <= 1e-12) {
    return false;
  }

  frame.q_cam2world.normalize();
  const Eigen::Matrix3d camera_to_control = frame.R_cam2world();
  const Eigen::Matrix3d camera_to_barrel =
      extrinsics.cameraOpticalToBarrelRotation();
  const Eigen::Matrix3d barrel_to_control =
      camera_to_control * camera_to_barrel.transpose();

  frame.t_cam2world = barrel_to_control * extrinsics.xyz;
  frame.gimbal_yaw = std::atan2(barrel_to_control(1, 0), barrel_to_control(0, 0));
  frame.gimbal_pitch = std::atan2(
      barrel_to_control(2, 0),
      std::hypot(barrel_to_control(0, 0), barrel_to_control(1, 0)));
  return frame.t_cam2world.allFinite() && std::isfinite(frame.gimbal_yaw) &&
         std::isfinite(frame.gimbal_pitch);
}

}  // namespace hfut

#endif  // HFUT_AUTO_AIM_CAMERA_FRAME_HPP
