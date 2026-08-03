#include <cmath>
#include <cstdio>

#include <Eigen/Geometry>

#include "hfut_auto_aim/camera_frame.hpp"

namespace {

bool near(double lhs, double rhs, double epsilon = 1e-9) {
  return std::abs(lhs - rhs) <= epsilon;
}

bool vectorNear(
    const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs,
    double epsilon = 1e-9) {
  return (lhs - rhs).norm() <= epsilon;
}

}  // namespace

int main() {
  hfut::CameraToBarrelExtrinsics extrinsics;
  extrinsics.xyz = Eigen::Vector3d(0.12, -0.03, 0.05);
  extrinsics.rpy = Eigen::Vector3d(0.02, -0.04, 0.06);

  const double yaw = 0.7;
  const double pitch = -0.25;
  const Eigen::Matrix3d barrel_to_control =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();

  hfut::CameraFrame frame;
  const Eigen::Matrix3d camera_to_control =
      barrel_to_control * extrinsics.cameraOpticalToBarrelRotation();
  frame.q_cam2world = Eigen::Quaterniond(camera_to_control);

  const bool applied = hfut::applyCameraToBarrelExtrinsics(frame, extrinsics);
  const Eigen::Vector3d expected_translation = barrel_to_control * extrinsics.xyz;
  if (!applied || !vectorNear(frame.t_cam2world, expected_translation) ||
      !near(frame.gimbal_yaw, yaw) || !near(frame.gimbal_pitch, pitch) ||
      (frame.R_cam2world() - camera_to_control).norm() > 1e-9) {
    std::fprintf(stderr,
                 "camera extrinsics test failed: applied=%d yaw=%.12f pitch=%.12f "
                 "translation=[%.12f %.12f %.12f]\n",
                 applied, frame.gimbal_yaw, frame.gimbal_pitch,
                 frame.t_cam2world.x(), frame.t_cam2world.y(), frame.t_cam2world.z());
    return 1;
  }

  hfut::CameraFrame invalid;
  invalid.q_cam2world.coeffs().setZero();
  if (hfut::applyCameraToBarrelExtrinsics(invalid, extrinsics)) {
    std::fprintf(stderr, "invalid camera quaternion was accepted\n");
    return 2;
  }
  return 0;
}
