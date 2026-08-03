#include <cmath>
#include <cstdio>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"
#include "max_entropy_tracker/utils/body_attitude_estimator.hpp"

namespace {

using Usage = fyt::auto_aim::robot_description::TrackedRobotUsage;

constexpr double kTolerance = 1e-9;

bool near(double lhs, double rhs, double tolerance = kTolerance) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool nearVector(
    const Eigen::Vector3d& lhs,
    const Eigen::Vector3d& rhs,
    double tolerance = kTolerance) {
  return (lhs - rhs).norm() <= tolerance;
}

void assignQuaternion(
    geometry_msgs::msg::Quaternion& output,
    const Eigen::Quaterniond& input) {
  output.x = input.x();
  output.y = input.y();
  output.z = input.z();
  output.w = input.w();
}

Eigen::Quaterniond readQuaternion(const geometry_msgs::msg::Quaternion& input) {
  return Eigen::Quaterniond(input.w, input.x, input.y, input.z).normalized();
}

rm_interfaces::msg::TrackedRobot makeRobot(double roll, double pitch, double yaw) {
  rm_interfaces::msg::TrackedRobot robot;
  robot.robot_id = "4";
  robot.robot_type = rm_interfaces::msg::TrackedRobot::STANDARD_4;
  robot.representation_mode =
      rm_interfaces::msg::TrackedRobot::REP_STRUCTURED_ROBOT;
  robot.num_armors = 4;
  robot.radius = 0.20;
  robot.radius_2 = 0.17;
  robot.d_za = 0.03;
  robot.center_position.x = 1.2;
  robot.center_position.y = -0.4;
  robot.center_position.z = 0.8;
  robot.center_pose.position = robot.center_position;
  robot.yaw = yaw;
  robot.yaw_velocity = 0.7;
  robot.center_twist.angular.z = robot.yaw_velocity;
  const Eigen::Quaterniond orientation =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  assignQuaternion(robot.center_pose.orientation, orientation);
  robot.full_state_valid = true;
  robot.armors_offset = Usage::generateArmorsOffsetFromProfile(
      4, robot.radius, robot.radius_2, robot.d_za, 0.0);
  return robot;
}

int fail(const char* message) {
  std::fprintf(stderr, "armor geometry 3d test failed: %s\n", message);
  return 1;
}

}  // namespace

int main() {
  constexpr double kRoll = 8.0 * M_PI / 180.0;
  constexpr double kPitch = -11.0 * M_PI / 180.0;
  constexpr double kYaw = 0.4;
  constexpr double kMountPitch = 15.0 * M_PI / 180.0;

  auto robot = makeRobot(kRoll, kPitch, kYaw);
  if (robot.armors_offset.size() != 4) return fail("wrong armor count");

  for (size_t i = 0; i < robot.armors_offset.size(); ++i) {
    const auto& offset = robot.armors_offset[i];
    const double expected_radius = (i % 2 == 0) ? 0.20 : 0.17;
    const double expected_z = (i % 2 == 0) ? -0.03 : 0.03;
    if (!near(std::hypot(offset.position.x, offset.position.y), expected_radius) ||
        !near(offset.position.z, expected_z)) {
      return fail("r1/r2/dza opposite-pair layout changed");
    }
  }

  const Eigen::Quaterniond q_center =
      readQuaternion(robot.center_pose.orientation);
  const auto full_poses = Usage::calculateArmorWorldPosesEigen(
      robot,
      0.0,
      Usage::MotionModel::CONSTANT_VELOCITY,
      Usage::ProjectionMode::FULL_SE3);
  if (full_poses.size() != robot.armors_offset.size()) {
    return fail("full SE3 pose count mismatch");
  }

  const Eigen::Vector3d center(
      robot.center_position.x,
      robot.center_position.y,
      robot.center_position.z);
  for (size_t i = 0; i < full_poses.size(); ++i) {
    const auto& offset = robot.armors_offset[i];
    const Eigen::Vector3d local_offset(
        offset.position.x, offset.position.y, offset.position.z);
    const Eigen::Quaterniond q_offset = readQuaternion(offset.orientation);
    const Eigen::Quaterniond q_armor = q_center * q_offset;
    if (!nearVector(full_poses[i].position, center + q_center * local_offset) ||
        !nearVector(full_poses[i].normal, q_armor * Eigen::Vector3d::UnitX()) ||
        !nearVector(full_poses[i].width_axis, q_armor * Eigen::Vector3d::UnitY()) ||
        !nearVector(full_poses[i].height_axis, q_armor * Eigen::Vector3d::UnitZ()) ||
        !full_poses[i].surface_orientation_valid) {
      return fail("full SE3 did not compose center and plate poses");
    }
  }

  const Eigen::Quaterniond first_offset =
      readQuaternion(robot.armors_offset.front().orientation);
  const Eigen::Vector3d local_normal =
      first_offset * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d local_height =
      first_offset * Eigen::Vector3d::UnitZ();
  if (!near(local_normal.z(), std::sin(kMountPitch), 2e-5) ||
      !near(local_height.z(), std::cos(kMountPitch), 2e-5)) {
    return fail("nominal Webots-style 15 degree plate pitch was lost");
  }

  const auto yaw_plane_poses = Usage::calculateArmorWorldPosesEigen(
      robot,
      0.0,
      Usage::MotionModel::CONSTANT_VELOCITY,
      Usage::ProjectionMode::YAW_PLANE);
  const auto auto_poses = Usage::calculateArmorWorldPosesEigen(
      robot, 0.0, Usage::MotionModel::CONSTANT_VELOCITY);
  const Eigen::Quaterniond q_yaw(
      Eigen::AngleAxisd(kYaw, Eigen::Vector3d::UnitZ()));
  for (size_t i = 0; i < yaw_plane_poses.size(); ++i) {
    const auto& offset = robot.armors_offset[i];
    const Eigen::Vector3d local_offset(
        offset.position.x, offset.position.y, offset.position.z);
    if (!nearVector(yaw_plane_poses[i].position, center + q_yaw * local_offset)) {
      return fail("explicit yaw-plane fallback changed");
    }
    if (nearVector(yaw_plane_poses[i].height_axis, Eigen::Vector3d::UnitZ(), 1e-3)) {
      return fail("yaw-plane visualization forced the plate face vertical");
    }
    if (!nearVector(auto_poses[i].position, yaw_plane_poses[i].position) ||
        !nearVector(auto_poses[i].normal, yaw_plane_poses[i].normal) ||
        !nearVector(auto_poses[i].height_axis, yaw_plane_poses[i].height_axis)) {
      return fail("AUTO no longer preserves the configured Z-dominant default");
    }
  }

  auto trusted_robot = robot;
  trusted_robot.layout_attitude_valid = true;
  const auto trusted_auto_poses = Usage::calculateArmorWorldPosesEigen(
      trusted_robot, 0.0, Usage::MotionModel::CONSTANT_VELOCITY);
  for (size_t i = 0; i < trusted_auto_poses.size(); ++i) {
    if (!nearVector(trusted_auto_poses[i].position, full_poses[i].position) ||
        !nearVector(trusted_auto_poses[i].normal, full_poses[i].normal) ||
        !nearVector(trusted_auto_poses[i].height_axis, full_poses[i].height_axis)) {
      return fail("trusted layout attitude did not activate AUTO full SE3");
    }
  }

  const auto predicted_zero =
      Usage::predict(robot, 0.0, Usage::MotionModel::CONSTANT_VELOCITY);
  const Eigen::Quaterniond q_zero =
      readQuaternion(predicted_zero.center_pose.orientation);
  if (!near(std::abs(q_zero.dot(q_center)), 1.0)) {
    return fail("zero-time prediction erased roll/pitch");
  }

  constexpr double kDt = 0.4;
  const Eigen::Vector3d omega(
      robot.center_twist.angular.x,
      robot.center_twist.angular.y,
      robot.center_twist.angular.z);
  const double rotation_angle = omega.norm() * kDt;
  const Eigen::Quaterniond q_expected =
      Eigen::AngleAxisd(rotation_angle, omega.normalized()) * q_center;
  const auto predicted =
      Usage::predict(robot, kDt, Usage::MotionModel::CONSTANT_VELOCITY);
  const Eigen::Quaterniond q_predicted =
      readQuaternion(predicted.center_pose.orientation);
  if (!near(std::abs(q_predicted.dot(q_expected)), 1.0)) {
    return fail("prediction did not preserve the tilted layout");
  }

  const auto predicted_poses = Usage::calculateArmorWorldPosesEigen(
      robot,
      kDt,
      Usage::MotionModel::CONSTANT_VELOCITY,
      Usage::ProjectionMode::FULL_SE3);
  const Eigen::Vector3d first_local(
      robot.armors_offset.front().position.x,
      robot.armors_offset.front().position.y,
      robot.armors_offset.front().position.z);
  if (!nearVector(predicted_poses.front().position, center + q_expected * first_local)) {
    return fail("tilted armor position prediction is incorrect");
  }

  auto tilted_axis_robot = robot;
  const Eigen::Vector3d tilted_axis =
      Eigen::Vector3d(0.12, -0.08, 1.0).normalized();
  const Eigen::Vector3d tilted_omega = tilted_axis * 0.7;
  tilted_axis_robot.center_twist.angular.x = tilted_omega.x();
  tilted_axis_robot.center_twist.angular.y = tilted_omega.y();
  tilted_axis_robot.center_twist.angular.z = tilted_omega.z();
  const auto tilted_axis_prediction = Usage::predict(
      tilted_axis_robot, kDt, Usage::MotionModel::CONSTANT_VELOCITY);
  const Eigen::Quaterniond q_tilted_axis_expected =
      Eigen::AngleAxisd(tilted_omega.norm() * kDt, tilted_axis) * q_center;
  const Eigen::Quaterniond q_tilted_axis_actual =
      readQuaternion(tilted_axis_prediction.center_pose.orientation);
  if (!near(std::abs(q_tilted_axis_actual.dot(q_tilted_axis_expected)), 1.0)) {
    return fail("full angular vector was replaced by a hard-coded Z axis");
  }

  auto planar_robot = makeRobot(0.0, 0.0, kYaw);
  const auto planar_auto = Usage::calculateArmorWorldPosesEigen(
      planar_robot, 0.0, Usage::MotionModel::CONSTANT_VELOCITY);
  const auto planar_yaw = Usage::calculateArmorWorldPosesEigen(
      planar_robot,
      0.0,
      Usage::MotionModel::CONSTANT_VELOCITY,
      Usage::ProjectionMode::YAW_PLANE);
  for (size_t i = 0; i < planar_auto.size(); ++i) {
    if (!nearVector(planar_auto[i].position, planar_yaw[i].position) ||
        !nearVector(planar_auto[i].normal, planar_yaw[i].normal) ||
        !nearVector(planar_auto[i].height_axis, planar_yaw[i].height_axis)) {
      return fail("Z-dominant planar AUTO behavior changed");
    }
  }

  auto fallback_robot = planar_robot;
  fallback_robot.armors_offset.clear();
  gimbal_controller::ArmorPositionCalculator calculator;
  const auto fallback_poses = calculator.calculatePoses(fallback_robot);
  if (fallback_poses.size() != 4) {
    return fail("complete fallback did not produce four armor poses");
  }
  for (const auto& pose : fallback_poses) {
    if (!pose.surface_orientation_valid ||
        nearVector(pose.height_axis, Eigen::Vector3d::UnitZ(), 1e-3)) {
      return fail("fallback geometry lost the nominal plate surface pitch");
    }
  }

  const Eigen::Quaterniond body_orientation =
      Eigen::AngleAxisd(kYaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(kPitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(kRoll, Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond plate_orientation = body_orientation * first_offset;
  fyt::auto_aim::BodyAttitudeEstimator attitude_estimator(kMountPitch, 1.0);
  attitude_estimator.addPlateOrientation(
      plate_orientation.toRotationMatrix(), true);
  const auto attitude = attitude_estimator.estimate(kYaw);
  if (!attitude.valid || !attitude.trusted_for_geometry ||
      !nearVector(
          attitude.up, body_orientation * Eigen::Vector3d::UnitZ(), 2e-5) ||
      !nearVector(
          attitude.orientation * Eigen::Vector3d::UnitZ(),
          body_orientation * Eigen::Vector3d::UnitZ(),
          2e-5)) {
    return fail("body attitude reconstruction is not exact or lost trust");
  }

  return 0;
}
