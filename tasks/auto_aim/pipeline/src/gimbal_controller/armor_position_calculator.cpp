// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gimbal_controller/armor_position_calculator.hpp"

#include <rclcpp/rclcpp.hpp>

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace gimbal_controller
{
namespace
{

using Usage = fyt::auto_aim::robot_description::TrackedRobotUsage;
using ArmorWorldPose = fyt::auto_aim::robot_description::ArmorWorldPose;

rm_interfaces::msg::TrackedRobot withResolvedArmorPoses(
  const rm_interfaces::msg::TrackedRobot & robot)
{
  auto resolved = Usage::normalizeState(robot);
  if (!resolved.armors_offset.empty()) {
    return resolved;
  }

  if (Usage::isSingleArmorRepresentation(resolved)) {
    geometry_msgs::msg::Pose zero_offset;
    zero_offset.orientation.w = 1.0;
    resolved.armors_offset = {zero_offset};
  } else {
    resolved.armors_offset = Usage::generateArmorsOffsetFromProfile(
      resolved.num_armors,
      resolved.radius,
      resolved.radius_2,
      resolved.d_za,
      resolved.d_zc);
  }
  return resolved;
}

std::vector<Eigen::Vector3d> positionsFromPoses(
  const std::vector<ArmorWorldPose> & poses)
{
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(poses.size());
  for (const auto & pose : poses) {
    positions.push_back(pose.position);
  }
  return positions;
}

}  // namespace

std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculate(
  const rm_interfaces::msg::TrackedRobot & robot) const
{
  return positionsFromPoses(calculatePoses(robot));
}

std::vector<ArmorWorldPose> ArmorPositionCalculator::calculatePoses(
  const rm_interfaces::msg::TrackedRobot & robot) const
{
  const auto resolved_robot = withResolvedArmorPoses(robot);

  if (!robot.armors_offset.empty()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArmorPositionCalculator"),
      "Using armors_offset from TrackedRobot (size=%zu, num_armors=%d)",
      robot.armors_offset.size(), robot.num_armors);
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArmorPositionCalculator"),
      "armors_offset empty, using complete profile fallback (num_armors=%d)",
      robot.num_armors);
  }

  return Usage::calculateArmorWorldPosesEigen(
    resolved_robot,
    0.0,
    Usage::MotionModel::CONSTANT_VELOCITY);
}

std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculatePredicted(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt) const
{
  return positionsFromPoses(calculatePredictedPoses(robot, dt));
}

std::vector<ArmorWorldPose> ArmorPositionCalculator::calculatePredictedPoses(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt) const
{
  return Usage::calculateArmorWorldPosesEigen(
    withResolvedArmorPoses(robot),
    dt,
    Usage::MotionModel::CONSTANT_ACCELERATION);
}

std::vector<Eigen::Vector3d> ArmorPositionCalculator::generateDefaultOffsets(
  uint8_t robot_type,
  int num_armors,
  double radius,
  double radius_2,
  double d_za,
  double d_zc)
{
  if (robot_type == rm_interfaces::msg::TrackedRobot::OUTPOST_3 && num_armors == 3) {
    std::vector<Eigen::Vector3d> offsets;
    offsets.reserve(3);

    for (int i = 0; i < 3; ++i) {
      const double panel_angle = i * (2.0 * M_PI / 3.0);
      double dz = d_zc;
      if (i == 0) {
        dz = d_zc + d_za;
      } else if (i == 2) {
        dz = d_zc - d_za;
      }

      offsets.emplace_back(
        -radius * std::cos(panel_angle),
        -radius * std::sin(panel_angle),
        dz);
    }
    return offsets;
  }

  std::vector<Eigen::Vector3d> offsets;
  offsets.reserve(static_cast<size_t>(num_armors));

  for (int i = 0; i < num_armors; i++) {

    double panel_angle = i * (2.0 * M_PI / num_armors);

    double r = (i % 2 == 0) ? radius : radius_2;

    double dz = (i % 2 == 0) ? -d_za : d_za;

    Eigen::Vector3d offset(
        r * std::cos(panel_angle),
        r * std::sin(panel_angle),
        dz);

    offsets.push_back(offset);
  }

  return offsets;
}

}  // namespace gimbal_controller
