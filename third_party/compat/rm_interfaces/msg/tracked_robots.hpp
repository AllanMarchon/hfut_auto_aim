// ROS-free shim for rm_interfaces/msg/TrackedRobots.
#ifndef HFUT_COMPAT_RM_INTERFACES_TRACKED_ROBOTS_HPP
#define HFUT_COMPAT_RM_INTERFACES_TRACKED_ROBOTS_HPP

#include <memory>
#include <vector>

#include <std_msgs/msg/header.hpp>

#include "rm_interfaces/msg/tracked_robot.hpp"

namespace rm_interfaces {
namespace msg {

struct TrackedRobots {
  using SharedPtr = std::shared_ptr<TrackedRobots>;
  using ConstSharedPtr = std::shared_ptr<const TrackedRobots>;

  std_msgs::msg::Header header;
  std::vector<TrackedRobot> robots;
};

}  // namespace msg
}  // namespace rm_interfaces

#endif  // HFUT_COMPAT_RM_INTERFACES_TRACKED_ROBOTS_HPP
