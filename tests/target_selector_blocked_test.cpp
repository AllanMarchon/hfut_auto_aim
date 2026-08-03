// Block-list coverage for the target selector.
//
// priority_robot_ids is ordering-only, so blocking must happen inside
// filterCandidates: a blocked id must never be selected, neither via the
// priority list nor via the min-yaw-deviation fallback.

#include "target_selector/strategies/priority_list_strategy.hpp"

#include <cstdio>

namespace {

using fyt::auto_aim::SelectionConfig;
using fyt::auto_aim::PriorityListStrategy;

rm_interfaces::msg::TrackedRobot makeRobot(const std::string& id, double x) {
  rm_interfaces::msg::TrackedRobot robot;
  robot.robot_id = id;
  robot.confidence = 1.0;
  robot.center_position.x = x;
  robot.center_position.y = 0.0;
  robot.center_position.z = 0.0;
  return robot;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "%s\n", message);
  return code;
}

}  // namespace

int main() {
  rm_interfaces::msg::TrackedRobots robots;
  robots.robots.push_back(makeRobot("outpost", 2.0));
  robots.robots.push_back(makeRobot("1", 3.0));

  SelectionConfig config;
  config.min_confidence = 0.15;
  config.max_distance = 10.0;
  config.max_yaw_deviation = 3.14159;
  config.reference_yaw = 0.0;
  config.priority_robot_ids = {"outpost", "1"};

  PriorityListStrategy strategy;

  // Without a block list the priority order wins.
  const auto unblocked = strategy.selectTarget(robots, config);
  if (!unblocked || unblocked->robot_id != "outpost") {
    return fail("priority order broken without block list", 1);
  }

  // Blocking outpost: robot "1" must be selected even though outpost has
  // higher priority and better yaw deviation.
  config.blocked_robot_ids = {"outpost"};
  const auto blocked = strategy.selectTarget(robots, config);
  if (!blocked || blocked->robot_id != "1") {
    return fail("blocked robot was still selected", 2);
  }

  // Blocking everything: no selection at all.
  config.blocked_robot_ids = {"outpost", "1"};
  const auto all_blocked = strategy.selectTarget(robots, config);
  if (all_blocked.has_value()) {
    return fail("selection succeeded with every robot blocked", 3);
  }
  return 0;
}
