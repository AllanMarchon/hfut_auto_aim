#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  const double command_yaw_delta = std::abs(tools::limit_rad(last_command_.yaw - command.yaw));
  const double command_pitch_delta = std::abs(last_command_.pitch - command.pitch);
  const double gimbal_yaw_error = std::abs(tools::limit_rad(gimbal_pos[0] - last_command_.yaw));
  const double gimbal_pitch_error = std::abs(gimbal_pos[1] - last_command_.pitch);

  if (
    command_yaw_delta < tolerance * 2 &&    // command 突变时不应该射击
    command_pitch_delta < tolerance * 2 &&
    gimbal_yaw_error < tolerance &&         // 云台实际角度需要追到上一帧瞄准角
    gimbal_pitch_error < tolerance &&
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim
