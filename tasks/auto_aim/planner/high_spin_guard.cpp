#include "high_spin_guard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{

void HighSpinGuard::configure(const HighSpinGuardConfig & config)
{
  if (
    !std::isfinite(config.enter_speed_rad_s) ||
    config.enter_speed_rad_s <= 0.0) {
    throw std::invalid_argument(
      "high_spin_guard.enter_speed_rad_s must be positive");
  }

  if (
    !std::isfinite(config.exit_speed_rad_s) ||
    config.exit_speed_rad_s < 0.0 ||
    config.exit_speed_rad_s > config.enter_speed_rad_s) {
    throw std::invalid_argument(
      "high_spin_guard.exit_speed_rad_s must be non-negative "
      "and no greater than enter_speed_rad_s");
  }

  if (
    !std::isfinite(config.phase_tolerance_rad) ||
    config.phase_tolerance_rad <= 0.0 ||
    config.phase_tolerance_rad >= CV_PI) {
    throw std::invalid_argument(
      "high_spin_guard.phase_tolerance_rad must be in (0, pi)");
  }

  if (
    !std::isfinite(config.aligned_hold_s) ||
    config.aligned_hold_s < 0.0) {
    throw std::invalid_argument(
      "high_spin_guard.aligned_hold_s must be non-negative");
  }

  config_ = config;
  reset();
}

void HighSpinGuard::reset()
{
  active_ = false;
  target_name_ = not_armor;
  guard_radius_m_ = 0.0;
  guard_height_offset_m_ = 0.0;
  eligible_armor_indices_.clear();
}

bool HighSpinGuard::active() const
{
  return active_;
}

bool HighSpinGuard::update(const Target & target)
{
  if (!config_.enable || target.name == outpost) {
    reset();
    return false;
  }

  const Eigen::VectorXd x = target.ekf_x();
  if (x.size() <= 10 || !x.allFinite()) {
    reset();
    return false;
  }

  const double spin_speed = std::abs(x[7]);
  if (!std::isfinite(spin_speed)) {
    reset();
    return false;
  }

  // 目标编号改变时，不能继续沿用上一个目标的半径和高度层。
  if (active_ && target.name != target_name_) {
    reset();
  }

  // 已进入守点模式后，低于退出阈值才恢复普通策略。
  if (active_ && spin_speed <= config_.exit_speed_rad_s) {
    reset();
    return false;
  }

  if (active_) {
    return true;
  }

  // 没超过进入阈值时保持原有 EKF + MPC 策略。
  if (spin_speed <= config_.enter_speed_rad_s) {
    return false;
  }

  active_ = true;
  target_name_ = target.name;

  if (!initializeGuard(target)) {
    reset();
    return false;
  }

  return true;
}

bool HighSpinGuard::initializeGuard(const Target & target)
{
  const Eigen::VectorXd x = target.ekf_x();
  const std::vector<Eigen::Vector4d> armors =
    target.armor_xyza_list();

  if (x.size() <= 10 || armors.empty()) {
    return false;
  }

  int nearest_index = 0;
  double nearest_distance =
    armors.front().head<2>().norm();

  for (int i = 1; i < static_cast<int>(armors.size()); ++i) {
    const double distance = armors[i].head<2>().norm();

    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  const Eigen::Vector4d & nearest = armors[nearest_index];

  guard_radius_m_ = std::hypot(
    nearest[0] - x[0],
    nearest[1] - x[2]);

  guard_height_offset_m_ = nearest[2] - x[4];

  if (
    !std::isfinite(guard_radius_m_) ||
    !std::isfinite(guard_height_offset_m_) ||
    guard_radius_m_ <= 0.0) {
    return false;
  }

  eligible_armor_indices_.clear();

  if (armors.size() == 4) {
    // Target 的四装甲板模型中，0/2 为一组，1/3 为另一组。
    const int parity = nearest_index % 2;

    for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
      if (i % 2 == parity) {
        eligible_armor_indices_.push_back(i);
      }
    }
  } else {
    // 非四装甲板模型默认认为所有装甲板半径和高度相同。
    for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
      eligible_armor_indices_.push_back(i);
    }
  }

  return !eligible_armor_indices_.empty();
}

double HighSpinGuard::centerBearing(const Target & target)
{
  const Eigen::VectorXd x = target.ekf_x();

  if (x.size() <= 2) {
    throw std::runtime_error(
      "high spin target state is incomplete");
  }

  return std::atan2(x[2], x[0]);
}

Eigen::Vector4d HighSpinGuard::aimPoint(
  const Target & target) const
{
  if (!active_) {
    throw std::runtime_error(
      "high spin guard is not active");
  }

  const Eigen::VectorXd x = target.ekf_x();
  if (x.size() <= 10 || !x.allFinite()) {
    throw std::runtime_error(
      "high spin target state is invalid");
  }

  const double bearing = centerBearing(target);

  // Target 使用：
  // armor_x = center_x - radius * cos(angle)
  // armor_y = center_y - radius * sin(angle)
  // 因此 angle == center bearing 时，装甲板位于旋转中心朝向相机的一侧。
  return {
    x[0] - guard_radius_m_ * std::cos(bearing),
    x[2] - guard_radius_m_ * std::sin(bearing),
    x[4] + guard_height_offset_m_,
    tools::limit_rad(bearing)
  };
}

double HighSpinGuard::phaseError(
  const Target & impact_target) const
{
  if (!active_ || eligible_armor_indices_.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const std::vector<Eigen::Vector4d> armors =
    impact_target.armor_xyza_list();

  if (armors.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const double desired_angle =
    centerBearing(impact_target);

  double minimum_error =
    std::numeric_limits<double>::infinity();

  for (const int index : eligible_armor_indices_) {
    if (index < 0 ||
        index >= static_cast<int>(armors.size())) {
      continue;
    }

    const double error = std::abs(
      tools::limit_rad(
        armors[index][3] - desired_angle));

    minimum_error = std::min(minimum_error, error);
  }

  return minimum_error;
}

bool HighSpinGuard::phaseReady(
  const Target & impact_target) const
{
  return phaseError(impact_target) <=
    config_.phase_tolerance_rad;
}

}  // namespace auto_aim