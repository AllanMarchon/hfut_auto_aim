#ifndef AUTO_AIM__SPIN_SPEED_PROFILE_HPP
#define AUTO_AIM__SPIN_SPEED_PROFILE_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
struct SpinSpeedProfilePoint
{
  double speed_rad_s{0.0};
  double extra_prediction_s{0.0};
  double aligned_hold_s{0.0};
};

struct SpinSpeedProfileValue
{
  double extra_prediction_s{0.0};
  double aligned_hold_s{0.0};
};

class SpinSpeedProfile
{
public:
  static SpinSpeedProfile fromYaml(const YAML::Node & node)
  {
    SpinSpeedProfile profile;
    if (!node || !node.IsSequence()) {
      if (node) throw std::runtime_error("ordinary_spin_profile must be a sequence");
      return profile;
    }

    for (const auto & item : node) {
      SpinSpeedProfilePoint point;
      point.speed_rad_s = item["speed_rad_s"].as<double>();
      point.extra_prediction_s = item["extra_prediction_s"].as<double>();
      point.aligned_hold_s = item["aligned_hold_ms"].as<double>() / 1000.0;
      if (!std::isfinite(point.speed_rad_s) || point.speed_rad_s < 0.0 ||
          !std::isfinite(point.extra_prediction_s) || point.extra_prediction_s < 0.0 ||
          !std::isfinite(point.aligned_hold_s) || point.aligned_hold_s < 0.0) {
        throw std::runtime_error("ordinary_spin_profile values must be finite and non-negative");
      }
      if (!profile.points_.empty() && point.speed_rad_s <= profile.points_.back().speed_rad_s) {
        throw std::runtime_error("ordinary_spin_profile speeds must be strictly increasing");
      }
      profile.points_.push_back(point);
    }
    if (profile.points_.size() < 2) {
      throw std::runtime_error("ordinary_spin_profile requires at least two points");
    }
    return profile;
  }

  bool empty() const { return points_.empty(); }

  SpinSpeedProfileValue sample(double absolute_speed_rad_s) const
  {
    if (points_.empty()) return {};
    if (!std::isfinite(absolute_speed_rad_s)) return {};

    const double speed = std::max(absolute_speed_rad_s, 0.0);
    if (speed <= points_.front().speed_rad_s) return value(points_.front());
    if (speed >= points_.back().speed_rad_s) return value(points_.back());

    const auto upper = std::upper_bound(
      points_.begin(), points_.end(), speed,
      [](double value, const SpinSpeedProfilePoint & point) {
        return value < point.speed_rad_s;
      });
    const auto & high = *upper;
    const auto & low = *(upper - 1);
    const double ratio = (speed - low.speed_rad_s) / (high.speed_rad_s - low.speed_rad_s);
    return {
      low.extra_prediction_s + ratio * (high.extra_prediction_s - low.extra_prediction_s),
      low.aligned_hold_s + ratio * (high.aligned_hold_s - low.aligned_hold_s)};
  }

private:
  std::vector<SpinSpeedProfilePoint> points_;

  static SpinSpeedProfileValue value(const SpinSpeedProfilePoint & point)
  {
    return {point.extra_prediction_s, point.aligned_hold_s};
  }
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SPIN_SPEED_PROFILE_HPP
