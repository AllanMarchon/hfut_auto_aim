#ifndef AUTO_AIM__ARMOR_AVOIDANCE_OPTIMIZER_HPP
#define AUTO_AIM__ARMOR_AVOIDANCE_OPTIMIZER_HPP

#include <algorithm>
#include <cmath>

namespace auto_aim
{

struct ArmorAvoidanceConfig
{
  bool enabled{false};
  double plate_width_mm{135.0};
  double plate_height_mm{56.0};
  double center_x_mm{0.0};
  double center_y_mm{0.0};
  double radius_mm{20.0};
  double dispersion_sigma_mm{10.0};
};

struct ArmorAvoidanceResult
{
  bool valid{true};
  double x_mm{0.0};
  double y_mm{0.0};
  double clean_probability{1.0};
  double reinforced_probability{0.0};
  double miss_probability{0.0};
};

inline void applyArmorAvoidanceOffset(
  const ArmorAvoidanceResult & result, double yaw, double & world_x_m, double & world_y_m,
  double & world_z_m)
{
  const double right_offset_m = result.x_mm / 1000.0;
  const double up_offset_m = result.y_mm / 1000.0;
  // PnP 物点的局部 y 正方向指向图像左侧，因此板面 x 向右要取其反方向。
  world_x_m += std::sin(yaw) * right_offset_m;
  world_y_m -= std::cos(yaw) * right_offset_m;
  world_z_m += up_offset_m;
}

namespace armor_avoidance_detail
{
constexpr double kInvSqrtTwo{0.70710678118654752440};
constexpr double kInvSqrtTwoPi{0.39894228040143267794};
constexpr double kUnreinforcedDamageMultiplier{1.5};
constexpr double kReinforcedDamageMultiplier{0.5};

inline double normal_cdf(double value)
{
  return 0.5 * std::erfc(-value * kInvSqrtTwo);
}

inline double normal_interval(double low, double high, double mean, double sigma)
{
  if (high <= low) return 0.0;
  return std::clamp(
    normal_cdf((high - mean) / sigma) - normal_cdf((low - mean) / sigma), 0.0, 1.0);
}

inline double plate_probability(double x, double y, const ArmorAvoidanceConfig & config)
{
  const double half_width = config.plate_width_mm / 2.0;
  const double half_height = config.plate_height_mm / 2.0;
  return normal_interval(-half_width, half_width, x, config.dispersion_sigma_mm) *
         normal_interval(-half_height, half_height, y, config.dispersion_sigma_mm);
}

inline double reinforced_probability(double x, double y, const ArmorAvoidanceConfig & config)
{
  const double sigma = config.dispersion_sigma_mm;
  const double half_width = config.plate_width_mm / 2.0;
  const double half_height = config.plate_height_mm / 2.0;
  const double farthest_x = std::max(
    std::abs(-half_width - config.center_x_mm),
    std::abs(half_width - config.center_x_mm));
  const double farthest_y = std::max(
    std::abs(-half_height - config.center_y_mm),
    std::abs(half_height - config.center_y_mm));
  if (farthest_x * farthest_x + farthest_y * farthest_y <=
      config.radius_mm * config.radius_mm) {
    return plate_probability(x, y, config);
  }

  const double nearest_x = std::max(std::abs(config.center_x_mm) - half_width, 0.0);
  const double nearest_y = std::max(std::abs(config.center_y_mm) - half_height, 0.0);
  if (nearest_x * nearest_x + nearest_y * nearest_y >=
      config.radius_mm * config.radius_mm) {
    return 0.0;
  }

  const double low =
    std::max({-half_width, config.center_x_mm - config.radius_mm, x - 6.0 * sigma});
  const double high =
    std::min({half_width, config.center_x_mm + config.radius_mm, x + 6.0 * sigma});
  if (high <= low) return 0.0;

  // 用复合 Simpson 积分计算高斯散布落入圆形加固区与矩形装甲板交集的概率。
  constexpr int intervals = 64;
  const double step = (high - low) / intervals;
  const double inverse_sigma = 1.0 / sigma;
  const double normalizer = kInvSqrtTwoPi * inverse_sigma;
  double sum = 0.0;
  for (int i = 0; i <= intervals; ++i) {
    const double sample_x = low + i * step;
    const double delta_x = sample_x - x;
    const double density_x = normalizer * std::exp(-0.5 * delta_x * delta_x * inverse_sigma * inverse_sigma);
    const double circle_dx = sample_x - config.center_x_mm;
    const double half_chord = std::sqrt(std::max(0.0, config.radius_mm * config.radius_mm - circle_dx * circle_dx));
    const double forbidden_low = std::max(-half_height, config.center_y_mm - half_chord);
    const double forbidden_high = std::min(half_height, config.center_y_mm + half_chord);
    const double density_y = normal_interval(forbidden_low, forbidden_high, y, sigma);
    const double weight = (i == 0 || i == intervals) ? 1.0 : (i % 2 == 0 ? 2.0 : 4.0);
    sum += weight * density_x * density_y;
  }
  return std::clamp(sum * step / 3.0, 0.0, 1.0);
}

inline ArmorAvoidanceResult evaluate(double x, double y, const ArmorAvoidanceConfig & config)
{
  const double plate = plate_probability(x, y, config);
  const double reinforced = std::min(plate, reinforced_probability(x, y, config));
  return {
    true, x, y, std::max(0.0, plate - reinforced), reinforced,
    std::clamp(1.0 - plate, 0.0, 1.0)};
}

inline double expected_damage(const ArmorAvoidanceResult & result)
{
  return kUnreinforcedDamageMultiplier * result.clean_probability +
         kReinforcedDamageMultiplier * result.reinforced_probability;
}

inline double hit_probability(const ArmorAvoidanceResult & result)
{
  return result.clean_probability + result.reinforced_probability;
}

inline bool better_candidate(
  const ArmorAvoidanceResult & candidate, const ArmorAvoidanceResult & current)
{
  if (!current.valid) return true;

  constexpr double score_epsilon = 1e-10;
  const double candidate_damage = expected_damage(candidate);
  const double current_damage = expected_damage(current);
  if (candidate_damage > current_damage + score_epsilon) return true;
  if (candidate_damage < current_damage - score_epsilon) return false;

  // 收益几乎相同时，优先保住命中概率，再选更靠近板面中心的位置。
  const double candidate_hit = hit_probability(candidate);
  const double current_hit = hit_probability(current);
  if (candidate_hit > current_hit + score_epsilon) return true;
  if (candidate_hit < current_hit - score_epsilon) return false;
  return candidate.x_mm * candidate.x_mm + candidate.y_mm * candidate.y_mm <
         current.x_mm * current.x_mm + current.y_mm * current.y_mm - score_epsilon;
}

inline bool outside_circle(double x, double y, const ArmorAvoidanceConfig & config)
{
  const double dx = x - config.center_x_mm;
  const double dy = y - config.center_y_mm;
  return dx * dx + dy * dy > config.radius_mm * config.radius_mm + 1e-8;
}

inline bool inside_plate(double x, double y, const ArmorAvoidanceConfig & config)
{
  return std::abs(x) <= config.plate_width_mm / 2.0 &&
         std::abs(y) <= config.plate_height_mm / 2.0;
}

inline void consider_zero_scatter(
  double x, double y, const ArmorAvoidanceConfig & config, ArmorAvoidanceResult & best)
{
  if (!inside_plate(x, y, config)) return;
  const bool reinforced = !outside_circle(x, y, config);
  const ArmorAvoidanceResult candidate{
    true, x, y, reinforced ? 0.0 : 1.0, reinforced ? 1.0 : 0.0, 0.0};
  if (better_candidate(candidate, best)) best = candidate;
}
}  // namespace armor_avoidance_detail

inline ArmorAvoidanceResult optimizeArmorAvoidance(const ArmorAvoidanceConfig & config)
{
  using namespace armor_avoidance_detail;
  if (!config.enabled) return {};
  if (!(std::isfinite(config.plate_width_mm) && config.plate_width_mm > 0.0 &&
        std::isfinite(config.plate_height_mm) && config.plate_height_mm > 0.0 &&
        std::isfinite(config.center_x_mm) && std::isfinite(config.center_y_mm) &&
        std::isfinite(config.radius_mm) && config.radius_mm >= 0.0 &&
        std::isfinite(config.dispersion_sigma_mm) && config.dispersion_sigma_mm >= 0.0)) {
    return {false, 0.0, 0.0, 0.0, 0.0, 1.0};
  }

  const double half_width = config.plate_width_mm / 2.0;
  const double half_height = config.plate_height_mm / 2.0;
  const double sigma = config.dispersion_sigma_mm;
  if (sigma <= 1e-6) {
    ArmorAvoidanceResult best{false, 0.0, 0.0, 0.0, 0.0, 1.0};
    const double step = 2.0;
    for (double y = -half_height; y <= half_height + 1e-9; y += step) {
      for (double x = -half_width; x <= half_width + 1e-9; x += step) {
        consider_zero_scatter(x, y, config, best);
      }
    }
    for (double y : {-half_height, half_height}) {
      for (double x : {-half_width, half_width}) {
        consider_zero_scatter(x, y, config, best);
      }
    }
    // 圆边外侧取样，避免粗网格漏掉很窄的未加固区域。
    constexpr int circle_samples = 1440;
    for (int i = 0; i < circle_samples; ++i) {
      const double angle = 2.0 * 3.14159265358979323846 * i / circle_samples;
      const double x = config.center_x_mm + (config.radius_mm + 0.05) * std::cos(angle);
      const double y = config.center_y_mm + (config.radius_mm + 0.05) * std::sin(angle);
      consider_zero_scatter(x, y, config, best);
    }
    // 即使加固圆覆盖整块板，也返回板内收益最高的加固区落点。
    if (!best.valid) consider_zero_scatter(0.0, 0.0, config, best);
    const double fine_step = 0.25;
    const double start_x = std::max(-half_width, best.x_mm - step);
    const double end_x = std::min(half_width, best.x_mm + step);
    const double start_y = std::max(-half_height, best.y_mm - step);
    const double end_y = std::min(half_height, best.y_mm + step);
    for (double y = start_y; y <= end_y + 1e-9; y += fine_step) {
      for (double x = start_x; x <= end_x + 1e-9; x += fine_step) {
        consider_zero_scatter(x, y, config, best);
      }
    }
    return best;
  }

  const double grid_step = std::clamp(sigma * 0.5, 2.0, 8.0);
  ArmorAvoidanceResult best{false, 0.0, 0.0, -1.0, 0.0, 1.0};
  const auto consider = [&](double x, double y, ArmorAvoidanceResult & current) {
    // 整块板都参与比较，加固区内的高命中率落点也可以胜出。
    if (!inside_plate(x, y, config)) return;
    const auto candidate = evaluate(x, y, config);
    if (better_candidate(candidate, current)) current = candidate;
  };
  consider(0.0, 0.0, best);
  consider(
    std::clamp(config.center_x_mm, -half_width, half_width),
    std::clamp(config.center_y_mm, -half_height, half_height), best);
  for (double y = -half_height; y <= half_height + 1e-9; y += grid_step) {
    for (double x = -half_width; x <= half_width + 1e-9; x += grid_step) {
      consider(x, y, best);
    }
  }
  for (double y : {-half_height, half_height}) {
    for (double x : {-half_width, half_width}) consider(x, y, best);
  }
  if (!best.valid) return {false, 0.0, 0.0, 0.0, 0.0, 1.0};

  // 在全局粗搜的最优点周围细化到 1 毫米，避免每帧对整板做高精度搜索。
  const double fine_step = 1.0;
  const double start_x = std::max(-half_width, best.x_mm - grid_step);
  const double end_x = std::min(half_width, best.x_mm + grid_step);
  const double start_y = std::max(-half_height, best.y_mm - grid_step);
  const double end_y = std::min(half_height, best.y_mm + grid_step);
  for (double y = start_y; y <= end_y + 1e-9; y += fine_step) {
    for (double x = start_x; x <= end_x + 1e-9; x += fine_step) {
      consider(x, y, best);
    }
  }
  return best;
}

inline bool sameArmorAvoidanceConfig(
  const ArmorAvoidanceConfig & left, const ArmorAvoidanceConfig & right)
{
  return left.enabled == right.enabled && left.plate_width_mm == right.plate_width_mm &&
         left.plate_height_mm == right.plate_height_mm && left.center_x_mm == right.center_x_mm &&
         left.center_y_mm == right.center_y_mm && left.radius_mm == right.radius_mm &&
         left.dispersion_sigma_mm == right.dispersion_sigma_mm;
}

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_AVOIDANCE_OPTIMIZER_HPP
