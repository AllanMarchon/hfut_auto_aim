#include "tasks/auto_aim/armor_avoidance_optimizer.hpp"

#include <cassert>
#include <cmath>

int main()
{
  using auto_aim::ArmorAvoidanceConfig;
  using auto_aim::ArmorAvoidanceResult;
  using auto_aim::applyArmorAvoidanceOffset;
  using auto_aim::optimizeArmorAvoidance;

  const ArmorAvoidanceResult sample_offset{true, 10.0, 5.0, 0.0, 0.0, 0.0};
  double world_x = 0.0, world_y = 0.0, world_z = 0.0;
  applyArmorAvoidanceOffset(sample_offset, 0.0, world_x, world_y, world_z);
  assert(std::abs(world_x) < 1e-12 && std::abs(world_y + 0.01) < 1e-12);
  assert(std::abs(world_z - 0.005) < 1e-12);
  world_x = 0.0;
  world_y = 0.0;
  world_z = 0.0;
  applyArmorAvoidanceOffset(sample_offset, 1.5707963267948966, world_x, world_y, world_z);
  assert(std::abs(world_x - 0.01) < 1e-12 && std::abs(world_y) < 1e-12);
  assert(std::abs(world_z - 0.005) < 1e-12);

  ArmorAvoidanceConfig config;
  const auto disabled = optimizeArmorAvoidance(config);
  assert(disabled.valid);
  assert(disabled.x_mm == 0.0 && disabled.y_mm == 0.0);

  config.enabled = true;
  config.dispersion_sigma_mm = 10.0;
  const auto narrow_scatter = optimizeArmorAvoidance(config);
  assert(narrow_scatter.valid);
  assert(narrow_scatter.clean_probability > 0.97);
  assert(narrow_scatter.miss_probability < 0.02);
  assert(std::hypot(narrow_scatter.x_mm, narrow_scatter.y_mm) > config.radius_mm);

  config.dispersion_sigma_mm = 30.0;
  const auto scattered = optimizeArmorAvoidance(config);
  assert(scattered.valid);
  const double probability_sum =
    scattered.clean_probability + scattered.reinforced_probability + scattered.miss_probability;
  assert(std::abs(probability_sum - 1.0) < 1e-8);
  const auto center_candidate = auto_aim::armor_avoidance_detail::evaluate(0.0, 0.0, config);
  assert(
    auto_aim::armor_avoidance_detail::expected_damage(scattered) + 1e-8 >=
    auto_aim::armor_avoidance_detail::expected_damage(center_candidate));

  config.radius_mm = 5.0;
  const auto small_circle = optimizeArmorAvoidance(config);
  assert(small_circle.valid);
  assert(
    auto_aim::armor_avoidance_detail::expected_damage(small_circle) + 1e-8 >=
    auto_aim::armor_avoidance_detail::expected_damage(
      auto_aim::armor_avoidance_detail::evaluate(0.0, 0.0, config)));

  config.center_x_mm = -25.0;
  const auto offset_circle = optimizeArmorAvoidance(config);
  assert(offset_circle.valid);
  assert(std::isfinite(offset_circle.x_mm) && std::isfinite(offset_circle.y_mm));

  config.center_x_mm = 0.0;
  config.radius_mm = 20.0;
  config.dispersion_sigma_mm = 50.0;
  const auto distant_small_circle = optimizeArmorAvoidance(config);
  assert(distant_small_circle.valid);
  assert(
    std::hypot(distant_small_circle.x_mm, distant_small_circle.y_mm) < config.radius_mm);
  const auto risky_clean_candidate =
    auto_aim::armor_avoidance_detail::evaluate(45.0, 0.0, config);
  assert(
    auto_aim::armor_avoidance_detail::expected_damage(distant_small_circle) >
    auto_aim::armor_avoidance_detail::expected_damage(risky_clean_candidate));

  config.dispersion_sigma_mm = 30.0;
  config.radius_mm = 65.0;
  const auto large_circle = optimizeArmorAvoidance(config);
  assert(large_circle.valid);
  assert(std::hypot(large_circle.x_mm, large_circle.y_mm) < 2.0);
  assert(large_circle.reinforced_probability > 0.5);
  const auto risky_corner_candidate =
    auto_aim::armor_avoidance_detail::evaluate(62.0, 20.0, config);
  assert(
    auto_aim::armor_avoidance_detail::expected_damage(large_circle) >
    auto_aim::armor_avoidance_detail::expected_damage(risky_corner_candidate));

  config.radius_mm = 300.0;
  const auto covered_plate = optimizeArmorAvoidance(config);
  assert(covered_plate.valid);
  assert(covered_plate.clean_probability < 1e-8);
  assert(covered_plate.reinforced_probability > 0.0);
  assert(std::hypot(covered_plate.x_mm, covered_plate.y_mm) < 2.0);

  config.dispersion_sigma_mm = 0.0;
  const auto deterministic_covered_plate = optimizeArmorAvoidance(config);
  assert(deterministic_covered_plate.valid);
  assert(deterministic_covered_plate.clean_probability == 0.0);
  assert(deterministic_covered_plate.reinforced_probability == 1.0);
  assert(deterministic_covered_plate.miss_probability == 0.0);

  return 0;
}
