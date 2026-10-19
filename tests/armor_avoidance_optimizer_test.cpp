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
  assert(
    std::hypot(scattered.x_mm - config.center_x_mm, scattered.y_mm - config.center_y_mm) >
    config.radius_mm);
  const double probability_sum =
    scattered.clean_probability + scattered.reinforced_probability + scattered.miss_probability;
  assert(std::abs(probability_sum - 1.0) < 1e-8);
  assert(scattered.clean_probability > 0.0);

  config.radius_mm = 5.0;
  const auto small_circle = optimizeArmorAvoidance(config);
  assert(small_circle.valid);
  assert(std::hypot(small_circle.x_mm, small_circle.y_mm) > config.radius_mm);

  config.center_x_mm = -25.0;
  const auto offset_circle = optimizeArmorAvoidance(config);
  assert(offset_circle.valid);
  assert(offset_circle.x_mm > 0.0);

  config.center_x_mm = 0.0;
  config.radius_mm = 20.0;
  config.dispersion_sigma_mm = 0.0;
  const auto deterministic = optimizeArmorAvoidance(config);
  assert(deterministic.valid);
  assert(std::hypot(deterministic.x_mm, deterministic.y_mm) > config.radius_mm);
  assert(std::abs(deterministic.x_mm) < config.plate_width_mm / 2.0);
  assert(std::abs(deterministic.y_mm) < config.plate_height_mm / 2.0);

  config.radius_mm = 72.5;
  const auto narrow_safe_corner = optimizeArmorAvoidance(config);
  assert(narrow_safe_corner.valid);
  assert(std::hypot(narrow_safe_corner.x_mm, narrow_safe_corner.y_mm) > config.radius_mm);

  config.radius_mm = 300.0;
  const auto covered_plate = optimizeArmorAvoidance(config);
  assert(!covered_plate.valid);

  config.radius_mm = 74.0;
  const auto fully_covered_board = optimizeArmorAvoidance(config);
  assert(!fully_covered_board.valid);

  return 0;
}
