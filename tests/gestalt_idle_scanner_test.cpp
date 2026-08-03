#include "io/gestalt/gestalt_idle_scanner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

bool near(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
  hfut::io::GestaltIdleScanConfig config;
  config.yaw_rate_deg_s = 20.0;
  config.pitch_rate_deg_s = 6.0;
  config.pitch_limit_deg = 30.0;
  config.activation_delay_s = 3.0;
  config.max_step_s = 0.20;
  hfut::io::GestaltIdleScanner scanner(config);

  const auto t0 = hfut::io::GestaltIdleScanner::Clock::time_point{};
  hfut::GimbalCommand command;
  command.yaw = 123.0;
  if (scanner.update(false, false, 1.0, 0.0, t0, command) ||
      scanner.scanning() || !near(command.yaw, 123.0)) {
    std::fprintf(stderr, "idle scanner took ownership before activation delay\n");
    return 1;
  }

  const auto activation_time = t0 + std::chrono::seconds(3);
  if (!scanner.update(false, false, 1.0, 0.0, activation_time, command) ||
      !scanner.scanning() || !near(command.yaw, 1.0) ||
      !near(command.pitch, 0.0) || command.fire_advice ||
      command.mode != hfut::GimbalMode::normal_measurement) {
    std::fprintf(stderr, "idle scanner did not activate after delay\n");
    return 2;
  }

  // A long frame is capped at 0.2 s: yaw advances 4 deg and pitch 1.2 deg.
  scanner.update(false, false, 1.0, 0.0, activation_time + std::chrono::seconds(1), command);
  if (!near(command.yaw, 1.0 + 4.0 * kDegToRad) ||
      !near(command.pitch, -1.2 * kDegToRad) ||
      !near(command.yaw_vel, 20.0 * kDegToRad) ||
      !near(command.pitch_vel, -6.0 * kDegToRad)) {
    std::fprintf(stderr, "idle scanner rate or capped time step is incorrect\n");
    return 3;
  }

  double min_pitch = command.pitch;
  double max_pitch = command.pitch;
  auto time = activation_time + std::chrono::seconds(1);
  for (int i = 0; i < 100; ++i) {
    time += std::chrono::milliseconds(200);
    scanner.update(false, false, command.yaw, command.pitch, time, command);
    min_pitch = std::min(min_pitch, command.pitch);
    max_pitch = std::max(max_pitch, command.pitch);
    if (std::abs(command.pitch) > 30.0 * kDegToRad + 1e-9) {
      std::fprintf(stderr, "idle scanner exceeded pitch limits\n");
      return 4;
    }
  }
  if (min_pitch > -29.9 * kDegToRad || max_pitch < 29.9 * kDegToRad) {
    std::fprintf(stderr, "idle scanner did not cover both pitch limits\n");
    return 5;
  }

  if (!scanner.update(false, true, 0.5, 0.2, time, command) ||
      scanner.scanning() || !near(command.yaw, 0.5) || !near(command.pitch, 0.2) ||
      !near(command.yaw_vel, 0.0) || !near(command.pitch_vel, 0.0)) {
    std::fprintf(stderr, "visual contact did not freeze at feedback pose\n");
    return 6;
  }

  command.yaw = 123.0;
  if (scanner.update(true, false, 0.5, 0.2, time, command) ||
      scanner.scanning() || !near(command.yaw, 123.0)) {
    std::fprintf(stderr, "auto aim did not retain ownership of its command\n");
    return 7;
  }

  command.yaw = 321.0;
  if (scanner.update(false, false, 0.5, 0.2, time, command) ||
      scanner.scanning() || !near(command.yaw, 321.0)) {
    std::fprintf(stderr, "idle scanner delay was not restarted after contact\n");
    return 8;
  }
  return 0;
}
