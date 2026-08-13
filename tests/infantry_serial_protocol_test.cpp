#include <cmath>
#include <cstdio>

#include "io/serial/infantry_serial.hpp"

namespace {

bool near(double left, double right, double tolerance = 1e-9) {
  return std::fabs(left - right) <= tolerance;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "infantry serial protocol test failed: %s\n", message);
  return code;
}

}  // namespace

int main() {
  hfut::GimbalCommand command;
  command.mode = hfut::GimbalMode::normal_measurement;
  command.fire_advice = true;
  command.yaw = 0.1;
  command.pitch = -0.2;
  command.distance = 3.4;
  command.yaw_vel = 0.5;
  command.pitch_vel = -0.6;
  command.yaw_acc = 7.0;
  command.pitch_acc = -8.0;

  hfut::GimbalCommand normal =
      hfut::io::sanitizeInfantryCommandForTransport(command);
  if (!normal.fire_advice || !near(normal.distance, 3.4) ||
      !near(normal.yaw_vel, 0.5) || !near(normal.pitch_vel, -0.6) ||
      !near(normal.yaw_acc, 7.0) || !near(normal.pitch_acc, -8.0)) {
    return fail("normal measurement command was changed", 1);
  }

  command.mode = hfut::GimbalMode::no_valid_measurement;
  hfut::GimbalCommand safe =
      hfut::io::sanitizeInfantryCommandForTransport(command);
  if (safe.fire_advice || !near(safe.distance, -1.0) ||
      !near(safe.yaw_vel, 0.0) || !near(safe.pitch_vel, 0.0) ||
      !near(safe.yaw_acc, 0.0) || !near(safe.pitch_acc, 0.0)) {
    return fail("non-normal command was not sanitized", 2);
  }
  if (!near(safe.yaw, command.yaw) || !near(safe.pitch, command.pitch)) {
    return fail("sanitizer changed absolute angle setpoints", 3);
  }

  hfut::io::InfantrySerialConfig config;
  if (config.tx_layout != hfut::io::InfantryPacketLayout::kInfantry24 ||
      config.rx_layout != hfut::io::InfantryPacketLayout::kInfantry24) {
    return fail("default TX/RX protocols are not 24-byte infantry", 4);
  }
  if (config.tail_fields != hfut::io::Infantry32TailFields::kDuplicateVelocity) {
    return fail("default 32-byte tail fields are not legacy-compatible", 5);
  }

  hfut::io::Infantry32TailFields parsed{};
  if (!hfut::io::parseInfantry32TailFields("acceleration", parsed) ||
      parsed != hfut::io::Infantry32TailFields::kAcceleration) {
    return fail("acceleration tail field option did not parse", 6);
  }
  if (!hfut::io::parseInfantry32TailFields("duplicate_velocity", parsed) ||
      parsed != hfut::io::Infantry32TailFields::kDuplicateVelocity) {
    return fail("duplicate_velocity tail field option did not parse", 7);
  }

  return 0;
}
