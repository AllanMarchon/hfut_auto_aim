#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>

#include "io/serial/fixed_packet.hpp"

namespace {

bool near(float left, float right, float tolerance = 1e-5F) {
  return std::fabs(left - right) <= tolerance;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "fixed packet test failed: %s\n", message);
  return code;
}

}  // namespace

int main() {
  hfut::io::FixedPacket32 packet;
  const std::uint8_t mode = 3;
  const float roll = 1.25F;
  const float pitch = -2.5F;
  const float yaw = 17.75F;

  if (!packet.load(mode, 1) || !packet.load(roll, 2) ||
      !packet.load(pitch, 6) || !packet.load(yaw, 10)) {
    return fail("valid payload offsets were rejected", 1);
  }
  packet.setCrc();
  if (!packet.valid()) {
    return fail("fresh packet did not validate after CRC", 2);
  }

  std::uint8_t decoded_mode = 0;
  float decoded_roll = 0.0F;
  float decoded_pitch = 0.0F;
  float decoded_yaw = 0.0F;
  if (!packet.unload(decoded_mode, 1) || !packet.unload(decoded_roll, 2) ||
      !packet.unload(decoded_pitch, 6) || !packet.unload(decoded_yaw, 10)) {
    return fail("valid payload offsets could not be decoded", 3);
  }
  if (decoded_mode != mode || !near(decoded_roll, roll) ||
      !near(decoded_pitch, pitch) || !near(decoded_yaw, yaw)) {
    return fail("decoded payload changed values", 4);
  }

  hfut::io::FixedPacket32 corrupted;
  corrupted.copyFrom(packet.data());
  corrupted.data()[10] ^= 0x10U;
  if (corrupted.valid()) {
    return fail("corrupted packet passed CRC validation", 5);
  }

  std::deque<std::uint8_t> rx = {0x00U, 0x7EU};
  for (std::size_t i = 0; i < packet.size(); ++i) {
    rx.push_back(packet.data()[i]);
  }
  rx.push_back(0x55U);

  hfut::io::FixedPacket32 parsed;
  if (!hfut::io::FixedPacket32::takeFromBuffer(rx, parsed)) {
    return fail("packet was not extracted from noisy receive buffer", 6);
  }
  if (!parsed.valid() || rx.size() != 1U || rx.front() != 0x55U) {
    return fail("receive buffer extraction left unexpected state", 7);
  }

  std::uint8_t parsed_mode = 0;
  parsed.unload(parsed_mode, 1);
  if (parsed_mode != mode) {
    return fail("parsed packet payload was not preserved", 8);
  }

  return 0;
}
