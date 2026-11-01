#include "io/serial/sentry_serial.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "io/serial/fixed_packet.hpp"
#include "io/serial/sentry_packet.hpp"

namespace hfut::io {
namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

SentrySerialTransport::SentrySerialTransport(SentrySerialConfig config)
    : config_(std::move(config)),
      uart_(std::make_unique<UartTransport>(config_.port, config_.baudrate)) {}

bool SentrySerialTransport::open() { return uart_->open(); }

void SentrySerialTransport::close() { uart_->close(); }

bool SentrySerialTransport::isOpen() const { return uart_->isOpen(); }

float SentrySerialTransport::finiteOrZero(double value) {
  return std::isfinite(value) ? static_cast<float>(value) : 0.0F;
}

float SentrySerialTransport::angleToWire(double radians) {
  return finiteOrZero(radians * kRadToDeg);
}

bool SentrySerialTransport::sendCommand(const GimbalCommand& command) {
  if (!isOpen() && !open()) return false;

  const bool control = command.mode == GimbalMode::normal_measurement;
  const bool fire = control && config_.allow_fire && command.fire_advice;
  const float distance = control ? finiteOrZero(command.distance) : -1.0F;
  // 上层 convertCommand 已经完成 SP pitch 到电控 pitch 的符号转换。
  const float pitch_velocity = control ? angleToWire(command.pitch_vel) : 0.0F;
  const float yaw_velocity = control ? angleToWire(command.yaw_vel) : 0.0F;

  FixedPacket64 packet;
  packet.load(static_cast<std::uint8_t>(fire ? 1 : 0), 1);
  packet.load(angleToWire(command.pitch), 2);
  packet.load(angleToWire(command.yaw), 6);
  packet.load(distance, 10);
  packet.load(static_cast<std::int32_t>(control ? 0 : -1), 14);
  packet.load(pitch_velocity, 18);
  packet.load(yaw_velocity, 22);
  packet.setCrc();
  return uart_->writeAll(packet.data(), packet.size(), config_.write_timeout_ms);
}

bool SentrySerialTransport::readFeedbackPacket(SerialFeedback& feedback) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "sentry feedback requires 32-bit IEEE 754 floats");

  FixedPacket64 packet;
  if (!FixedPacket64::takeFromBuffer(rx_buffer_, packet)) return false;

  std::uint8_t mode = 0;
  float roll = 0.0F;
  float yaw = 0.0F;
  float pitch = 0.0F;
  packet.unload(mode, 1);
  packet.unload(roll, 2);
  packet.unload(yaw, 6);
  packet.unload(pitch, 10);

  feedback.received = true;
  feedback.mode = mode;
  feedback.roll_rad = std::isfinite(roll) ? static_cast<double>(roll) * kDegToRad : 0.0;
  feedback.yaw_rad = std::isfinite(yaw) ? static_cast<double>(yaw) * kDegToRad : 0.0;
  feedback.pitch_rad = std::isfinite(pitch) ? static_cast<double>(pitch) * kDegToRad : 0.0;
  feedback.bullet_speed = 0.0;
  feedback.fire_delay_s.reset();
  feedback.chassis_vx = 0.0;
  feedback.chassis_vy = 0.0;
  feedback.chassis_wz = 0.0;
  return true;
}

bool SentrySerialTransport::readFeedback(SerialFeedback& feedback) {
  if (!isOpen() && !open()) return false;

  std::uint8_t buffer[256]{};
  const int count = uart_->readSome(buffer, sizeof(buffer), config_.read_timeout_ms);
  if (count < 0) {
    rx_buffer_.clear();
    close();
    return false;
  }
  for (int i = 0; i < count; ++i) rx_buffer_.push_back(buffer[i]);
  if (rx_buffer_.size() > kSentryPacketSize * 32) rx_buffer_.clear();

  bool received = false;
  SerialFeedback parsed;
  while (readFeedbackPacket(parsed)) {
    feedback = parsed;
    received = true;
  }
  return received;
}

const std::string& SentrySerialTransport::errorMessage() const {
  return uart_->errorMessage();
}

}  // namespace hfut::io
