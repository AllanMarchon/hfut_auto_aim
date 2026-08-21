#include "io/serial/infantry32_serial.hpp"

#include <cmath>
#include <utility>

namespace hfut::io {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

GimbalCommand sanitizeInfantry32CommandForTransport(const GimbalCommand& command) {
  GimbalCommand sanitized = command;
  if (sanitized.mode != GimbalMode::normal_measurement) {
    sanitized.fire_advice = false;
    sanitized.distance = -1.0;
    sanitized.pitch_vel = 0.0;
    sanitized.yaw_vel = 0.0;
    sanitized.pitch_acc = 0.0;
    sanitized.yaw_acc = 0.0;
  }
  return sanitized;
}
}  // namespace

Infantry32SerialTransport::Infantry32SerialTransport(Infantry32SerialConfig config)
    : config_(std::move(config)),
      uart_(std::make_unique<UartTransport>(config_.port, config_.baudrate)) {}

bool Infantry32SerialTransport::open() { return uart_->open(); }

void Infantry32SerialTransport::close() { uart_->close(); }

bool Infantry32SerialTransport::isOpen() const { return uart_->isOpen(); }

float Infantry32SerialTransport::finiteOrZero(double value) {
  return std::isfinite(value) ? static_cast<float>(value) : 0.0F;
}

double Infantry32SerialTransport::feedbackAngleToRad(float value) const {
  return config_.feedback_angles_in_degrees ? static_cast<double>(value) * kDegToRad
                                            : static_cast<double>(value);
}

float Infantry32SerialTransport::commandAngle(double rad) const {
  const double value = config_.command_angles_in_degrees ? rad * kRadToDeg : rad;
  return finiteOrZero(value);
}

bool Infantry32SerialTransport::sendCommand(const GimbalCommand& command) {
  if (!isOpen() && !open()) return false;

  const GimbalCommand wire_command = sanitizeInfantry32CommandForTransport(command);
  FixedPacket32 packet;
  const std::uint8_t fire = (config_.allow_fire && wire_command.fire_advice) ? 1 : 0;
  packet.load(fire, 1);
  packet.load(commandAngle(wire_command.pitch), 2);
  packet.load(commandAngle(wire_command.yaw), 6);
  packet.load(finiteOrZero(wire_command.distance), 10);
  packet.load(commandAngle(wire_command.pitch_vel), 14);
  packet.load(commandAngle(wire_command.yaw_vel), 18);
  packet.load(commandAngle(wire_command.pitch_acc), 22);
  packet.load(commandAngle(wire_command.yaw_acc), 26);
  packet.setCrc();
  return uart_->writeAll(packet.data(), packet.size(), config_.write_timeout_ms);
}

bool Infantry32SerialTransport::readFeedback(SerialFeedback& feedback) {
  if (!isOpen() && !open()) return false;

  std::uint8_t tmp[256]{};
  const int n = uart_->readSome(tmp, sizeof(tmp), config_.read_timeout_ms);
  if (n < 0) {
    rx_buffer_.clear();
    close();
    return false;
  }
  for (int i = 0; i < n; ++i) rx_buffer_.push_back(tmp[i]);
  if (rx_buffer_.size() > 32 * 32) rx_buffer_.clear();

  FixedPacket32 packet;
  bool got_latest = false;
  while (FixedPacket32::takeFromBuffer(rx_buffer_, packet)) {
    std::uint8_t mode = 0;
    float roll = 0.0F;
    float pitch = 0.0F;
    float yaw = 0.0F;
    packet.unload(mode, 1);
    packet.unload(roll, 2);
    packet.unload(pitch, 6);
    packet.unload(yaw, 10);

    feedback.received = true;
    feedback.mode = mode;
    feedback.roll_rad = feedbackAngleToRad(roll);
    feedback.pitch_rad = feedbackAngleToRad(pitch);
    feedback.yaw_rad = feedbackAngleToRad(yaw);
    got_latest = true;
  }

  return got_latest;
}

}  // namespace hfut::io
