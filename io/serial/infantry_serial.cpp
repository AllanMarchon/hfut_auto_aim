#include "io/serial/infantry_serial.hpp"

#include <cmath>
#include <utility>

namespace hfut::io {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
}  // namespace

bool parseInfantryPacketLayout(const std::string& value,
                               InfantryPacketLayout& layout) {
  if (value == "infantry" || value == "infantry_24" || value == "24") {
    layout = InfantryPacketLayout::kInfantry24;
    return true;
  }
  if (value == "infantry_16" || value == "16") {
    layout = InfantryPacketLayout::kInfantry16;
    return true;
  }
  if (value == "infantry_32" || value == "32") {
    layout = InfantryPacketLayout::kInfantry32;
    return true;
  }
  return false;
}

const char* infantryPacketLayoutName(InfantryPacketLayout layout) {
  switch (layout) {
    case InfantryPacketLayout::kInfantry16: return "infantry_16";
    case InfantryPacketLayout::kInfantry24: return "infantry";
    case InfantryPacketLayout::kInfantry32: return "infantry_32";
  }
  return "unknown";
}

bool parseInfantry32TailFields(const std::string& value,
                               Infantry32TailFields& fields) {
  if (value == "acceleration" || value == "accel") {
    fields = Infantry32TailFields::kAcceleration;
    return true;
  }
  if (value == "duplicate_velocity" || value == "velocity") {
    fields = Infantry32TailFields::kDuplicateVelocity;
    return true;
  }
  return false;
}

const char* infantry32TailFieldsName(Infantry32TailFields fields) {
  switch (fields) {
    case Infantry32TailFields::kAcceleration: return "acceleration";
    case Infantry32TailFields::kDuplicateVelocity: return "duplicate_velocity";
  }
  return "unknown";
}

InfantrySerialTransport::InfantrySerialTransport(InfantrySerialConfig config)
    : config_(std::move(config)),
      uart_(std::make_unique<UartTransport>(config_.port, config_.baudrate)) {}

GimbalCommand sanitizeInfantryCommandForTransport(const GimbalCommand& command) {
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

bool InfantrySerialTransport::open() { return uart_->open(); }

void InfantrySerialTransport::close() { uart_->close(); }

bool InfantrySerialTransport::isOpen() const { return uart_->isOpen(); }

float InfantrySerialTransport::finiteOrZero(double value) {
  return std::isfinite(value) ? static_cast<float>(value) : 0.0F;
}

double InfantrySerialTransport::feedbackAngleToRad(float value) const {
  return config_.feedback_angles_in_degrees ? static_cast<double>(value) * kDegToRad
                                            : static_cast<double>(value);
}

float InfantrySerialTransport::commandAngle(double rad) const {
  const double value = config_.command_angles_in_degrees ? rad * kRadToDeg : rad;
  return finiteOrZero(value);
}

template <std::size_t Capacity>
bool InfantrySerialTransport::sendCommandPacket(const GimbalCommand& command) {
  const GimbalCommand wire_command = sanitizeInfantryCommandForTransport(command);
  FixedPacket<Capacity> packet;
  const std::uint8_t fire = (config_.allow_fire && wire_command.fire_advice) ? 1 : 0;
  packet.load(fire, 1);
  packet.load(commandAngle(wire_command.pitch), 2);
  packet.load(commandAngle(wire_command.yaw), 6);
  packet.load(finiteOrZero(wire_command.distance), 10);
  if constexpr (Capacity >= 24) {
    packet.load(commandAngle(wire_command.pitch_vel), 14);
    packet.load(commandAngle(wire_command.yaw_vel), 18);
  }
  if constexpr (Capacity >= 32) {
    const double pitch_tail =
        config_.tail_fields == Infantry32TailFields::kDuplicateVelocity
            ? wire_command.pitch_vel
            : wire_command.pitch_acc;
    const double yaw_tail =
        config_.tail_fields == Infantry32TailFields::kDuplicateVelocity
            ? wire_command.yaw_vel
            : wire_command.yaw_acc;
    packet.load(commandAngle(pitch_tail), 22);
    packet.load(commandAngle(yaw_tail), 26);
  }
  packet.setCrc();
  return uart_->writeAll(packet.data(), packet.size());
}

bool InfantrySerialTransport::sendCommand(const GimbalCommand& command) {
  if (!isOpen() && !open()) return false;
  switch (config_.tx_layout) {
    case InfantryPacketLayout::kInfantry16:
      return sendCommandPacket<16>(command);
    case InfantryPacketLayout::kInfantry24:
      return sendCommandPacket<24>(command);
    case InfantryPacketLayout::kInfantry32:
      return sendCommandPacket<32>(command);
  }
  return false;
}

template <std::size_t Capacity>
bool InfantrySerialTransport::readFeedbackPacket(SerialFeedback& feedback) {
  FixedPacket<Capacity> packet;
  if (!FixedPacket<Capacity>::takeFromBuffer(rx_buffer_, packet)) return false;

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
  return true;
}

bool InfantrySerialTransport::readFeedback(SerialFeedback& feedback) {
  if (!isOpen() && !open()) return false;

  std::uint8_t tmp[64]{};
  const int n = uart_->readSome(tmp, sizeof(tmp), config_.read_timeout_ms);
  if (n < 0) {
    rx_buffer_.clear();
    close();
    return false;
  }
  for (int i = 0; i < n; ++i) rx_buffer_.push_back(tmp[i]);
  if (rx_buffer_.size() > 32 * 16) rx_buffer_.clear();

  switch (config_.rx_layout) {
    case InfantryPacketLayout::kInfantry16:
      return readFeedbackPacket<16>(feedback);
    case InfantryPacketLayout::kInfantry24:
      return readFeedbackPacket<24>(feedback);
    case InfantryPacketLayout::kInfantry32:
      return readFeedbackPacket<32>(feedback);
  }
  return false;
}

}  // namespace hfut::io
