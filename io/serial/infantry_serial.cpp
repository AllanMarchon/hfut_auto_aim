#include "io/serial/infantry_serial.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace hfut::io {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
}  // namespace

bool parseInfantryCommandLayout(const std::string& value,
                                InfantryCommandLayout& layout) {
  if (value == "infantry_24") {
    layout = InfantryCommandLayout::kInfantry24;
    return true;
  }
  if (value == "infantry_16") {
    layout = InfantryCommandLayout::kInfantry16;
    return true;
  }
  if (value == "infantry_32") {
    layout = InfantryCommandLayout::kInfantry32;
    return true;
  }
  return false;
}

const char* infantryCommandLayoutName(InfantryCommandLayout layout) {
  switch (layout) {
    case InfantryCommandLayout::kInfantry16: return "infantry_16";
    case InfantryCommandLayout::kInfantry24: return "infantry_24";
    case InfantryCommandLayout::kInfantry32: return "infantry_32";
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
  if (config_.status_byte == InfantryStatusByte::kCommandMode) {
    const std::int8_t mode = static_cast<std::int8_t>(wire_command.mode);
    packet.load(mode, 1);
  } else {
    const std::uint8_t fire = (config_.allow_fire && wire_command.fire_advice) ? 1 : 0;
    packet.load(fire, 1);
  }
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
  return uart_->writeAll(packet.data(), packet.size(), config_.write_timeout_ms);
}

bool InfantrySerialTransport::sendCommand(const GimbalCommand& command) {
  if (!isOpen() && !open()) return false;
  switch (config_.tx_layout) {
    case InfantryCommandLayout::kInfantry16:
      return sendCommandPacket<16>(command);
    case InfantryCommandLayout::kInfantry24:
      return sendCommandPacket<24>(command);
    case InfantryCommandLayout::kInfantry32:
      return sendCommandPacket<32>(command);
  }
  return false;
}

bool InfantrySerialTransport::readFeedbackPacket(SerialFeedback& feedback) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                "feedback requires 32-bit IEEE 754 floats");
  using FeedbackPacket = FixedPacket<kInfantryFeedbackPacketSize>;
  FeedbackPacket packet;
  if (!FeedbackPacket::takeFromBuffer(rx_buffer_, packet)) return false;

  std::uint8_t mode = 0;
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  float fire_delay_ms = 0.0F;
  packet.unload(mode, 1);
  packet.unload(roll, 2);
  packet.unload(pitch, 6);
  packet.unload(yaw, 10);
  packet.unload(fire_delay_ms, 14);

  feedback.received = true;
  feedback.mode = mode;
  feedback.roll_rad = feedbackAngleToRad(roll);
  feedback.pitch_rad = feedbackAngleToRad(pitch);
  feedback.yaw_rad = feedbackAngleToRad(yaw);
  feedback.fire_delay_s.reset();
  // 电控发送毫秒，内部保存秒；非法延迟不作为有效值，不影响姿态反馈。
  if (std::isfinite(fire_delay_ms) && fire_delay_ms >= 0.0F) {
    feedback.fire_delay_s = static_cast<double>(fire_delay_ms) / 1000.0;
  }
  return true;
}

bool InfantrySerialTransport::readFeedback(SerialFeedback& feedback) {
  if (!isOpen() && !open()) return false;

  std::uint8_t tmp[256]{};
  const int n = uart_->readSome(tmp, sizeof(tmp), config_.read_timeout_ms);
  if (n < 0) {
    rx_buffer_.clear();
    close();
    return false;
  }
  for (int i = 0; i < n; ++i) rx_buffer_.push_back(tmp[i]);
  if (rx_buffer_.size() > 1024) rx_buffer_.clear();

  bool got_latest = false;
  SerialFeedback parsed;
  // 电控只发送新版20字节反馈；一次收到多帧时取最后一帧。
  while (readFeedbackPacket(parsed)) {
    feedback = parsed;
    got_latest = true;
  }
  return got_latest;
}

}  // namespace hfut::io
