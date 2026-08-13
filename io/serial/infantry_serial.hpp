#ifndef HFUT_AUTO_AIM_INFANTRY_SERIAL_HPP
#define HFUT_AUTO_AIM_INFANTRY_SERIAL_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "io/serial/fixed_packet.hpp"
#include "io/serial/gimbal_transport.hpp"
#include "io/serial/uart_transport.hpp"

namespace hfut::io {

enum class InfantryPacketLayout {
  kInfantry16,
  kInfantry24,
  kInfantry32,
};

enum class Infantry32TailFields {
  kAcceleration,
  kDuplicateVelocity,
};

struct InfantrySerialConfig {
  std::string port = "/dev/ttyACM0";
  int baudrate = 115200;
  InfantryPacketLayout tx_layout = InfantryPacketLayout::kInfantry24;
  InfantryPacketLayout rx_layout = InfantryPacketLayout::kInfantry24;
  Infantry32TailFields tail_fields = Infantry32TailFields::kDuplicateVelocity;
  bool command_angles_in_degrees = true;
  bool feedback_angles_in_degrees = true;
  bool allow_fire = false;
  int read_timeout_ms = 20;
};

GimbalCommand sanitizeInfantryCommandForTransport(const GimbalCommand& command);

bool parseInfantryPacketLayout(const std::string& value,
                               InfantryPacketLayout& layout);
const char* infantryPacketLayoutName(InfantryPacketLayout layout);

bool parseInfantry32TailFields(const std::string& value,
                               Infantry32TailFields& fields);
const char* infantry32TailFieldsName(Infantry32TailFields fields);

class InfantrySerialTransport final : public GimbalTransport {
 public:
  explicit InfantrySerialTransport(InfantrySerialConfig config);

  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool readFeedback(SerialFeedback& feedback) override;
  bool sendCommand(const GimbalCommand& command) override;

  const std::string& errorMessage() const { return uart_->errorMessage(); }

 private:
  template <std::size_t Capacity>
  bool sendCommandPacket(const GimbalCommand& command);

  template <std::size_t Capacity>
  bool readFeedbackPacket(SerialFeedback& feedback);

  static float finiteOrZero(double value);
  double feedbackAngleToRad(float value) const;
  float commandAngle(double rad) const;

  InfantrySerialConfig config_;
  std::unique_ptr<UartTransport> uart_;
  std::deque<std::uint8_t> rx_buffer_;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_INFANTRY_SERIAL_HPP
