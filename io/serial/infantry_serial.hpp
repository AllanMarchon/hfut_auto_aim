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

// 仅描述上位机发送的控制包；电控反馈使用独立固定格式。
enum class InfantryCommandLayout {
  kInfantry16,
  kInfantry24,
  kInfantry32,
};

inline constexpr std::size_t kInfantryFeedbackPacketSize = 24;
inline constexpr char kInfantryFeedbackProtocol[] = "infantry_feedback_24";

enum class Infantry32TailFields {
  kAcceleration,
  kDuplicateVelocity,
};

enum class InfantryStatusByte {
  kFireAdvice,
  kCommandMode,
};

struct InfantrySerialConfig {
  std::string port = "/dev/ttyACM0";
  int baudrate = 115200;
  InfantryCommandLayout tx_layout = InfantryCommandLayout::kInfantry32;
  Infantry32TailFields tail_fields = Infantry32TailFields::kDuplicateVelocity;
  InfantryStatusByte status_byte = InfantryStatusByte::kFireAdvice;
  bool command_angles_in_degrees = false;
  bool feedback_angles_in_degrees = false;
  bool allow_fire = false;
  int read_timeout_ms = 20;
  int write_timeout_ms = 5;
};

GimbalCommand sanitizeInfantryCommandForTransport(const GimbalCommand& command);

bool parseInfantryCommandLayout(const std::string& value,
                                InfantryCommandLayout& layout);
const char* infantryCommandLayoutName(InfantryCommandLayout layout);

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

  const std::string& errorMessage() const override { return uart_->errorMessage(); }

 private:
  template <std::size_t Capacity>
  bool sendCommandPacket(const GimbalCommand& command);

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
