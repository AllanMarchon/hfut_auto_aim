#ifndef HFUT_AUTO_AIM_INFANTRY32_SERIAL_HPP
#define HFUT_AUTO_AIM_INFANTRY32_SERIAL_HPP

#include <deque>
#include <memory>
#include <string>

#include "io/serial/fixed_packet.hpp"
#include "io/serial/gimbal_transport.hpp"
#include "io/serial/uart_transport.hpp"

namespace hfut::io {

struct Infantry32SerialConfig {
  std::string port = "/dev/ttyACM0";
  int baudrate = 115200;
  bool command_angles_in_degrees = false;
  bool feedback_angles_in_degrees = false;
  bool allow_fire = false;
  int read_timeout_ms = 20;
};

class Infantry32SerialTransport final : public GimbalTransport {
 public:
  explicit Infantry32SerialTransport(Infantry32SerialConfig config);

  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool readFeedback(SerialFeedback& feedback) override;
  bool sendCommand(const GimbalCommand& command) override;

  const std::string& errorMessage() const { return uart_->errorMessage(); }

 private:
  static float finiteOrZero(double value);
  double feedbackAngleToRad(float value) const;
  float commandAngle(double rad) const;

  Infantry32SerialConfig config_;
  std::unique_ptr<UartTransport> uart_;
  std::deque<std::uint8_t> rx_buffer_;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_INFANTRY32_SERIAL_HPP
