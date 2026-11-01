#ifndef HFUT_AUTO_AIM_SENTRY_SERIAL_HPP
#define HFUT_AUTO_AIM_SENTRY_SERIAL_HPP

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "io/serial/gimbal_transport.hpp"
#include "io/serial/uart_transport.hpp"

namespace hfut::io {

struct SentrySerialConfig {
  std::string port = "/dev/ttyACM0";
  int baudrate = 115200;
  bool allow_fire = false;
  int read_timeout_ms = 20;
  int write_timeout_ms = 5;
};

class SentrySerialTransport final : public GimbalTransport {
 public:
  explicit SentrySerialTransport(SentrySerialConfig config);

  bool open() override;
  void close() override;
  bool isOpen() const override;
  bool readFeedback(SerialFeedback& feedback) override;
  bool sendCommand(const GimbalCommand& command) override;
  const std::string& errorMessage() const override;

 private:
  bool readFeedbackPacket(SerialFeedback& feedback);
  static float finiteOrZero(double value);
  static float angleToWire(double radians);

  SentrySerialConfig config_;
  std::unique_ptr<UartTransport> uart_;
  std::deque<std::uint8_t> rx_buffer_;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_SENTRY_SERIAL_HPP
