#ifndef HFUT_AUTO_AIM_UART_TRANSPORT_HPP
#define HFUT_AUTO_AIM_UART_TRANSPORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace hfut::io {

class UartTransport {
 public:
  explicit UartTransport(std::string device_path, int baudrate = 115200);
  ~UartTransport();

  bool open();
  void close();
  bool isOpen() const { return fd_ >= 0; }
  int readSome(void* buffer, std::size_t len, int timeout_ms);
  bool writeAll(const void* buffer, std::size_t len);
  const std::string& errorMessage() const { return error_message_; }

 private:
  bool configure();
  static bool baudrateToTermios(int baudrate, unsigned int& speed);

  std::string device_path_;
  int baudrate_{115200};
  int fd_{-1};
  std::string error_message_;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_UART_TRANSPORT_HPP
