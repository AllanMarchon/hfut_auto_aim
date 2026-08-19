#include "io/serial/uart_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <utility>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace hfut::io {

UartTransport::UartTransport(std::string device_path, int baudrate)
    : device_path_(std::move(device_path)), baudrate_(baudrate) {}

UartTransport::~UartTransport() { close(); }

bool UartTransport::baudrateToTermios(int baudrate, unsigned int& speed) {
#if defined(__unix__) || defined(__APPLE__)
  switch (baudrate) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 115200: speed = B115200; return true;
#ifdef B230400
    case 230400: speed = B230400; return true;
#endif
#ifdef B460800
    case 460800: speed = B460800; return true;
#endif
#ifdef B921600
    case 921600: speed = B921600; return true;
#endif
    default: return false;
  }
#else
  (void)baudrate;
  (void)speed;
  return false;
#endif
}

bool UartTransport::open() {
#if defined(__unix__) || defined(__APPLE__)
  if (isOpen()) return true;
  fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error_message_ = "open failed for " + device_path_ + ": " + std::strerror(errno);
    return false;
  }
  if (!configure()) {
    close();
    return false;
  }
  return true;
#else
  error_message_ = "UartTransport is only implemented for POSIX platforms";
  return false;
#endif
}

void UartTransport::close() {
#if defined(__unix__) || defined(__APPLE__)
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

bool UartTransport::configure() {
#if defined(__unix__) || defined(__APPLE__)
  termios options{};
  if (::tcgetattr(fd_, &options) != 0) {
    error_message_ = "tcgetattr failed: " + std::string(std::strerror(errno));
    return false;
  }

  unsigned int speed = 0;
  if (!baudrateToTermios(baudrate_, speed)) {
    error_message_ = "unsupported baudrate: " + std::to_string(baudrate_);
    return false;
  }

  ::cfsetispeed(&options, speed);
  ::cfsetospeed(&options, speed);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  options.c_cflag &= ~CRTSCTS;
#endif
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  options.c_oflag &= ~OPOST;
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = 0;

  ::tcflush(fd_, TCIFLUSH);
  if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
    error_message_ = "tcsetattr failed: " + std::string(std::strerror(errno));
    return false;
  }
  return true;
#else
  return false;
#endif
}

int UartTransport::readSome(void* buffer, std::size_t len, int timeout_ms) {
#if defined(__unix__) || defined(__APPLE__)
  if (fd_ < 0) return -1;
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int poll_ret = ::poll(&pfd, 1, timeout_ms);
  if (poll_ret == 0) return 0;
  if (poll_ret < 0) {
    if (errno == EINTR) return 0;
    error_message_ = "poll failed: " + std::string(std::strerror(errno));
    return -1;
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    error_message_ = "serial poll reported disconnected/error";
    return -1;
  }
  const auto n = ::read(fd_, buffer, len);
  if (n < 0) {
    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    error_message_ = "read failed: " + std::string(std::strerror(errno));
    return -1;
  }
  return static_cast<int>(n);
#else
  (void)buffer;
  (void)len;
  (void)timeout_ms;
  return -1;
#endif
}

bool UartTransport::writeAll(const void* buffer, std::size_t len, int timeout_ms) {
#if defined(__unix__) || defined(__APPLE__)
  if (fd_ < 0) return false;
  if (len == 0) return true;
  const auto* bytes = static_cast<const std::uint8_t*>(buffer);
  std::size_t written = 0;
  const bool finite_timeout = timeout_ms >= 0;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(std::max(timeout_ms, 0));

  const auto remainingTimeoutMs = [&]() -> int {
    if (!finite_timeout) return -1;
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return 0;
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
    return static_cast<int>(std::max<long long>(remaining, 1));
  };

  while (written < len) {
    const auto n = ::write(fd_, bytes + written, len - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      error_message_ = "write failed: " + std::string(std::strerror(errno));
      return false;
    }

    const int wait_ms = remainingTimeoutMs();
    if (wait_ms == 0) {
      error_message_ = "write timed out after " + std::to_string(timeout_ms) +
                       " ms (" + std::to_string(written) + "/" +
                       std::to_string(len) + " bytes written)";
      return false;
    }

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLOUT;
    const int poll_ret = ::poll(&pfd, 1, wait_ms);
    if (poll_ret == 0) {
      error_message_ = "write timed out after " + std::to_string(timeout_ms) +
                       " ms (" + std::to_string(written) + "/" +
                       std::to_string(len) + " bytes written)";
      return false;
    }
    if (poll_ret < 0) {
      if (errno == EINTR) continue;
      error_message_ = "write poll failed: " + std::string(std::strerror(errno));
      return false;
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      error_message_ = "serial write poll reported disconnected/error";
      return false;
    }
  }
  return true;
#else
  (void)buffer;
  (void)len;
  (void)timeout_ms;
  return false;
#endif
}

}  // namespace hfut::io
