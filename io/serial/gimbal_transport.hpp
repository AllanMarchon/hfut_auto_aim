#ifndef HFUT_AUTO_AIM_GIMBAL_TRANSPORT_HPP
#define HFUT_AUTO_AIM_GIMBAL_TRANSPORT_HPP

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/serial/serial_feedback.hpp"

namespace hfut::io {

class GimbalTransport {
 public:
  virtual ~GimbalTransport() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual bool readFeedback(SerialFeedback& feedback) = 0;
  virtual bool sendCommand(const GimbalCommand& command) = 0;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_GIMBAL_TRANSPORT_HPP
