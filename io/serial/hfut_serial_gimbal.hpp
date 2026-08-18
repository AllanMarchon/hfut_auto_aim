#ifndef HFUT_AUTO_AIM_HFUT_SERIAL_GIMBAL_HPP
#define HFUT_AUTO_AIM_HFUT_SERIAL_GIMBAL_HPP

#include <atomic>
#include <chrono>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/serial/infantry_serial.hpp"
#include "io/serial/serial_feedback.hpp"

namespace hfut::io {

struct HfutSerialGimbalConfig {
  InfantrySerialConfig serial;
  std::size_t history_size = 1000;
  double timestamp_offset_s = 0.001;
  double max_sample_age_s = 0.05;
  bool send_velocity = true;
  bool send_acceleration = true;
};

class HfutSerialGimbal {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit HfutSerialGimbal(HfutSerialGimbalConfig config);
  ~HfutSerialGimbal();

  HfutSerialGimbal(const HfutSerialGimbal&) = delete;
  HfutSerialGimbal& operator=(const HfutSerialGimbal&) = delete;

  bool open();
  void close();
  bool isOpen() const;

  bool latest(SerialFeedback& feedback, TimePoint* feedback_time = nullptr) const;
  bool sampleAt(TimePoint frame_time, SerialFeedback& feedback,
                TimePoint* feedback_time = nullptr) const;
  bool send(const GimbalCommand& command);

  const std::string& errorMessage() const;

 private:
  struct FeedbackSample {
    TimePoint time;
    SerialFeedback feedback;
  };

  void receiveLoop();
  void pushFeedback(TimePoint receive_time, const SerialFeedback& feedback);
  GimbalCommand adaptCommandForWire(const GimbalCommand& command) const;

  static double lerp(double a, double b, double alpha);
  static double normalizeAngle(double angle);

  HfutSerialGimbalConfig config_;
  InfantrySerialTransport transport_;
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  mutable std::mutex transport_mutex_;
  mutable std::mutex feedback_mutex_;
  std::deque<FeedbackSample> history_;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_HFUT_SERIAL_GIMBAL_HPP
