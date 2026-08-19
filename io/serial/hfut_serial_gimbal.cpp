#include "io/serial/hfut_serial_gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <thread>
#include <utility>

namespace hfut::io {
namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

HfutSerialGimbal::HfutSerialGimbal(HfutSerialGimbalConfig config)
    : config_(std::move(config)), transport_(config_.serial) {
  config_.history_size = std::max<std::size_t>(config_.history_size, 2U);
}

HfutSerialGimbal::~HfutSerialGimbal() { close(); }

bool HfutSerialGimbal::open() {
  if (running_.load()) return true;
  {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    if (!transport_.open()) return false;
  }
  running_.store(true);
  receive_thread_ = std::thread(&HfutSerialGimbal::receiveLoop, this);
  return true;
}

void HfutSerialGimbal::close() {
  running_.store(false);
  if (receive_thread_.joinable()) receive_thread_.join();
  std::lock_guard<std::mutex> lock(transport_mutex_);
  transport_.close();
}

bool HfutSerialGimbal::isOpen() const {
  std::lock_guard<std::mutex> lock(transport_mutex_);
  return transport_.isOpen();
}

bool HfutSerialGimbal::latest(SerialFeedback& feedback, TimePoint* feedback_time) const {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (history_.empty()) return false;
  const auto& sample = history_.back();
  feedback = sample.feedback;
  if (feedback_time) *feedback_time = sample.time;
  return true;
}

bool HfutSerialGimbal::sampleAt(TimePoint frame_time, SerialFeedback& feedback,
                                TimePoint* feedback_time) const {
  const auto target_time = frame_time -
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(config_.timestamp_offset_s));

  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (history_.empty()) return false;
  if (!std::isfinite(config_.max_sample_age_s) || config_.max_sample_age_s <= 0.0) return false;

  const auto tooFar = [&](TimePoint a, TimePoint b) {
    return std::chrono::duration<double>(a > b ? a - b : b - a).count() >
           config_.max_sample_age_s;
  };

  if (history_.size() == 1 || target_time <= history_.front().time) {
    if (tooFar(target_time, history_.front().time)) return false;
    feedback = history_.front().feedback;
    if (feedback_time) *feedback_time = history_.front().time;
    return true;
  }
  if (target_time >= history_.back().time) {
    if (tooFar(target_time, history_.back().time)) return false;
    feedback = history_.back().feedback;
    if (feedback_time) *feedback_time = history_.back().time;
    return true;
  }

  const auto upper = std::upper_bound(
      history_.begin(), history_.end(), target_time,
      [](TimePoint time, const FeedbackSample& sample) { return time < sample.time; });
  if (upper == history_.begin() || upper == history_.end()) return false;
  const auto lower = std::prev(upper);
  const double duration_s = std::chrono::duration<double>(upper->time - lower->time).count();
  const double alpha = duration_s > 1e-9
                           ? std::clamp(
                                 std::chrono::duration<double>(target_time - lower->time).count() /
                                     duration_s,
                                 0.0, 1.0)
                           : 0.0;

  feedback = lower->feedback;
  feedback.roll_rad = lerp(lower->feedback.roll_rad, upper->feedback.roll_rad, alpha);
  feedback.yaw_rad = lower->feedback.yaw_rad +
                     alpha * normalizeAngle(upper->feedback.yaw_rad - lower->feedback.yaw_rad);
  feedback.pitch_rad = lerp(lower->feedback.pitch_rad, upper->feedback.pitch_rad, alpha);
  feedback.bullet_speed = lerp(lower->feedback.bullet_speed, upper->feedback.bullet_speed, alpha);
  feedback.chassis_vx = lerp(lower->feedback.chassis_vx, upper->feedback.chassis_vx, alpha);
  feedback.chassis_vy = lerp(lower->feedback.chassis_vy, upper->feedback.chassis_vy, alpha);
  feedback.chassis_wz = lerp(lower->feedback.chassis_wz, upper->feedback.chassis_wz, alpha);
  if (feedback_time) *feedback_time = target_time;
  return true;
}

bool HfutSerialGimbal::send(const GimbalCommand& command) {
  send_pending_.store(true);
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    ok = transport_.sendCommand(adaptCommandForWire(command));
  }
  send_pending_.store(false);
  return ok;
}

const std::string& HfutSerialGimbal::errorMessage() const { return transport_.errorMessage(); }

void HfutSerialGimbal::receiveLoop() {
  while (running_.load()) {
    if (send_pending_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    SerialFeedback feedback;
    bool received = false;
    {
      std::unique_lock<std::mutex> lock(transport_mutex_, std::try_to_lock);
      if (!lock.owns_lock()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      received = transport_.readFeedback(feedback);
    }
    if (received) pushFeedback(Clock::now(), feedback);
  }
}

void HfutSerialGimbal::pushFeedback(TimePoint receive_time, const SerialFeedback& feedback) {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  history_.push_back({receive_time, feedback});
  while (history_.size() > config_.history_size) history_.pop_front();
}

GimbalCommand HfutSerialGimbal::adaptCommandForWire(const GimbalCommand& command) const {
  GimbalCommand wire = command;
  if (!config_.send_velocity) {
    wire.yaw_vel = 0.0;
    wire.pitch_vel = 0.0;
  }
  if (!config_.send_acceleration) {
    wire.yaw_acc = 0.0;
    wire.pitch_acc = 0.0;
  }
  return wire;
}

double HfutSerialGimbal::lerp(double a, double b, double alpha) { return a + alpha * (b - a); }

double HfutSerialGimbal::normalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

}  // namespace hfut::io
