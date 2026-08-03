#include "gestalt_latest_frame_receiver.hpp"

#include <utility>

namespace hfut::io {

GestaltLatestFrameReceiver::GestaltLatestFrameReceiver(GestaltBridgeClient& client)
    : client_(client), receive_thread_(&GestaltLatestFrameReceiver::receiveLoop, this) {}

GestaltLatestFrameReceiver::~GestaltLatestFrameReceiver() {
  stop_.store(true);
  frame_ready_.notify_all();
  if (receive_thread_.joinable()) receive_thread_.join();
}

bool GestaltLatestFrameReceiver::readLatest(
    CameraFrame& frame, std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(frame_mutex_);
  if (!frame_ready_.wait_for(lock, timeout, [this] {
        return stop_.load() || published_generation_ > consumed_generation_;
      })) {
    return false;
  }
  if (published_generation_ <= consumed_generation_) return false;
  frame = std::move(latest_frame_);
  consumed_generation_ = published_generation_;
  return true;
}

void GestaltLatestFrameReceiver::receiveLoop() {
  while (!stop_.load()) {
    CameraFrame frame;
    if (!client_.read(frame, std::chrono::milliseconds(500))) continue;

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (published_generation_ > consumed_generation_) {
        dropped_frames_.fetch_add(1);
      }
      latest_frame_ = std::move(frame);
      ++published_generation_;
    }
    frame_ready_.notify_one();
  }
}

}  // namespace hfut::io
