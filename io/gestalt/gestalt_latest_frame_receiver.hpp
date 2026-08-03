#ifndef HFUT_AUTO_AIM_GESTALT_LATEST_FRAME_RECEIVER_HPP
#define HFUT_AUTO_AIM_GESTALT_LATEST_FRAME_RECEIVER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include "hfut_auto_aim/camera_frame.hpp"
#include "io/gestalt/gestalt_bridge_client.hpp"

namespace hfut::io {

class GestaltLatestFrameReceiver {
 public:
  explicit GestaltLatestFrameReceiver(GestaltBridgeClient& client);
  ~GestaltLatestFrameReceiver();

  GestaltLatestFrameReceiver(const GestaltLatestFrameReceiver&) = delete;
  GestaltLatestFrameReceiver& operator=(const GestaltLatestFrameReceiver&) = delete;

  bool readLatest(CameraFrame& frame, std::chrono::milliseconds timeout);
  uint64_t droppedFrames() const { return dropped_frames_.load(); }

 private:
  void receiveLoop();

  GestaltBridgeClient& client_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> dropped_frames_{0};
  std::thread receive_thread_;

  std::mutex frame_mutex_;
  std::condition_variable frame_ready_;
  CameraFrame latest_frame_;
  uint64_t published_generation_{0};
  uint64_t consumed_generation_{0};
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_GESTALT_LATEST_FRAME_RECEIVER_HPP
