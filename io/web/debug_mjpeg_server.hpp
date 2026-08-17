#ifndef HFUT_AUTO_AIM_DEBUG_MJPEG_SERVER_HPP_
#define HFUT_AUTO_AIM_DEBUG_MJPEG_SERVER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace hfut::io {

struct DebugMjpegServerConfig {
  std::string host{"0.0.0.0"};
  uint16_t port{8080};
  int jpeg_quality{80};
  int max_width{960};
};

struct DebugMjpegStatus {
  uint64_t frames{0};
  double fps{0.0};
  double latency_ms{0.0};
  int detections{0};
  int poses{0};
  int armors{0};
  int tracked{0};
  std::string selected_id{"none"};
  std::string track_state{"none"};
  std::string reason{"none"};
  int mode{-1};
  double feedback_yaw_deg{0.0};
  double feedback_pitch_deg{0.0};
  double command_yaw_deg{0.0};
  double command_pitch_deg{0.0};
  double command_yaw_vel_rad_s{0.0};
  double command_pitch_vel_rad_s{0.0};
  double command_yaw_acc_rad_s2{0.0};
  double command_pitch_acc_rad_s2{0.0};
  double distance_m{0.0};
  double pnp_first_distance_m{0.0};
  double yaw_error_deg{0.0};
  double pitch_error_deg{0.0};
  double feedback_age_ms{0.0};
  bool fire_advice{false};
  bool fire{false};
  bool dry_run{false};
  bool fire_enabled{false};
  std::string enemy_color;
  std::string camera_backend;
  std::string serial_tx;
  std::string serial_rx;
};

class DebugMjpegServer {
 public:
  explicit DebugMjpegServer(DebugMjpegServerConfig config = {});
  ~DebugMjpegServer();

  DebugMjpegServer(const DebugMjpegServer&) = delete;
  DebugMjpegServer& operator=(const DebugMjpegServer&) = delete;

  bool start();
  void stop();
  bool running() const { return running_.load(); }
  std::string errorMessage() const;
  std::string url() const;

  void publish(const cv::Mat& image, const DebugMjpegStatus& status);

 private:
  void acceptLoop();
  void handleClient(int client_fd);
  bool bindListenSocket();

  DebugMjpegServerConfig config_;
  std::atomic<bool> running_{false};
  int server_fd_{-1};
  std::thread accept_thread_;
  mutable std::mutex state_mutex_;
  std::condition_variable frame_cv_;
  std::vector<uint8_t> latest_jpeg_;
  DebugMjpegStatus latest_status_;
  uint64_t frame_version_{0};
  std::string error_message_;
  std::mutex clients_mutex_;
  std::vector<std::thread> client_threads_;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_DEBUG_MJPEG_SERVER_HPP_
