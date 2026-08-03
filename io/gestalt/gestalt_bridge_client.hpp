#ifndef HFUT_AUTO_AIM_GESTALT_BRIDGE_CLIENT_HPP
#define HFUT_AUTO_AIM_GESTALT_BRIDGE_CLIENT_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "hfut_auto_aim/camera_frame.hpp"
#include "hfut_auto_aim/gimbal_command.hpp"

namespace hfut::io {

class GestaltBridgeClient {
 public:
  GestaltBridgeClient(std::string host, uint16_t port, int expected_player_id = 0);
  GestaltBridgeClient(int read_fd, int write_fd, int expected_player_id = 0);
  ~GestaltBridgeClient();

  GestaltBridgeClient(const GestaltBridgeClient&) = delete;
  GestaltBridgeClient& operator=(const GestaltBridgeClient&) = delete;

  bool read(CameraFrame& frame, std::chrono::milliseconds timeout);
  bool send(const GimbalCommand& command, double frame_time_s);

  const std::string& host() const { return host_; }
  uint16_t port() const { return port_; }
  bool usesStdioTransport() const { return stream_transport_; }

 private:
  bool connect(std::chrono::milliseconds timeout);
  bool receiveExact(void* data, size_t size,
                    std::chrono::steady_clock::time_point deadline,
                    bool* timed_out = nullptr);
  bool sendExact(int socket_fd, const void* data, size_t size);
  void disconnect();

  std::string host_;
  uint16_t port_;
  int expected_player_id_;
  bool stream_transport_{false};
  mutable std::mutex socket_mutex_;
  int read_fd_{-1};
  int write_fd_{-1};
  uint64_t last_frame_seq_{0};
  uint64_t command_seq_{0};
  double previous_frame_time_s_{0.0};
  double previous_yaw_{0.0};
  double previous_pitch_{0.0};
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_GESTALT_BRIDGE_CLIENT_HPP
