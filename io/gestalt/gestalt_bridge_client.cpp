#include "gestalt_bridge_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <lz4.h>
#include <opencv2/imgproc.hpp>

#include "io/bridge_protocol.hpp"
#include "io/gestalt/gestalt_protocol.hpp"

namespace hfut::io {
namespace {

constexpr double kDegToRad = M_PI / 180.0;

bool validEnvelope(const gestalt::Envelope& envelope, gestalt::MessageType type) {
  return std::memcmp(envelope.magic, gestalt::kEnvelopeMagic, 8) == 0 &&
         envelope.version == gestalt::kProtocolVersion &&
         envelope.type == static_cast<uint32_t>(type);
}

double normalizeAngle(double angle) {
  return std::remainder(angle, 2.0 * M_PI);
}

bool isLz4PixelFormat(gestalt::PixelFormat format) {
  return format == gestalt::PixelFormat::lz4_bgra8 ||
         format == gestalt::PixelFormat::lz4_rgba8 ||
         format == gestalt::PixelFormat::lz4_a2b10g10r10;
}

gestalt::PixelFormat uncompressedPixelFormat(gestalt::PixelFormat format) {
  switch (format) {
    case gestalt::PixelFormat::lz4_bgra8:
      return gestalt::PixelFormat::bgra8;
    case gestalt::PixelFormat::lz4_rgba8:
      return gestalt::PixelFormat::rgba8;
    case gestalt::PixelFormat::lz4_a2b10g10r10:
      return gestalt::PixelFormat::a2b10g10r10;
    default:
      return format;
  }
}

}  // namespace

GestaltBridgeClient::GestaltBridgeClient(
    std::string host, uint16_t port, int expected_player_id)
    : host_(std::move(host)), port_(port), expected_player_id_(expected_player_id) {}

GestaltBridgeClient::GestaltBridgeClient(
    int read_fd, int write_fd, int expected_player_id)
    : host_("stdio"), port_(0), expected_player_id_(expected_player_id),
      stream_transport_(true), read_fd_(read_fd), write_fd_(write_fd) {
  if (read_fd < 0 || write_fd < 0) {
    throw std::invalid_argument("Gestalt stdio file descriptors must be non-negative");
  }
}

GestaltBridgeClient::~GestaltBridgeClient() { disconnect(); }

void GestaltBridgeClient::disconnect() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  const int read_fd = read_fd_;
  const int write_fd = write_fd_;
  read_fd_ = -1;
  write_fd_ = -1;
  if (read_fd >= 0) ::close(read_fd);
  if (write_fd >= 0 && write_fd != read_fd) ::close(write_fd);
  previous_frame_time_s_ = 0.0;
}

bool GestaltBridgeClient::connect(std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (read_fd_ >= 0 && write_fd_ >= 0) return true;
  if (stream_transport_) return false;

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* addresses = nullptr;
  const std::string service = std::to_string(port_);
  const int lookup = ::getaddrinfo(host_.c_str(), service.c_str(), &hints, &addresses);
  if (lookup != 0) {
    std::fprintf(stderr, "[gestalt] cannot resolve %s: %s\n", host_.c_str(),
                 gai_strerror(lookup));
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (addrinfo* address = addresses; address; address = address->ai_next) {
    const int fd = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (fd < 0) continue;

    const int send_timeout_ms = 1000;
    timeval send_timeout{send_timeout_ms / 1000, (send_timeout_ms % 1000) * 1000};
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
    const int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags >= 0) ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int result = ::connect(fd, address->ai_addr, address->ai_addrlen);
    if (result < 0 && errno == EINPROGRESS) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
      pollfd descriptor{fd, POLLOUT, 0};
      result = remaining.count() > 0
                   ? ::poll(&descriptor, 1, static_cast<int>(remaining.count()))
                   : 0;
      int socket_error = 0;
      socklen_t error_size = sizeof(socket_error);
      if (result > 0 &&
          (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &error_size) != 0 ||
           socket_error != 0)) {
        result = -1;
      }
    }
    if (flags >= 0) ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    if (result >= 0) {
      read_fd_ = fd;
      write_fd_ = fd;
      std::printf("[gestalt] connected to Windows bridge at %s:%u\n", host_.c_str(),
                  static_cast<unsigned>(port_));
      break;
    }
    ::close(fd);
  }
  ::freeaddrinfo(addresses);
  return read_fd_ >= 0 && write_fd_ >= 0;
}

bool GestaltBridgeClient::receiveExact(
    void* data, size_t size, std::chrono::steady_clock::time_point deadline,
    bool* timed_out) {
  if (timed_out) *timed_out = false;
  int read_fd = -1;
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    read_fd = read_fd_;
  }
  if (read_fd < 0) return false;
  auto* output = static_cast<uint8_t*>(data);
  size_t received = 0;
  while (received < size) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      if (timed_out && received == 0) *timed_out = true;
      return false;
    }
    pollfd descriptor{read_fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
    if (ready == 0) {
      if (timed_out && received == 0) *timed_out = true;
      return false;
    }
    if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
    const ssize_t count = stream_transport_
        ? ::read(read_fd, output + received, size - received)
        : ::recv(read_fd, output + received, size - received, 0);
    if (count <= 0) return false;
    received += static_cast<size_t>(count);
  }
  return true;
}

bool GestaltBridgeClient::sendExact(int socket_fd, const void* data, size_t size) {
  const auto* input = static_cast<const uint8_t*>(data);
  size_t sent = 0;
  while (sent < size) {
    const ssize_t count = stream_transport_
        ? ::write(socket_fd, input + sent, size - sent)
        : ::send(socket_fd, input + sent, size - sent, MSG_NOSIGNAL);
    if (count <= 0) return false;
    sent += static_cast<size_t>(count);
  }
  return true;
}

bool GestaltBridgeClient::read(CameraFrame& frame, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  if (!connect(timeout)) return false;

  gestalt::Envelope envelope{};
  bool timed_out = false;
  if (!receiveExact(&envelope, sizeof(envelope), deadline, &timed_out)) {
    if (!timed_out) disconnect();
    return false;
  }
  if (!validEnvelope(envelope, gestalt::MessageType::frame) ||
      envelope.payload_size < sizeof(gestalt::FrameMetadata) ||
      envelope.payload_size > gestalt::kMaxFramePayload) {
    disconnect();
    return false;
  }

  gestalt::FrameMetadata metadata{};
  if (!receiveExact(&metadata, sizeof(metadata), deadline)) {
    disconnect();
    return false;
  }
  const uint64_t payload_pixels = envelope.payload_size - sizeof(metadata);
  const auto wire_format = static_cast<gestalt::PixelFormat>(metadata.pixel_format);
  const bool compressed = isLz4PixelFormat(wire_format);
  const uint64_t raw_pixel_bytes =
      static_cast<uint64_t>(metadata.row_bytes) * metadata.height;
  if (metadata.seq != envelope.seq || metadata.seq <= last_frame_seq_ ||
      metadata.pixel_bytes != payload_pixels || metadata.pixel_bytes == 0 ||
      metadata.width == 0 || metadata.height == 0 ||
      metadata.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      metadata.height > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      metadata.row_bytes != metadata.width * 4ULL ||
      raw_pixel_bytes == 0 || raw_pixel_bytes > gestalt::kMaxFramePayload ||
      (!compressed && metadata.pixel_bytes != raw_pixel_bytes) ||
      metadata.pixel_bytes > static_cast<uint64_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(metadata.capture_time_s) ||
      !std::isfinite(metadata.horizontal_fov_degrees) ||
      metadata.horizontal_fov_degrees <= 1.0 || metadata.horizontal_fov_degrees >= 179.0 ||
      (expected_player_id_ >= 0 && metadata.takeover_player_id != expected_player_id_) ||
      (metadata.identity_flags & 0x7U) != 0x7U ||
      metadata.view_actor_unique_id == 0 ||
      metadata.view_actor_unique_id != metadata.takeover_target_unique_id) {
    disconnect();
    return false;
  }

  std::vector<uint8_t> wire_pixels(static_cast<size_t>(metadata.pixel_bytes));
  if (!receiveExact(wire_pixels.data(), wire_pixels.size(), deadline)) {
    disconnect();
    return false;
  }

  std::vector<uint8_t> decompressed_pixels;
  const uint8_t* pixel_data = wire_pixels.data();
  if (compressed) {
    decompressed_pixels.resize(static_cast<size_t>(raw_pixel_bytes));
    const int decoded_size = LZ4_decompress_safe(
        reinterpret_cast<const char*>(wire_pixels.data()),
        reinterpret_cast<char*>(decompressed_pixels.data()),
        static_cast<int>(wire_pixels.size()),
        static_cast<int>(decompressed_pixels.size()));
    if (decoded_size != static_cast<int>(decompressed_pixels.size())) {
      disconnect();
      return false;
    }
    pixel_data = decompressed_pixels.data();
  }

  const int width = static_cast<int>(metadata.width);
  const int height = static_cast<int>(metadata.height);
  const cv::Mat source(height, width, CV_8UC4, const_cast<uint8_t*>(pixel_data),
                       metadata.row_bytes);
  cv::Mat decoded_image;
  switch (uncompressedPixelFormat(wire_format)) {
    case gestalt::PixelFormat::bgra8:
      cv::cvtColor(source, decoded_image, cv::COLOR_BGRA2BGR);
      break;
    case gestalt::PixelFormat::rgba8:
      cv::cvtColor(source, decoded_image, cv::COLOR_RGBA2BGR);
      break;
    case gestalt::PixelFormat::a2b10g10r10:
      decoded_image.create(height, width, CV_8UC3);
      for (int y = 0; y < height; ++y) {
        const auto* packed = reinterpret_cast<const uint32_t*>(pixel_data +
            static_cast<size_t>(y) * metadata.row_bytes);
        auto* bgr = decoded_image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; ++x) {
          const uint32_t value = packed[x];
          bgr[x][2] = static_cast<uint8_t>(((value >> 0) & 0x3ffU) * 255U / 1023U);
          bgr[x][1] = static_cast<uint8_t>(((value >> 10) & 0x3ffU) * 255U / 1023U);
          bgr[x][0] = static_cast<uint8_t>(((value >> 20) & 0x3ffU) * 255U / 1023U);
        }
      }
      break;
    default:
      disconnect();
      return false;
  }

  Eigen::Quaterniond q_camera_ue(
      metadata.camera_quaternion_ue_xyzw[3], metadata.camera_quaternion_ue_xyzw[0],
      metadata.camera_quaternion_ue_xyzw[1], metadata.camera_quaternion_ue_xyzw[2]);
  if (!std::isfinite(q_camera_ue.norm()) || q_camera_ue.norm() <= 1e-12) {
    disconnect();
    return false;
  }
  q_camera_ue.normalize();

  // UE world: X forward, Y right, Z up (left-handed). Control world: X
  // forward, Y left, Z up (right-handed). OpenCV camera: x right, y down,
  // z forward. The two reflections compose into a proper rotation.
  Eigen::Matrix3d ue_world_to_control;
  ue_world_to_control << 1, 0, 0, 0, -1, 0, 0, 0, 1;
  Eigen::Matrix3d cv_camera_to_ue_camera;
  cv_camera_to_ue_camera << 0, 0, 1, 1, 0, 0, 0, -1, 0;
  const Eigen::Matrix3d camera_to_control =
      ue_world_to_control * q_camera_ue.toRotationMatrix() * cv_camera_to_ue_camera;

  Eigen::Matrix3d camera_to_gimbal;
  camera_to_gimbal << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  const Eigen::Matrix3d gimbal_to_control = camera_to_control * camera_to_gimbal.transpose();
  const double yaw = std::atan2(gimbal_to_control(1, 0), gimbal_to_control(0, 0));
  const double pitch = std::atan2(
      gimbal_to_control(2, 0),
      std::hypot(gimbal_to_control(0, 0), gimbal_to_control(1, 0)));

  const double focal = (0.5 * metadata.width) /
      std::tan(0.5 * metadata.horizontal_fov_degrees * kDegToRad);
  frame = CameraFrame{};
  frame.input_mode = FrameInputMode::vision;
  frame.seq = metadata.seq;
  frame.sim_time_s = metadata.capture_time_s;
  frame.image = std::move(decoded_image);
  frame.intrinsics.width = width;
  frame.intrinsics.height = height;
  frame.intrinsics.fx = focal;
  frame.intrinsics.fy = focal;
  frame.intrinsics.cx = 0.5 * metadata.width;
  frame.intrinsics.cy = 0.5 * metadata.height;
  frame.q_cam2world = Eigen::Quaterniond(camera_to_control).normalized();
  // Transport frames stay shooter-centered here. bringup_sim applies the
  // common camera_to_barrel calibration shared by every bridge/input mode.
  frame.t_cam2world = Eigen::Vector3d::Zero();
  frame.gimbal_yaw = yaw;
  frame.gimbal_pitch = pitch;
  const double dt = metadata.capture_time_s - previous_frame_time_s_;
  if (previous_frame_time_s_ > 0.0 && dt > 1e-4 && dt < 0.5) {
    frame.gimbal_yaw_vel = normalizeAngle(yaw - previous_yaw_) / dt;
    frame.gimbal_pitch_vel = (pitch - previous_pitch_) / dt;
  }

  previous_frame_time_s_ = metadata.capture_time_s;
  previous_yaw_ = yaw;
  previous_pitch_ = pitch;
  last_frame_seq_ = metadata.seq;
  return true;
}

bool GestaltBridgeClient::send(const GimbalCommand& command, double frame_time_s) {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (write_fd_ < 0) return false;

  bridge::CommandPacket packet{};
  std::memcpy(packet.magic, bridge::kCommandMagic, 8);
  packet.version = bridge::kProtocolVersion;
  packet.seq = ++command_seq_;
  packet.sim_time_s = frame_time_s;
  packet.yaw = command.yaw;
  packet.yaw_diff = command.yaw_diff;
  packet.yaw_vel = command.yaw_vel;
  packet.yaw_acc = command.yaw_acc;
  packet.pitch = command.pitch;
  packet.pitch_diff = command.pitch_diff;
  packet.pitch_vel = command.pitch_vel;
  packet.pitch_acc = command.pitch_acc;
  packet.distance = command.distance;
  packet.fire_advice = command.fire_advice ? 1 : 0;
  packet.mode = static_cast<int8_t>(command.mode);

  gestalt::Envelope envelope{};
  std::memcpy(envelope.magic, gestalt::kEnvelopeMagic, 8);
  envelope.version = gestalt::kProtocolVersion;
  envelope.type = static_cast<uint32_t>(gestalt::MessageType::command);
  envelope.payload_size = sizeof(packet);
  envelope.seq = packet.seq;
  if (!sendExact(write_fd_, &envelope, sizeof(envelope)) ||
      !sendExact(write_fd_, &packet, sizeof(packet))) {
    if (!stream_transport_) ::shutdown(write_fd_, SHUT_RDWR);
    return false;
  }
  return true;
}

}  // namespace hfut::io
