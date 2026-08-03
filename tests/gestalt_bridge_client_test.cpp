#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

#include <lz4.h>

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/bridge_protocol.hpp"
#include "io/gestalt/gestalt_bridge_client.hpp"
#include "io/gestalt/gestalt_latest_frame_receiver.hpp"
#include "io/gestalt/gestalt_protocol.hpp"

namespace {

bool sendExact(int fd, const void* data, size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  size_t sent = 0;
  while (sent < size) {
    const ssize_t count = ::send(fd, bytes + sent, size - sent, 0);
    if (count <= 0) return false;
    sent += static_cast<size_t>(count);
  }
  return true;
}

bool receiveExact(int fd, void* data, size_t size) {
  auto* bytes = static_cast<uint8_t*>(data);
  size_t received = 0;
  while (received < size) {
    const ssize_t count = ::recv(fd, bytes + received, size - received, 0);
    if (count <= 0) return false;
    received += static_cast<size_t>(count);
  }
  return true;
}

bool writeExact(int fd, const void* data, size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  size_t written = 0;
  while (written < size) {
    const ssize_t count = ::write(fd, bytes + written, size - written);
    if (count <= 0) return false;
    written += static_cast<size_t>(count);
  }
  return true;
}

bool readExact(int fd, void* data, size_t size) {
  auto* bytes = static_cast<uint8_t*>(data);
  size_t read_size = 0;
  while (read_size < size) {
    const ssize_t count = ::read(fd, bytes + read_size, size - read_size);
    if (count <= 0) return false;
    read_size += static_cast<size_t>(count);
  }
  return true;
}

bool near(double lhs, double rhs, double epsilon = 1e-9) {
  return std::abs(lhs - rhs) <= epsilon;
}

bool testStdioTransport() {
  int frame_pipe[2];
  int command_pipe[2];
  if (::pipe(frame_pipe) != 0 || ::pipe(command_pipe) != 0) return false;

  bool ok = false;
  {
    hfut::io::GestaltBridgeClient bridge(frame_pipe[0], command_pipe[1], 0);
    hfut::io::GestaltLatestFrameReceiver receiver(bridge);

    const uint8_t pixels[4] = {7, 8, 9, 255};
    hfut::gestalt::FrameMetadata metadata{};
    metadata.seq = 100;
    metadata.capture_time_s = 20.0;
    metadata.width = 1;
    metadata.height = 1;
    metadata.row_bytes = 4;
    metadata.pixel_format = static_cast<uint32_t>(hfut::gestalt::PixelFormat::bgra8);
    metadata.pixel_bytes = sizeof(pixels);
    metadata.horizontal_fov_degrees = 90.0;
    metadata.camera_quaternion_ue_xyzw[3] = 1.0;
    metadata.view_actor_unique_id = 2;
    metadata.takeover_target_unique_id = 2;
    metadata.takeover_player_id = 0;
    metadata.takeover_epoch = 1;
    metadata.identity_flags = 0x7;

    hfut::gestalt::Envelope envelope{};
    std::memcpy(envelope.magic, hfut::gestalt::kEnvelopeMagic, 8);
    envelope.version = hfut::gestalt::kProtocolVersion;
    envelope.type = static_cast<uint32_t>(hfut::gestalt::MessageType::frame);
    envelope.payload_size = sizeof(metadata) + sizeof(pixels);
    envelope.seq = metadata.seq;

    hfut::CameraFrame frame;
    const bool frame_written =
        writeExact(frame_pipe[1], &envelope, sizeof(envelope)) &&
        writeExact(frame_pipe[1], &metadata, sizeof(metadata)) &&
        writeExact(frame_pipe[1], pixels, sizeof(pixels));
    const bool frame_read = receiver.readLatest(frame, std::chrono::seconds(1));

    hfut::GimbalCommand command;
    command.yaw = 0.5;
    command.pitch = -0.25;
    command.mode = hfut::GimbalMode::normal_measurement;
    const bool command_written = bridge.send(command, frame.sim_time_s);
    hfut::gestalt::Envelope command_envelope{};
    hfut::bridge::CommandPacket command_packet{};
    const bool command_read =
        readExact(command_pipe[0], &command_envelope, sizeof(command_envelope)) &&
        readExact(command_pipe[0], &command_packet, sizeof(command_packet));

    ok = frame_written && frame_read && frame.seq == 100 &&
        frame.image.at<cv::Vec3b>(0, 0) == cv::Vec3b(7, 8, 9) &&
        command_written && command_read && near(command_packet.yaw, 0.5) &&
        near(command_packet.pitch, -0.25) && bridge.usesStdioTransport();
    ::close(frame_pipe[1]);
  }
  ::close(command_pipe[0]);
  return ok;
}

}  // namespace

int main() {
  const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listener < 0) return 1;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (::bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
      ::listen(listener, 1) != 0) {
    ::close(listener);
    return 2;
  }
  socklen_t address_size = sizeof(address);
  ::getsockname(listener, reinterpret_cast<sockaddr*>(&address), &address_size);
  const uint16_t port = ntohs(address.sin_port);

  bool server_ok = false;
  std::thread server([&] {
    const int client = ::accept(listener, nullptr, nullptr);
    if (client < 0) return;

    const uint8_t pixels[8] = {
        10, 20, 30, 255,
        40, 50, 60, 255,
    };
    char compressed_pixels[LZ4_COMPRESSBOUND(sizeof(pixels))];
    const int compressed_size = LZ4_compress_default(
        reinterpret_cast<const char*>(pixels), compressed_pixels,
        sizeof(pixels), sizeof(compressed_pixels));
    if (compressed_size <= 0) {
      ::close(client);
      return;
    }
    hfut::gestalt::FrameMetadata metadata{};
    metadata.seq = 42;
    metadata.capture_time_s = 12.5;
    metadata.world_time_s = 3.0;
    metadata.width = 2;
    metadata.height = 1;
    metadata.row_bytes = 8;
    metadata.pixel_format = static_cast<uint32_t>(hfut::gestalt::PixelFormat::lz4_bgra8);
    metadata.pixel_bytes = static_cast<uint64_t>(compressed_size);
    metadata.horizontal_fov_degrees = 90.0;
    metadata.camera_quaternion_ue_xyzw[3] = 1.0;
    metadata.writer_process_id = 100;
    metadata.view_actor_unique_id = 9;
    metadata.takeover_target_unique_id = 9;
    metadata.takeover_player_id = 0;
    metadata.takeover_attribute_map_id = 77;
    metadata.takeover_epoch = 4;
    metadata.identity_flags = 0x7;

    for (uint64_t sequence = 42; sequence <= 44; ++sequence) {
      metadata.seq = sequence;
      metadata.capture_time_s = 12.5 + static_cast<double>(sequence - 42) / 60.0;
      hfut::gestalt::Envelope frame_envelope{};
      std::memcpy(frame_envelope.magic, hfut::gestalt::kEnvelopeMagic, 8);
      frame_envelope.version = hfut::gestalt::kProtocolVersion;
      frame_envelope.type = static_cast<uint32_t>(hfut::gestalt::MessageType::frame);
      frame_envelope.payload_size = sizeof(metadata) + metadata.pixel_bytes;
      frame_envelope.seq = metadata.seq;
      if (!sendExact(client, &frame_envelope, sizeof(frame_envelope)) ||
          !sendExact(client, &metadata, sizeof(metadata)) ||
          !sendExact(client, compressed_pixels, static_cast<size_t>(compressed_size))) {
        ::close(client);
        return;
      }
    }

    hfut::gestalt::Envelope command_envelope{};
    hfut::bridge::CommandPacket command{};
    server_ok = receiveExact(client, &command_envelope, sizeof(command_envelope)) &&
                receiveExact(client, &command, sizeof(command)) &&
                std::memcmp(command_envelope.magic, hfut::gestalt::kEnvelopeMagic, 8) == 0 &&
                command_envelope.type ==
                    static_cast<uint32_t>(hfut::gestalt::MessageType::command) &&
                command_envelope.payload_size == sizeof(command) &&
                std::memcmp(command.magic, hfut::bridge::kCommandMagic, 8) == 0 &&
                near(command.yaw, 0.25) && near(command.pitch, -0.1) &&
                command.fire_advice == 1 &&
                command.mode == static_cast<int8_t>(hfut::GimbalMode::normal_measurement);
    ::close(client);
  });

  hfut::io::GestaltBridgeClient bridge("127.0.0.1", port, 0);
  hfut::io::GestaltLatestFrameReceiver receiver(bridge);
  const auto drop_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (receiver.droppedFrames() < 2 && std::chrono::steady_clock::now() < drop_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  hfut::CameraFrame frame;
  const bool frame_ok = receiver.readLatest(frame, std::chrono::seconds(2));
  const bool decoded_ok = frame_ok && frame.seq == 44 && receiver.droppedFrames() == 2 &&
      frame.image.rows == 1 &&
      frame.image.cols == 2 && frame.image.at<cv::Vec3b>(0, 0) == cv::Vec3b(10, 20, 30) &&
      near(frame.intrinsics.fx, 1.0) && near(frame.gimbal_yaw, 0.0) &&
      near(frame.gimbal_pitch, 0.0);

  hfut::GimbalCommand command;
  command.yaw = 0.25;
  command.pitch = -0.1;
  command.fire_advice = true;
  command.mode = hfut::GimbalMode::normal_measurement;
  const bool command_ok = bridge.send(command, frame.sim_time_s);
  server.join();
  ::close(listener);

  if (!decoded_ok || !command_ok || !server_ok) {
    std::fprintf(stderr, "gestalt bridge test failed: frame=%d command=%d server=%d\n",
                 decoded_ok, command_ok, server_ok);
    return 3;
  }
  if (!testStdioTransport()) {
    std::fprintf(stderr, "gestalt stdio transport test failed\n");
    return 4;
  }
  return 0;
}
