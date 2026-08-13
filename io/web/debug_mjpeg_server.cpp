#include "debug_mjpeg_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace hfut::io {
namespace {

constexpr const char* kBoundary = "hfut_auto_aim_frame";

std::string escapeJson(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (char c : value) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

double finiteOrZero(double value) {
  return std::isfinite(value) ? value : 0.0;
}

std::string statusJson(const DebugMjpegStatus& status) {
  std::ostringstream out;
  out << '{'
      << "\"frames\":" << status.frames
      << ",\"fps\":" << finiteOrZero(status.fps)
      << ",\"latency_ms\":" << finiteOrZero(status.latency_ms)
      << ",\"detections\":" << status.detections
      << ",\"poses\":" << status.poses
      << ",\"armors\":" << status.armors
      << ",\"tracked\":" << status.tracked
      << ",\"selected_id\":\"" << escapeJson(status.selected_id) << "\""
      << ",\"track_state\":\"" << escapeJson(status.track_state) << "\""
      << ",\"reason\":\"" << escapeJson(status.reason) << "\""
      << ",\"mode\":" << status.mode
      << ",\"feedback_yaw_deg\":" << finiteOrZero(status.feedback_yaw_deg)
      << ",\"feedback_pitch_deg\":" << finiteOrZero(status.feedback_pitch_deg)
      << ",\"command_yaw_deg\":" << finiteOrZero(status.command_yaw_deg)
      << ",\"command_pitch_deg\":" << finiteOrZero(status.command_pitch_deg)
      << ",\"feedback_age_ms\":" << finiteOrZero(status.feedback_age_ms)
      << ",\"fire\":" << (status.fire ? "true" : "false")
      << ",\"dry_run\":" << (status.dry_run ? "true" : "false")
      << ",\"fire_enabled\":" << (status.fire_enabled ? "true" : "false")
      << ",\"enemy_color\":\"" << escapeJson(status.enemy_color) << "\""
      << ",\"camera_backend\":\"" << escapeJson(status.camera_backend) << "\""
      << ",\"serial_tx\":\"" << escapeJson(status.serial_tx) << "\""
      << ",\"serial_rx\":\"" << escapeJson(status.serial_rx) << "\""
      << '}';
  return out.str();
}

std::string indexHtml() {
  return R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HFUT Auto Aim Debug</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: Arial, sans-serif; }
    main { display: grid; grid-template-columns: minmax(0, 1fr) 340px; gap: 12px; padding: 12px; }
    img { width: 100%; height: auto; background: #000; border: 1px solid #333; }
    aside { background: #1b1b1b; border: 1px solid #333; padding: 12px; }
    h1 { font-size: 18px; margin: 0 0 10px; }
    p { color: #bbb; line-height: 1.5; margin: 8px 0; font-size: 14px; }
    a { color: #9bd1ff; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
<main>
  <section><img src="/stream.mjpg" alt="debug stream"></section>
  <aside>
    <h1>HFUT Auto Aim Debug</h1>
    <p>Status is overlaid on the video stream.</p>
    <p><a href="/snapshot.jpg">snapshot.jpg</a></p>
    <p><a href="/status.json">status.json</a></p>
  </aside>
</main>
</body>
</html>
)HTML";
}

bool sendAll(int fd, const void* data, size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  while (size > 0) {
#ifdef MSG_NOSIGNAL
    const ssize_t sent = ::send(fd, bytes, size, MSG_NOSIGNAL);
#else
    const ssize_t sent = ::send(fd, bytes, size, 0);
#endif
    if (sent <= 0) return false;
    bytes += sent;
    size -= static_cast<size_t>(sent);
  }
  return true;
}

bool sendString(int fd, const std::string& text) {
  return sendAll(fd, text.data(), text.size());
}

std::string httpHeader(const std::string& status, const std::string& content_type,
                       size_t content_length = 0, bool close = true) {
  std::ostringstream out;
  out << "HTTP/1.1 " << status << "\r\n"
      << "Content-Type: " << content_type << "\r\n"
      << "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
      << "Pragma: no-cache\r\n";
  if (content_length > 0) out << "Content-Length: " << content_length << "\r\n";
  if (close) out << "Connection: close\r\n";
  out << "\r\n";
  return out.str();
}

std::string requestPath(const std::string& request) {
  const size_t method_end = request.find(' ');
  if (method_end == std::string::npos) return "/";
  const size_t path_end = request.find(' ', method_end + 1);
  if (path_end == std::string::npos) return "/";
  return request.substr(method_end + 1, path_end - method_end - 1);
}

}  // namespace

DebugMjpegServer::DebugMjpegServer(DebugMjpegServerConfig config)
    : config_(std::move(config)) {}

DebugMjpegServer::~DebugMjpegServer() { stop(); }

bool DebugMjpegServer::start() {
  if (running_.load()) return true;
  if (!bindListenSocket()) return false;
  running_.store(true);
  accept_thread_ = std::thread(&DebugMjpegServer::acceptLoop, this);
  return true;
}

void DebugMjpegServer::stop() {
  if (!running_.exchange(false)) return;
  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }
  frame_cv_.notify_all();
  if (accept_thread_.joinable()) accept_thread_.join();
  std::vector<std::thread> clients;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients.swap(client_threads_);
  }
  for (auto& thread : clients) {
    if (thread.joinable()) thread.join();
  }
}

std::string DebugMjpegServer::errorMessage() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return error_message_;
}

std::string DebugMjpegServer::url() const {
  const std::string host = (config_.host.empty() || config_.host == "0.0.0.0")
      ? "<nuc-ip>"
      : config_.host;
  return "http://" + host + ":" + std::to_string(config_.port) + "/";
}

void DebugMjpegServer::publish(const cv::Mat& image, const DebugMjpegStatus& status) {
  if (!running_.load() || image.empty()) return;

  cv::Mat encoded_image = image;
  cv::Mat resized;
  if (config_.max_width > 0 && image.cols > config_.max_width) {
    const double scale = static_cast<double>(config_.max_width) / image.cols;
    cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    encoded_image = resized;
  }

  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,
                             std::max(30, std::min(config_.jpeg_quality, 95))};
  std::vector<uint8_t> jpeg;
  if (!cv::imencode(".jpg", encoded_image, jpeg, params)) return;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_jpeg_ = std::move(jpeg);
    latest_status_ = status;
    ++frame_version_;
  }
  frame_cv_.notify_all();
}

bool DebugMjpegServer::bindListenSocket() {
  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = std::string("socket failed: ") + std::strerror(errno);
    return false;
  }

  int reuse = 1;
  ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(config_.port);
  if (config_.host.empty() || config_.host == "0.0.0.0" || config_.host == "*") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (::inet_pton(AF_INET, config_.host.c_str(), &addr.sin_addr) != 1) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = "web host must be an IPv4 address, got: " + config_.host;
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = std::string("bind failed: ") + std::strerror(errno);
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (::listen(server_fd_, 8) != 0) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_message_ = std::string("listen failed: ") + std::strerror(errno);
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }
  return true;
}

void DebugMjpegServer::acceptLoop() {
  while (running_.load()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(server_fd_, &read_fds);
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    const int ready = ::select(server_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0) continue;

    const int client_fd = ::accept(server_fd_, nullptr, nullptr);
    if (client_fd < 0) continue;
    std::lock_guard<std::mutex> lock(clients_mutex_);
    client_threads_.emplace_back(&DebugMjpegServer::handleClient, this, client_fd);
  }
}

void DebugMjpegServer::handleClient(int client_fd) {
  timeval send_timeout{};
  send_timeout.tv_sec = 1;
  ::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));

  char buffer[2048] = {};
  const ssize_t n = ::recv(client_fd, buffer, sizeof(buffer) - 1, 0);
  if (n <= 0) {
    ::close(client_fd);
    return;
  }
  const std::string path = requestPath(std::string(buffer, static_cast<size_t>(n)));

  if (path == "/" || path == "/index.html") {
    const std::string body = indexHtml();
    sendString(client_fd, httpHeader("200 OK", "text/html; charset=utf-8", body.size()));
    sendString(client_fd, body);
    ::close(client_fd);
    return;
  }

  if (path == "/status.json") {
    DebugMjpegStatus status;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      status = latest_status_;
    }
    const std::string body = statusJson(status);
    sendString(client_fd, httpHeader("200 OK", "application/json", body.size()));
    sendString(client_fd, body);
    ::close(client_fd);
    return;
  }

  if (path == "/snapshot.jpg") {
    std::vector<uint8_t> jpeg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      jpeg = latest_jpeg_;
    }
    if (jpeg.empty()) {
      const std::string body = "no frame yet\n";
      sendString(client_fd, httpHeader("503 Service Unavailable", "text/plain", body.size()));
      sendString(client_fd, body);
    } else {
      sendString(client_fd, httpHeader("200 OK", "image/jpeg", jpeg.size()));
      sendAll(client_fd, jpeg.data(), jpeg.size());
    }
    ::close(client_fd);
    return;
  }

  if (path == "/stream.mjpg") {
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n"
           << "Content-Type: multipart/x-mixed-replace; boundary=" << kBoundary << "\r\n"
           << "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
           << "Pragma: no-cache\r\n\r\n";
    if (!sendString(client_fd, header.str())) {
      ::close(client_fd);
      return;
    }

    uint64_t seen_version = 0;
    while (running_.load()) {
      std::vector<uint8_t> jpeg;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        frame_cv_.wait_for(lock, std::chrono::seconds(2), [&] {
          return !running_.load() || frame_version_ != seen_version;
        });
        if (!running_.load()) break;
        if (latest_jpeg_.empty() || frame_version_ == seen_version) continue;
        seen_version = frame_version_;
        jpeg = latest_jpeg_;
      }

      std::ostringstream part;
      part << "--" << kBoundary << "\r\n"
           << "Content-Type: image/jpeg\r\n"
           << "Content-Length: " << jpeg.size() << "\r\n\r\n";
      if (!sendString(client_fd, part.str())) break;
      if (!sendAll(client_fd, jpeg.data(), jpeg.size())) break;
      if (!sendString(client_fd, "\r\n")) break;
    }
    ::close(client_fd);
    return;
  }

  const std::string body = "not found\n";
  sendString(client_fd, httpHeader("404 Not Found", "text/plain", body.size()));
  sendString(client_fd, body);
  ::close(client_fd);
}

}  // namespace hfut::io
