#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <netinet/in.h>  // sockaddr_in
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

#include <cmath>
#include <cstdio>
#include <string>

namespace hfut::io {

Plotter::Plotter(const std::string& host, uint16_t port) {
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  auto* dst = new sockaddr_in{};
  dst->sin_family = AF_INET;
  dst->sin_port = ::htons(port);
  dst->sin_addr.s_addr = ::inet_addr(host.c_str());
  dest_ = dst;
}

Plotter::~Plotter() {
  if (socket_ >= 0) ::close(socket_);
  delete static_cast<sockaddr_in*>(dest_);
}

void Plotter::plot(std::initializer_list<std::pair<const char*, double>> fields) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ < 0) return;

  std::string out = "{";
  bool first = true;
  char buf[64];
  for (const auto& [key, value] : fields) {
    double v = std::isfinite(value) ? value : 0.0;
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    if (!first) out += ',';
    out += '"';
    out += key;
    out += "\":";
    out += buf;
    first = false;
  }
  out += '}';

  auto* dst = static_cast<sockaddr_in*>(dest_);
  ::sendto(socket_, out.c_str(), out.size(), 0,
           reinterpret_cast<sockaddr*>(dst), sizeof(*dst));
}

}  // namespace hfut::io
