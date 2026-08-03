// ROS-free scalar plotter: sends a flat {"key": number, ...} JSON object as one
// UDP datagram per call to PlotJuggler's UDP Server (default 127.0.0.1:9870,
// message protocol = JSON). Mirrors reference/sp_vision_25 tools::Plotter but
// hand-rolls the JSON so it needs no nlohmann dependency.
//
// PlotJuggler setup: Streaming -> UDP Server -> port 9870, protocol JSON.
#ifndef HFUT_AUTO_AIM_PLOTTER_HPP
#define HFUT_AUTO_AIM_PLOTTER_HPP

#include <cstdint>
#include <initializer_list>
#include <mutex>
#include <string>
#include <utility>

struct sockaddr_in;  // fwd; full def pulled in the .cpp

namespace hfut::io {

class Plotter {
 public:
  explicit Plotter(const std::string& host = "127.0.0.1", uint16_t port = 9870);
  ~Plotter();

  Plotter(const Plotter&) = delete;
  Plotter& operator=(const Plotter&) = delete;

  // Send one datagram: {"k0":v0,"k1":v1,...}. Non-finite values are emitted as
  // 0 (PlotJuggler's JSON parser rejects NaN/Inf).
  void plot(std::initializer_list<std::pair<const char*, double>> fields);

 private:
  int socket_{-1};
  void* dest_;  // sockaddr_in*, owned
  std::mutex mutex_;
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_PLOTTER_HPP
