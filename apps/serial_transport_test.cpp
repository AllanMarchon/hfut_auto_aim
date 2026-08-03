#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/serial/infantry32_serial.hpp"

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

void printUsage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s [port] [baudrate]\n"
               "Example: %s /dev/ttyACM0 115200\n",
               argv0, argv0);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc > 3 || (argc > 1 && std::string(argv[1]) == "--help")) {
    printUsage(argv[0]);
    return argc > 3 ? 1 : 0;
  }

  hfut::io::Infantry32SerialConfig config;
  if (argc >= 2) config.port = argv[1];
  if (argc >= 3) config.baudrate = std::atoi(argv[2]);
  config.allow_fire = false;

  hfut::io::Infantry32SerialTransport serial(config);
  if (!serial.open()) {
    std::fprintf(stderr, "serial open failed: %s\n", serial.errorMessage().c_str());
    return 1;
  }

  std::printf("serial opened: %s @ %d\n", config.port.c_str(), config.baudrate);
  while (true) {
    hfut::io::SerialFeedback feedback;
    if (serial.readFeedback(feedback)) {
      std::printf("mode=%u roll=%.3f yaw=%.3f pitch=%.3f rad\n",
                  static_cast<unsigned>(feedback.mode), feedback.roll_rad,
                  feedback.yaw_rad, feedback.pitch_rad);

      hfut::GimbalCommand idle;
      idle.yaw = feedback.yaw_rad;
      idle.pitch = feedback.pitch_rad;
      idle.yaw_diff = 0.0;
      idle.pitch_diff = 0.0;
      idle.distance = 0.0;
      idle.fire_advice = false;
      idle.mode = hfut::GimbalMode::no_valid_measurement;
      serial.sendCommand(idle);
    } else {
      hfut::GimbalCommand heartbeat;
      heartbeat.yaw = 0.0 * kDegToRad;
      heartbeat.pitch = 0.0 * kDegToRad;
      heartbeat.fire_advice = false;
      heartbeat.mode = hfut::GimbalMode::unknown;
      serial.sendCommand(heartbeat);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}
