#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/serial/infantry_serial.hpp"

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

void printUsage(const char* argv0) {
  std::fprintf(stderr,
               "Usage: %s [port] [baudrate] [protocol]\n"
               "Protocol: infantry | infantry_16 | infantry_32\n"
               "Example: %s /dev/ttyACM0 115200 infantry\n",
               argv0, argv0);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc > 4 || (argc > 1 && std::string(argv[1]) == "--help")) {
    printUsage(argv[0]);
    return argc > 4 ? 1 : 0;
  }

  hfut::io::InfantrySerialConfig config;
  if (argc >= 2) config.port = argv[1];
  if (argc >= 3) config.baudrate = std::atoi(argv[2]);
  if (argc >= 4 &&
      !hfut::io::parseInfantryPacketLayout(argv[3], config.layout)) {
    std::fprintf(stderr, "unsupported protocol: %s\n", argv[3]);
    return 1;
  }
  config.allow_fire = false;

  hfut::io::InfantrySerialTransport serial(config);
  if (!serial.open()) {
    std::fprintf(stderr, "serial open failed: %s\n", serial.errorMessage().c_str());
    return 1;
  }

  std::printf("serial opened: %s @ %d protocol=%s\n", config.port.c_str(),
              config.baudrate, hfut::io::infantryPacketLayoutName(config.layout));
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
