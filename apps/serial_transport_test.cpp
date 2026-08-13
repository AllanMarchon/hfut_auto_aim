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
               "Usage: %s [port] [baudrate] [tx_protocol] [rx_protocol] [tail_fields] [unit]\n"
               "Protocol: infantry | infantry_16 | infantry_32\n"
               "tail_fields: acceleration | duplicate_velocity\n"
               "unit: degrees | radians\n"
               "Example: %s /dev/ttyACM0 115200 infantry_32 infantry acceleration radians\n",
               argv0, argv0);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc > 7 || (argc > 1 && std::string(argv[1]) == "--help")) {
    printUsage(argv[0]);
    return argc > 7 ? 1 : 0;
  }

  hfut::io::InfantrySerialConfig config;
  if (argc >= 2) config.port = argv[1];
  if (argc >= 3) config.baudrate = std::atoi(argv[2]);
  if (argc >= 4 &&
      !hfut::io::parseInfantryPacketLayout(argv[3], config.tx_layout)) {
    std::fprintf(stderr, "unsupported tx_protocol: %s\n", argv[3]);
    return 1;
  }
  config.rx_layout = config.tx_layout;
  if (argc >= 5 &&
      !hfut::io::parseInfantryPacketLayout(argv[4], config.rx_layout)) {
    std::fprintf(stderr, "unsupported rx_protocol: %s\n", argv[4]);
    return 1;
  }
  if (argc >= 6 &&
      !hfut::io::parseInfantry32TailFields(argv[5], config.tail_fields)) {
    std::fprintf(stderr, "unsupported tail_fields: %s\n", argv[5]);
    return 1;
  }
  if (argc >= 7) {
    const std::string unit = argv[6];
    if (unit == "radians" || unit == "rad") {
      config.command_angles_in_degrees = false;
    } else if (unit == "degrees" || unit == "deg") {
      config.command_angles_in_degrees = true;
    } else {
      std::fprintf(stderr, "unsupported unit: %s\n", argv[6]);
      return 1;
    }
  }
  if (argc >= 4 && config.tx_layout != config.rx_layout) {
    std::fprintf(stderr, "asymmetric serial protocol: tx=%s rx=%s\n",
                 hfut::io::infantryPacketLayoutName(config.tx_layout),
                 hfut::io::infantryPacketLayoutName(config.rx_layout));
  }
  if (config.tx_layout == hfut::io::InfantryPacketLayout::kInfantry32 &&
      config.tail_fields == hfut::io::Infantry32TailFields::kDuplicateVelocity) {
    std::fprintf(stderr,
                 "warning: infantry_32 TX is using duplicate_velocity, not acceleration\n");
  }
  if (config.tx_layout == hfut::io::InfantryPacketLayout::kInfantry32 &&
      config.tail_fields == hfut::io::Infantry32TailFields::kAcceleration &&
      config.command_angles_in_degrees) {
    std::fprintf(stderr,
                 "warning: acceleration tail is currently sent in degrees/s^2; pass unit=radians for rad/s^2\n");
  }
  if (config.tx_layout == hfut::io::InfantryPacketLayout::kInfantry32 &&
      config.tail_fields == hfut::io::Infantry32TailFields::kAcceleration &&
      !config.command_angles_in_degrees) {
    std::fprintf(stderr, "OK: TX acceleration uses rad/s^2\n");
  }
  if (!config.command_angles_in_degrees) {
    std::fprintf(stderr, "command unit: radians, rad/s, rad/s^2\n");
  } else {
    std::fprintf(stderr, "command unit: degrees, deg/s, deg/s^2\n");
  }
  if (config.tx_layout == hfut::io::InfantryPacketLayout::kInfantry32 &&
      config.rx_layout == hfut::io::InfantryPacketLayout::kInfantry24) {
    std::fprintf(stderr, "expected current field-test setup: TX 32 bytes, RX 24 bytes\n");
  }
  if (config.tx_layout != hfut::io::InfantryPacketLayout::kInfantry32 && argc >= 6) {
    std::fprintf(stderr, "tail_fields is ignored unless tx_protocol=infantry_32\n");
  }
  config.allow_fire = false;

  hfut::io::InfantrySerialTransport serial(config);
  if (!serial.open()) {
    std::fprintf(stderr, "serial open failed: %s\n", serial.errorMessage().c_str());
    return 1;
  }

  std::printf("serial opened: %s @ %d tx=%s rx=%s tail=%s unit=%s\n",
              config.port.c_str(), config.baudrate,
              hfut::io::infantryPacketLayoutName(config.tx_layout),
              hfut::io::infantryPacketLayoutName(config.rx_layout),
              hfut::io::infantry32TailFieldsName(config.tail_fields),
              config.command_angles_in_degrees ? "degrees" : "radians");
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
