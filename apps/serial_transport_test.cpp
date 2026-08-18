#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/serial/infantry_serial.hpp"

namespace {

void printUsage(const char* argv0) {
  std::fprintf(
      stderr,
      "用法: %s [port] [baudrate] [tx_protocol] [rx_protocol] [tail_fields]\n"
      "协议: infantry | infantry_16 | infantry_32\n"
      "tail_fields: acceleration | duplicate_velocity\n"
      "示例: %s /dev/ttyACM0 115200 infantry_32 infantry acceleration\n"
      "说明: 串口收发单位固定为 rad / rad/s / rad/s²。\n",
      argv0, argv0);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 6 || (argc > 1 && std::string(argv[1]) == "--help")) {
    printUsage(argv[0]);
    return argc > 6 ? 1 : 0;
  }

  hfut::io::InfantrySerialConfig config;
  config.tx_layout = hfut::io::InfantryPacketLayout::kInfantry32;
  config.rx_layout = hfut::io::InfantryPacketLayout::kInfantry24;
  config.tail_fields = hfut::io::Infantry32TailFields::kAcceleration;
  config.command_angles_in_degrees = false;
  config.feedback_angles_in_degrees = false;
  config.allow_fire = false;
  config.read_timeout_ms = 2;

  if (argc >= 2) config.port = argv[1];
  if (argc >= 3) config.baudrate = std::atoi(argv[2]);
  if (argc >= 4 && !hfut::io::parseInfantryPacketLayout(argv[3], config.tx_layout)) {
    std::fprintf(stderr, "不支持的 tx_protocol: %s\n", argv[3]);
    return 1;
  }
  if (argc >= 5 && !hfut::io::parseInfantryPacketLayout(argv[4], config.rx_layout)) {
    std::fprintf(stderr, "不支持的 rx_protocol: %s\n", argv[4]);
    return 1;
  }
  if (argc >= 6 && !hfut::io::parseInfantry32TailFields(argv[5], config.tail_fields)) {
    std::fprintf(stderr, "不支持的 tail_fields: %s\n", argv[5]);
    return 1;
  }

  if (config.tx_layout != config.rx_layout) {
    std::fprintf(stderr, "收发协议不对称: tx=%s rx=%s\n",
                 hfut::io::infantryPacketLayoutName(config.tx_layout),
                 hfut::io::infantryPacketLayoutName(config.rx_layout));
  }
  if (config.tx_layout == hfut::io::InfantryPacketLayout::kInfantry32) {
    std::fprintf(stderr, "infantry_32 尾部字段: %s\n",
                 hfut::io::infantry32TailFieldsName(config.tail_fields));
  }
  std::fprintf(stderr, "串口单位: rad / rad/s / rad/s²；开火位固定关闭。\n");

  hfut::io::InfantrySerialTransport serial(config);
  if (!serial.open()) {
    std::fprintf(stderr, "串口打开失败: %s\n", serial.errorMessage().c_str());
    return 1;
  }

  std::printf("串口已打开: %s @ %d tx=%s rx=%s tail=%s\n",
              config.port.c_str(), config.baudrate,
              hfut::io::infantryPacketLayoutName(config.tx_layout),
              hfut::io::infantryPacketLayoutName(config.rx_layout),
              hfut::io::infantry32TailFieldsName(config.tail_fields));

  while (true) {
    hfut::io::SerialFeedback feedback;
    hfut::GimbalCommand command;
    command.fire_advice = false;

    if (serial.readFeedback(feedback)) {
      std::printf("反馈 mode=%u roll=%.4f yaw=%.4f pitch=%.4f rad\n",
                  static_cast<unsigned>(feedback.mode), feedback.roll_rad,
                  feedback.yaw_rad, feedback.pitch_rad);
      command.yaw = feedback.yaw_rad;
      command.pitch = feedback.pitch_rad;
      command.distance = 0.0;
      command.mode = hfut::GimbalMode::no_valid_measurement;
    } else {
      command.yaw = 0.0;
      command.pitch = 0.0;
      command.distance = -1.0;
      command.mode = hfut::GimbalMode::unknown;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    serial.sendCommand(command);
  }
}
