#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/serial/gimbal_transport.hpp"
#include "io/serial/infantry_serial.hpp"
#include "io/serial/sentry_packet.hpp"
#include "io/serial/sentry_serial.hpp"

namespace {

void printUsage(const char* argv0) {
  std::fprintf(
      stderr,
      "用法: %s [port] [baudrate] [tx_protocol] [rx_protocol] [tail_fields]\n"
      "发送: infantry_16 | infantry_24 | infantry_32 | sentry_64\n"
      "接收: infantry_feedback_24 | sentry_feedback_64（两者必须成对选择）。\n"
      "tail_fields: acceleration | duplicate_velocity\n"
      "示例: %s /dev/ttyACM0 115200 sentry_64 sentry_feedback_64\n"
      "说明: infantry 单位为弧度；sentry_64 在线路上使用角度制。\n",
      argv0, argv0);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 6 || (argc > 1 && std::string(argv[1]) == "--help")) {
    printUsage(argv[0]);
    return argc > 6 ? 1 : 0;
  }

  const std::string port = argc >= 2 ? argv[1] : "/dev/ttyACM0";
  const int baudrate = argc >= 3 ? std::atoi(argv[2]) : 115200;
  const std::string tx_protocol = argc >= 4 ? argv[3] : "infantry_32";
  const std::string rx_protocol = argc >= 5 ? argv[4] : hfut::io::kInfantryFeedbackProtocol;
  const bool sentry = tx_protocol == hfut::io::kSentryCommandProtocol;

  hfut::io::InfantrySerialConfig infantry_config;
  infantry_config.port = port;
  infantry_config.baudrate = baudrate;
  infantry_config.tail_fields = hfut::io::Infantry32TailFields::kAcceleration;
  infantry_config.command_angles_in_degrees = false;
  infantry_config.feedback_angles_in_degrees = false;
  infantry_config.allow_fire = false;
  infantry_config.read_timeout_ms = 2;

  if (sentry) {
    if (rx_protocol != hfut::io::kSentryFeedbackProtocol || argc >= 6) {
      std::fprintf(stderr, "sentry_64 必须搭配 sentry_feedback_64，且不接受 infantry tail_fields。\n");
      return 1;
    }
  } else {
    if (!hfut::io::parseInfantryCommandLayout(tx_protocol, infantry_config.tx_layout)) {
      std::fprintf(stderr, "不支持的 tx_protocol: %s\n", tx_protocol.c_str());
      return 1;
    }
    if (rx_protocol != hfut::io::kInfantryFeedbackProtocol) {
      std::fprintf(stderr, "infantry_16/24/32 必须搭配 infantry_feedback_24: %s\n", rx_protocol.c_str());
      return 1;
    }
    if (argc >= 6 && !hfut::io::parseInfantry32TailFields(argv[5], infantry_config.tail_fields)) {
      std::fprintf(stderr, "不支持的 tail_fields: %s\n", argv[5]);
      return 1;
    }
  }

  if (!sentry && infantry_config.tx_layout == hfut::io::InfantryCommandLayout::kInfantry32) {
    std::fprintf(stderr, "infantry_32 尾部字段: %s\n",
                 hfut::io::infantry32TailFieldsName(infantry_config.tail_fields));
  }
  std::fprintf(stderr, "串口单位: %s；开火位固定关闭。\n",
               sentry ? "sentry 角度制" : "rad / rad/s / rad/s²");

  std::unique_ptr<hfut::io::GimbalTransport> serial;
  if (sentry) {
    hfut::io::SentrySerialConfig sentry_config;
    sentry_config.port = port;
    sentry_config.baudrate = baudrate;
    sentry_config.allow_fire = false;
    sentry_config.read_timeout_ms = 2;
    serial = std::make_unique<hfut::io::SentrySerialTransport>(sentry_config);
  } else {
    serial = std::make_unique<hfut::io::InfantrySerialTransport>(infantry_config);
  }
  if (!serial->open()) {
    std::fprintf(stderr, "串口打开失败: %s\n", serial->errorMessage().c_str());
    return 1;
  }

  std::printf("串口已打开: %s @ %d tx=%s rx=%s tail=%s\n",
              port.c_str(), baudrate, tx_protocol.c_str(), rx_protocol.c_str(),
              sentry ? "not_applicable" : hfut::io::infantry32TailFieldsName(infantry_config.tail_fields));

  while (true) {
    hfut::io::SerialFeedback feedback;
    hfut::GimbalCommand command;
    command.fire_advice = false;

    if (serial->readFeedback(feedback)) {
      std::printf("反馈 mode=%u roll=%.4f yaw=%.4f pitch=%.4f rad ",
                  static_cast<unsigned>(feedback.mode), feedback.roll_rad,
                  feedback.yaw_rad, feedback.pitch_rad);
      if (feedback.fire_delay_s) {
        std::printf("fire_delay=%.3fms ", *feedback.fire_delay_s * 1000.0);
      } else {
        std::printf("fire_delay=empty ");
      }
      std::printf("bullet_speed=%.3fm/s\n", feedback.bullet_speed);
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

    serial->sendCommand(command);
  }
}
