#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/serial/infantry_serial.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr int kWindowWidth = 900;
constexpr int kWindowHeight = 650;

struct Options {
  std::string port = "/dev/ttyACM0";
  int baudrate = 115200;
  double yaw_range_deg = 20.0;
  double pitch_range_deg = 15.0;
  double hz = 100.0;
  double distance_m = 1.0;
  double max_rate_rad_s = 5.0;
  double max_acc_rad_s2 = 80.0;
  bool read_feedback = true;
};

struct MouseState {
  int x = kWindowWidth / 2;
  int y = kWindowHeight / 2;
  bool paused = false;
};

struct CommandState {
  double yaw = 0.0;
  double pitch = 0.0;
  double yaw_vel = 0.0;
  double pitch_vel = 0.0;
};

double clampAbs(double value, double limit) {
  if (limit <= 0.0) return value;
  return std::clamp(value, -limit, limit);
}

void printUsage(const char* argv0) {
  std::fprintf(
      stderr,
      "Usage: %s [options]\n"
      "Options:\n"
      "  --port PATH                 Serial device, default /dev/ttyACM0\n"
      "  --baudrate N                Serial baudrate, default 115200\n"
      "  --yaw-range-deg DEG         Mouse half-width yaw range, default 20\n"
      "  --pitch-range-deg DEG       Mouse half-height pitch range, default 15\n"
      "  --hz N                      Send rate, default 100\n"
      "  --distance M                Command distance, default 1.0\n"
      "  --max-rate-rad-s N          Velocity clamp, default 5\n"
      "  --max-acc-rad-s2 N          Acceleration clamp, default 80\n"
      "  --no-feedback               Do not read lower-controller feedback\n"
      "Keys: q/Esc quit, Space/p pause, r center, WASD/IJKL/arrows move\n",
      argv0);
}

bool parseDouble(const char* text, double& out) {
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0') return false;
  out = value;
  return true;
}

bool parseInt(const char* text, int& out) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0') return false;
  out = static_cast<int>(value);
  return true;
}

bool parseArgs(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto needValue = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s requires a value\n", name);
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--port") {
      const char* value = needValue("--port");
      if (!value) return false;
      options.port = value;
    } else if (arg == "--baudrate") {
      const char* value = needValue("--baudrate");
      if (!value || !parseInt(value, options.baudrate)) return false;
    } else if (arg == "--yaw-range-deg") {
      const char* value = needValue("--yaw-range-deg");
      if (!value || !parseDouble(value, options.yaw_range_deg)) return false;
    } else if (arg == "--pitch-range-deg") {
      const char* value = needValue("--pitch-range-deg");
      if (!value || !parseDouble(value, options.pitch_range_deg)) return false;
    } else if (arg == "--hz") {
      const char* value = needValue("--hz");
      if (!value || !parseDouble(value, options.hz)) return false;
    } else if (arg == "--distance") {
      const char* value = needValue("--distance");
      if (!value || !parseDouble(value, options.distance_m)) return false;
    } else if (arg == "--max-rate-rad-s") {
      const char* value = needValue("--max-rate-rad-s");
      if (!value || !parseDouble(value, options.max_rate_rad_s)) return false;
    } else if (arg == "--max-acc-rad-s2") {
      const char* value = needValue("--max-acc-rad-s2");
      if (!value || !parseDouble(value, options.max_acc_rad_s2)) return false;
    } else if (arg == "--no-feedback") {
      options.read_feedback = false;
    } else {
      std::fprintf(stderr, "unsupported argument: %s\n", arg.c_str());
      return false;
    }
  }

  if (options.baudrate <= 0 || options.hz <= 0.0 ||
      options.yaw_range_deg <= 0.0 || options.pitch_range_deg <= 0.0) {
    std::fprintf(stderr, "baudrate, hz, and angle ranges must be positive\n");
    return false;
  }
  return true;
}

void onMouse(int event, int x, int y, int, void* userdata) {
  if (event != cv::EVENT_MOUSEMOVE && event != cv::EVENT_LBUTTONDOWN &&
      event != cv::EVENT_RBUTTONDOWN) {
    return;
  }
  auto* state = static_cast<MouseState*>(userdata);
  state->x = std::clamp(x, 0, kWindowWidth - 1);
  state->y = std::clamp(y, 0, kWindowHeight - 1);
}

void nudgeInput(MouseState& state, const Options& options,
                double yaw_delta_deg, double pitch_delta_deg) {
  const double x_delta = yaw_delta_deg / options.yaw_range_deg *
                         (kWindowWidth / 2.0);
  const double y_delta = -pitch_delta_deg / options.pitch_range_deg *
                         (kWindowHeight / 2.0);
  state.x = std::clamp(static_cast<int>(std::lround(state.x + x_delta)), 0,
                       kWindowWidth - 1);
  state.y = std::clamp(static_cast<int>(std::lround(state.y + y_delta)), 0,
                       kWindowHeight - 1);
}

CommandState mouseToCommand(const MouseState& mouse, const Options& options,
                            double dt, const CommandState& previous) {
  const double nx = (static_cast<double>(mouse.x) - kWindowWidth / 2.0) /
                    (kWindowWidth / 2.0);
  const double ny = (static_cast<double>(mouse.y) - kWindowHeight / 2.0) /
                    (kWindowHeight / 2.0);

  CommandState command;
  command.yaw = nx * options.yaw_range_deg * kDegToRad;
  command.pitch = -ny * options.pitch_range_deg * kDegToRad;
  if (dt > 1e-6) {
    command.yaw_vel = clampAbs((command.yaw - previous.yaw) / dt,
                               options.max_rate_rad_s);
    command.pitch_vel = clampAbs((command.pitch - previous.pitch) / dt,
                                 options.max_rate_rad_s);
  }
  return command;
}

void drawText(cv::Mat& image, int& y, const std::string& text,
              const cv::Scalar& color = cv::Scalar(220, 220, 220)) {
  cv::putText(image, text, cv::Point(18, y), cv::FONT_HERSHEY_SIMPLEX, 0.56,
              color, 1, cv::LINE_AA);
  y += 24;
}

void drawWindow(const MouseState& mouse, const Options& options,
                const hfut::GimbalCommand& command,
                const hfut::io::SerialFeedback& feedback, bool have_feedback,
                double yaw_acc, double pitch_acc) {
  cv::Mat image(kWindowHeight, kWindowWidth, CV_8UC3, cv::Scalar(22, 24, 28));
  const cv::Point center(kWindowWidth / 2, kWindowHeight / 2);
  cv::line(image, cv::Point(center.x, 0), cv::Point(center.x, kWindowHeight),
           cv::Scalar(70, 70, 70), 1);
  cv::line(image, cv::Point(0, center.y), cv::Point(kWindowWidth, center.y),
           cv::Scalar(70, 70, 70), 1);
  cv::rectangle(image, cv::Rect(1, 1, kWindowWidth - 2, kWindowHeight - 2),
                cv::Scalar(90, 90, 90), 1);

  const cv::Scalar point_color = mouse.paused ? cv::Scalar(60, 60, 220)
                                              : cv::Scalar(80, 220, 120);
  cv::circle(image, cv::Point(mouse.x, mouse.y), 9, point_color, cv::FILLED);
  cv::line(image, center, cv::Point(mouse.x, mouse.y), point_color, 1);

  int y = 30;
  char line[256];
  std::snprintf(line, sizeof(line),
                "manual_gimbal_test  port=%s  baud=%d  send=infantry_32 mode-byte",
                options.port.c_str(), options.baudrate);
  drawText(image, y, line, cv::Scalar(250, 250, 250));
  std::snprintf(line, sizeof(line), "range yaw=+/-%.1fdeg pitch=+/-%.1fdeg  hz=%.1f",
                options.yaw_range_deg, options.pitch_range_deg, options.hz);
  drawText(image, y, line);
  std::snprintf(line, sizeof(line),
                "cmd mode=%d yaw=%.2fdeg pitch=%.2fdeg dist=%.2fm",
                static_cast<int>(command.mode), command.yaw * kRadToDeg,
                command.pitch * kRadToDeg, command.distance);
  drawText(image, y, line, mouse.paused ? cv::Scalar(80, 80, 250)
                                        : cv::Scalar(80, 240, 150));
  std::snprintf(line, sizeof(line),
                "vel yaw=%.3frad/s pitch=%.3frad/s  acc yaw=%.1frad/s2 pitch=%.1frad/s2",
                command.yaw_vel, command.pitch_vel, yaw_acc, pitch_acc);
  drawText(image, y, line, cv::Scalar(140, 220, 255));
  if (have_feedback) {
    std::snprintf(line, sizeof(line),
                  "feedback mode=%u yaw=%.2fdeg pitch=%.2fdeg roll=%.2fdeg",
                  static_cast<unsigned>(feedback.mode), feedback.yaw_rad * kRadToDeg,
                  feedback.pitch_rad * kRadToDeg, feedback.roll_rad * kRadToDeg);
  } else {
    std::snprintf(line, sizeof(line), "feedback: none");
  }
  drawText(image, y, line, cv::Scalar(255, 210, 120));
  drawText(image, y, "keys: WASD/IJKL/arrows move, r center, Space/p pause, q/Esc quit");
  cv::imshow("manual_gimbal_test", image);
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseArgs(argc, argv, options)) {
    printUsage(argv[0]);
    return 1;
  }

  hfut::io::InfantrySerialConfig serial_config;
  serial_config.port = options.port;
  serial_config.baudrate = options.baudrate;
  serial_config.tx_layout = hfut::io::InfantryPacketLayout::kInfantry32;
  serial_config.rx_layout = hfut::io::InfantryPacketLayout::kInfantry24;
  serial_config.tail_fields = hfut::io::Infantry32TailFields::kAcceleration;
  serial_config.status_byte = hfut::io::InfantryStatusByte::kCommandMode;
  serial_config.command_angles_in_degrees = false;
  serial_config.feedback_angles_in_degrees = true;
  serial_config.allow_fire = false;
  serial_config.read_timeout_ms = 1;

  hfut::io::InfantrySerialTransport serial(serial_config);
  if (!serial.open()) {
    std::fprintf(stderr, "serial open failed: %s\n", serial.errorMessage().c_str());
    return 1;
  }

  MouseState mouse;
  hfut::io::SerialFeedback latest_feedback;
  bool have_feedback = false;
  CommandState previous;
  double previous_yaw_vel = 0.0;
  double previous_pitch_vel = 0.0;
  double yaw_acc = 0.0;
  double pitch_acc = 0.0;

  try {
    cv::namedWindow("manual_gimbal_test", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("manual_gimbal_test", onMouse, &mouse);
  } catch (const cv::Exception& e) {
    std::fprintf(stderr, "OpenCV window failed: %s\n", e.what());
    return 1;
  }

  std::printf("manual gimbal opened: %s @ %d, TX=infantry_32, RX=infantry, unit=radians\n",
              options.port.c_str(), options.baudrate);
  std::printf("Move mouse in the window. q/Esc quit, Space/p pause, r center.\n");

  auto last = std::chrono::steady_clock::now();
  auto last_print = last;
  const auto period = std::chrono::duration<double>(1.0 / options.hz);

  bool running = true;
  while (running) {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::max(1e-4, std::chrono::duration<double>(now - last).count());
    last = now;

    if (options.read_feedback) {
      hfut::io::SerialFeedback feedback;
      if (serial.readFeedback(feedback)) {
        latest_feedback = feedback;
        have_feedback = true;
      }
    }

    CommandState command_state = mouseToCommand(mouse, options, dt, previous);
    yaw_acc = clampAbs((command_state.yaw_vel - previous_yaw_vel) / dt,
                       options.max_acc_rad_s2);
    pitch_acc = clampAbs((command_state.pitch_vel - previous_pitch_vel) / dt,
                         options.max_acc_rad_s2);

    hfut::GimbalCommand command;
    command.yaw = command_state.yaw;
    command.pitch = command_state.pitch;
    command.yaw_vel = command_state.yaw_vel;
    command.pitch_vel = command_state.pitch_vel;
    command.yaw_acc = yaw_acc;
    command.pitch_acc = pitch_acc;
    command.distance = options.distance_m;
    command.fire_advice = false;
    command.mode = mouse.paused ? hfut::GimbalMode::no_valid_measurement
                                : hfut::GimbalMode::normal_measurement;

    if (!serial.sendCommand(command)) {
      std::fprintf(stderr, "serial send failed: %s\n", serial.errorMessage().c_str());
    }

    drawWindow(mouse, options, command, latest_feedback, have_feedback, yaw_acc, pitch_acc);
    const int key = cv::waitKey(1) & 0xff;
    if (key == 27 || key == 'q' || key == 'Q') {
      running = false;
    } else if (key == ' ' || key == 'p' || key == 'P') {
      mouse.paused = !mouse.paused;
    } else if (key == 'r' || key == 'R') {
      mouse.x = kWindowWidth / 2;
      mouse.y = kWindowHeight / 2;
    } else if (key == 'a' || key == 'A' || key == 'j' || key == 'J' || key == 81) {
      nudgeInput(mouse, options, -1.0, 0.0);
    } else if (key == 'd' || key == 'D' || key == 'l' || key == 'L' || key == 83) {
      nudgeInput(mouse, options, 1.0, 0.0);
    } else if (key == 'w' || key == 'W' || key == 'i' || key == 'I' || key == 82) {
      nudgeInput(mouse, options, 0.0, 1.0);
    } else if (key == 's' || key == 'S' || key == 'k' || key == 'K' || key == 84) {
      nudgeInput(mouse, options, 0.0, -1.0);
    }

    if (now - last_print > std::chrono::milliseconds(500)) {
      std::printf("cmd mode=%d yaw=%.2fdeg pitch=%.2fdeg vel=(%.3f,%.3f) acc=(%.1f,%.1f)\n",
                  static_cast<int>(command.mode), command.yaw * kRadToDeg,
                  command.pitch * kRadToDeg, command.yaw_vel, command.pitch_vel,
                  yaw_acc, pitch_acc);
      last_print = now;
    }

    previous = command_state;
    previous_yaw_vel = command_state.yaw_vel;
    previous_pitch_vel = command_state.pitch_vel;
    const auto elapsed = std::chrono::steady_clock::now() - now;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }

  hfut::GimbalCommand safe;
  safe.mode = hfut::GimbalMode::no_valid_measurement;
  safe.fire_advice = false;
  safe.distance = -1.0;
  for (int i = 0; i < 5; ++i) {
    serial.sendCommand(safe);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
