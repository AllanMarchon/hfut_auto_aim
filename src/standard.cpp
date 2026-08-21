// SP25 实车入口：沿用本项目已跑通的相机、串口和可视化适配，
// 把检测、PnP、跟踪、瞄准和射击判定切到 SP25 主链。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <list>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "hfut_auto_aim/camera_frame.hpp"
#include "hfut_auto_aim/gimbal_command.hpp"
#include "hfut_auto_aim/video_calibration.hpp"
#include "io/camera/camera_source.hpp"
#ifdef HFUT_HAS_HIK_CAMERA
#include "io/camera/hik_camera_source.hpp"
#endif
#include "io/camera/opencv_camera_source.hpp"
#include "io/serial/hfut_serial_gimbal.hpp"
#include "io/serial/infantry_serial.hpp"
#include "io/web/debug_mjpeg_server.hpp"

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/math_tools.hpp"

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

struct Options {
  std::string hardware_config{"configs/hardware.yaml"};
  std::string sp25_config{"configs/standard3.yaml"};
  std::string controller_config{"configs/controller.yaml"};
  std::string runtime_sp25_config{"build/sp25_runtime.yaml"};
  std::string sp25_device;

  std::string camera_backend{"hik"};
  std::string camera_source;
  std::string camera_sn;
  int camera_index{0};
  int camera_width{0};
  int camera_height{0};
  double camera_fps{0.0};
  double camera_exposure_time_us{0.0};
  double camera_gain{0.0};
  bool camera_flip_image{false};
  std::string camera_info_path;
  std::string calibration_mode{"strict"};
  hfut::CameraToBarrelExtrinsics camera_to_barrel;

  std::string serial_port{"/dev/ttyACM0"};
  int serial_baudrate{115200};
  std::string serial_protocol{"infantry"};
  std::string serial_tx_protocol{"infantry"};
  std::string serial_rx_protocol{"infantry"};
  std::string infantry32_tail_fields{"duplicate_velocity"};
  bool command_angles_in_degrees{false};
  bool feedback_angles_in_degrees{false};
  int serial_read_timeout_ms{2};
  int serial_write_timeout_ms{5};
  int feedback_timeout_ms{100};
  bool require_feedback{true};
  bool serial_send{true};

  std::string enemy_color{"red"};
  double bullet_speed{23.0};
  bool dry_run{false};
  bool enable_fire{false};
  bool display{false};
  bool web_view{false};
  std::string web_host{"0.0.0.0"};
  int web_port{8080};
  int web_jpeg_quality{80};
  int web_max_width{960};
  int web_frame_step{2};
  int max_frames{-1};
};

struct CommandLimiterConfig {
  std::string planner_mode{"aimer_feedback_error"};
  bool enable{true};
  bool enable_clamping{true};
  bool enable_rate_limiter{true};
  bool enable_fire_gate{true};
  bool enable_target_stabilizer{true};
  bool target_stabilizer_freeze_on_unstable_state{true};
  double max_yaw_diff_rad{15.0 * kPi / 180.0};
  double max_pitch_diff_rad{10.0 * kPi / 180.0};
  double max_yaw_step_rad{1.5 * kPi / 180.0};
  double max_pitch_step_rad{1.0 * kPi / 180.0};
  double max_yaw_acc_rad_s2{80.0};
  double max_pitch_acc_rad_s2{80.0};
  double fire_yaw_tolerance_rad{3.0 * kPi / 180.0};
  double fire_pitch_tolerance_rad{2.0 * kPi / 180.0};
  double reference_rate_hz{250.0};
  double reset_timeout_s{0.25};
  double sp_pitch_to_command_sign{-1.0};
  double feedback_yaw_to_world_sign{-1.0};
  bool enable_feedback_alignment{true};
  double feedback_alignment_offset_s{0.001};
  double feedback_alignment_max_sample_age_s{0.05};
  int feedback_alignment_history_size{1000};
  bool serial_command_send_velocity{true};
  bool serial_command_send_acceleration{true};
  std::string serial_command_velocity_mode{"feedback_error"};
  double serial_command_yaw_error_gain{6.0};
  double serial_command_pitch_error_gain{6.0};
  double serial_command_max_yaw_velocity_rad_s{120.0 * kPi / 180.0};
  double serial_command_max_pitch_velocity_rad_s{90.0 * kPi / 180.0};
  double target_stabilizer_max_yaw_jump_rad{4.0 * kPi / 180.0};
  double target_stabilizer_max_yaw_rate_rad_s{120.0 * kPi / 180.0};
  double lost_target_hold_s{0.25};
  int target_stabilizer_hold_frames{3};
};

bool velocityModeUsesFeedbackError(const std::string& mode) {
  return mode == "feedback_error" || mode == "error_feedback" || mode == "tracking";
}

bool plannerModeUsesMpc(const std::string& mode) {
  return mode == "mpc" || mode == "tinympc";
}

struct FireGateResult {
  double yaw_error_rad{0.0};
  double pitch_error_rad{0.0};
  bool blocked{false};
};

class SimpleCommandGuard {
 public:
  explicit SimpleCommandGuard(CommandLimiterConfig config) : config_(config) {}

  void apply(hfut::GimbalCommand& command, const hfut::io::SerialFeedback& feedback,
             const std::string& track_state, std::chrono::steady_clock::time_point now) {
    if (!config_.enable) return passThrough(command, feedback, now);
    if (command.mode != hfut::GimbalMode::normal_measurement) return holdLastAimOnLoss(command, feedback, now);

    double target_yaw = command.yaw;
    double target_pitch = command.pitch;
    bool blocked = false;

    if (config_.enable_clamping) {
      const double clamped_yaw_diff = std::clamp(
          tools::limit_rad(target_yaw - feedback.yaw_rad),
          -config_.max_yaw_diff_rad, config_.max_yaw_diff_rad);
      const double clamped_pitch_diff = std::clamp(
          target_pitch - feedback.pitch_rad,
          -config_.max_pitch_diff_rad, config_.max_pitch_diff_rad);
      blocked = blocked || std::abs(tools::limit_rad(target_yaw - feedback.yaw_rad) - clamped_yaw_diff) > 1e-9;
      blocked = blocked || std::abs((target_pitch - feedback.pitch_rad) - clamped_pitch_diff) > 1e-9;
      target_yaw = feedback.yaw_rad + clamped_yaw_diff;
      target_pitch = feedback.pitch_rad + clamped_pitch_diff;
    }

    double dt = 1.0 / std::max(config_.reference_rate_hz, 1.0);
    if (have_last_) {
      dt = std::chrono::duration<double>(now - last_time_).count();
      if (!std::isfinite(dt) || dt <= 0.0 || dt > config_.reset_timeout_s) {
        dt = 1.0 / std::max(config_.reference_rate_hz, 1.0);
        have_last_ = false;
      }
    }

    if (have_last_ && config_.target_stabilizer_freeze_on_unstable_state && track_state != "tracking") {
      command.yaw = last_yaw_;
      command.pitch = last_pitch_;
      command.distance = last_distance_;
      command.fire_advice = false;
      fillDiffAndMotion(command, feedback, now, false);
      last_time_ = now;
      return;
    }

    if (have_last_ && config_.enable_target_stabilizer) {
      const double yaw_jump = std::abs(tools::limit_rad(target_yaw - last_yaw_));
      const double pitch_jump = std::abs(target_pitch - last_pitch_);
      const bool jump = yaw_jump > config_.target_stabilizer_max_yaw_jump_rad ||
                        pitch_jump > config_.max_pitch_diff_rad;
      bool hold_last_target = false;
      if (jump) {
        const bool same_candidate = have_jump_candidate_ &&
                                    std::abs(tools::limit_rad(target_yaw - jump_candidate_yaw_)) <=
                                        config_.target_stabilizer_max_yaw_jump_rad &&
                                    std::abs(target_pitch - jump_candidate_pitch_) <= config_.max_pitch_diff_rad;
        if (!same_candidate) {
          jump_candidate_yaw_ = target_yaw;
          jump_candidate_pitch_ = target_pitch;
          jump_hold_count_ = 0;
          have_jump_candidate_ = true;
        }
        if (jump_hold_count_ < config_.target_stabilizer_hold_frames) {
          ++jump_hold_count_;
          hold_last_target = true;
        }
        blocked = true;
      } else {
        have_jump_candidate_ = false;
        jump_hold_count_ = 0;
      }
      if (hold_last_target) {
        target_yaw = last_yaw_;
        target_pitch = last_pitch_;
        command.distance = last_distance_;
      }
    }

    if (have_last_ && config_.enable_rate_limiter) {
      double yaw_step_limit = config_.max_yaw_step_rad;
      const double target_rate_step = config_.target_stabilizer_max_yaw_rate_rad_s * dt;
      if (std::isfinite(target_rate_step) && target_rate_step > 0.0) {
        yaw_step_limit = std::min(yaw_step_limit, target_rate_step);
      }
      const double yaw_delta = tools::limit_rad(target_yaw - last_yaw_);
      const double pitch_delta = target_pitch - last_pitch_;
      const double limited_yaw_delta = std::clamp(yaw_delta, -yaw_step_limit, yaw_step_limit);
      const double limited_pitch_delta = std::clamp(
          pitch_delta, -config_.max_pitch_step_rad, config_.max_pitch_step_rad);
      blocked = blocked || std::abs(yaw_delta - limited_yaw_delta) > 1e-9;
      blocked = blocked || std::abs(pitch_delta - limited_pitch_delta) > 1e-9;
      target_yaw = tools::limit_rad(last_yaw_ + limited_yaw_delta);
      target_pitch = last_pitch_ + limited_pitch_delta;
    }

    command.yaw = tools::limit_rad(target_yaw);
    command.pitch = target_pitch;
    if (blocked) command.fire_advice = false;
    fillDiffAndMotion(command, feedback, now, true);
    saveLastAim(command, now);
  }

 private:
  void passThrough(hfut::GimbalCommand& command, const hfut::io::SerialFeedback& feedback,
                   std::chrono::steady_clock::time_point now) {
    fillDiffAndMotion(command, feedback, now, true);
    if (command.mode == hfut::GimbalMode::normal_measurement) saveLastAim(command, now);
    else reset();
  }

  void holdLastAimOnLoss(hfut::GimbalCommand& command, const hfut::io::SerialFeedback& feedback,
                         std::chrono::steady_clock::time_point now) {
    if (!have_last_ || config_.lost_target_hold_s <= 0.0) return resetCommand(command);
    const double lost_s = std::chrono::duration<double>(now - last_valid_time_).count();
    if (!std::isfinite(lost_s) || lost_s > config_.lost_target_hold_s) return resetCommand(command);

    command.mode = hfut::GimbalMode::normal_measurement;
    command.yaw = last_yaw_;
    command.pitch = last_pitch_;
    command.distance = last_distance_;
    command.fire_advice = false;
    fillDiffAndMotion(command, feedback, now, false);
    last_time_ = now;
  }

  void resetCommand(hfut::GimbalCommand& command) {
    command.yaw_vel = 0.0;
    command.pitch_vel = 0.0;
    command.yaw_acc = 0.0;
    command.pitch_acc = 0.0;
    reset();
  }

  void fillDiffAndMotion(hfut::GimbalCommand& command, const hfut::io::SerialFeedback& feedback,
                         std::chrono::steady_clock::time_point now, bool derive_motion) {
    command.yaw_diff = tools::limit_rad(command.yaw - feedback.yaw_rad);
    command.pitch_diff = command.pitch - feedback.pitch_rad;
    if (!derive_motion || !have_last_) {
      command.yaw_vel = 0.0;
      command.pitch_vel = 0.0;
      command.yaw_acc = 0.0;
      command.pitch_acc = 0.0;
      return;
    }
    const double dt = std::chrono::duration<double>(now - last_time_).count();
    if (!std::isfinite(dt) || dt <= 1e-6 || dt > config_.reset_timeout_s) {
      command.yaw_vel = 0.0;
      command.pitch_vel = 0.0;
      command.yaw_acc = 0.0;
      command.pitch_acc = 0.0;
      return;
    }
    const double yaw_target_vel = tools::limit_rad(command.yaw - last_yaw_) / dt;
    const double pitch_target_vel = (command.pitch - last_pitch_) / dt;
    command.yaw_vel = yaw_target_vel;
    command.pitch_vel = pitch_target_vel;
    if (velocityModeUsesFeedbackError(config_.serial_command_velocity_mode)) {
      command.yaw_vel += config_.serial_command_yaw_error_gain * command.yaw_diff;
      command.pitch_vel += config_.serial_command_pitch_error_gain * command.pitch_diff;
    }
    command.yaw_vel = std::clamp(command.yaw_vel,
                                 -config_.serial_command_max_yaw_velocity_rad_s,
                                 config_.serial_command_max_yaw_velocity_rad_s);
    command.pitch_vel = std::clamp(command.pitch_vel,
                                   -config_.serial_command_max_pitch_velocity_rad_s,
                                   config_.serial_command_max_pitch_velocity_rad_s);
    command.yaw_acc = std::clamp((command.yaw_vel - last_yaw_vel_) / dt,
                                 -config_.max_yaw_acc_rad_s2, config_.max_yaw_acc_rad_s2);
    command.pitch_acc = std::clamp((command.pitch_vel - last_pitch_vel_) / dt,
                                   -config_.max_pitch_acc_rad_s2, config_.max_pitch_acc_rad_s2);
  }

  void reset() {
    have_last_ = false;
    have_jump_candidate_ = false;
    jump_hold_count_ = 0;
  }

  void saveLastAim(const hfut::GimbalCommand& command, std::chrono::steady_clock::time_point now) {
    last_yaw_ = command.yaw;
    last_pitch_ = command.pitch;
    last_distance_ = command.distance;
    last_yaw_vel_ = command.yaw_vel;
    last_pitch_vel_ = command.pitch_vel;
    last_time_ = now;
    last_valid_time_ = now;
    have_last_ = true;
  }

  CommandLimiterConfig config_;
  bool have_last_{false};
  bool have_jump_candidate_{false};
  int jump_hold_count_{0};
  double jump_candidate_yaw_{0.0};
  double jump_candidate_pitch_{0.0};
  double last_yaw_{0.0};
  double last_pitch_{0.0};
  double last_distance_{0.0};
  double last_yaw_vel_{0.0};
  double last_pitch_vel_{0.0};
  std::chrono::steady_clock::time_point last_time_{};
  std::chrono::steady_clock::time_point last_valid_time_{};
};

class FinalVelocityAccelerationAdapter {
 public:
  explicit FinalVelocityAccelerationAdapter(CommandLimiterConfig config) : config_(config) {}

  void reset() { have_last_ = false; }

  void apply(hfut::GimbalCommand& command, std::chrono::steady_clock::time_point now) {
    if (command.mode != hfut::GimbalMode::normal_measurement) {
      command.yaw_acc = 0.0;
      command.pitch_acc = 0.0;
      reset();
      return;
    }
    if (!have_last_) {
      command.yaw_acc = 0.0;
      command.pitch_acc = 0.0;
      save(command, now);
      return;
    }

    const double dt = std::chrono::duration<double>(now - last_time_).count();
    if (!std::isfinite(dt) || dt <= 1e-6 || dt > config_.reset_timeout_s) {
      command.yaw_acc = 0.0;
      command.pitch_acc = 0.0;
      save(command, now);
      return;
    }

    command.yaw_acc = std::clamp((command.yaw_vel - last_yaw_vel_) / dt,
                                 -config_.max_yaw_acc_rad_s2, config_.max_yaw_acc_rad_s2);
    command.pitch_acc = std::clamp((command.pitch_vel - last_pitch_vel_) / dt,
                                   -config_.max_pitch_acc_rad_s2,
                                   config_.max_pitch_acc_rad_s2);
    save(command, now);
  }

 private:
  void save(const hfut::GimbalCommand& command, std::chrono::steady_clock::time_point now) {
    last_yaw_vel_ = command.yaw_vel;
    last_pitch_vel_ = command.pitch_vel;
    last_time_ = now;
    have_last_ = true;
  }

  CommandLimiterConfig config_;
  bool have_last_{false};
  double last_yaw_vel_{0.0};
  double last_pitch_vel_{0.0};
  std::chrono::steady_clock::time_point last_time_{};
};

FireGateResult applyFireGate(hfut::GimbalCommand& command,
                             double desired_yaw,
                             double desired_pitch,
                             const CommandLimiterConfig& config) {
  FireGateResult result;
  result.yaw_error_rad = std::abs(tools::limit_rad(desired_yaw - command.yaw));
  result.pitch_error_rad = std::abs(desired_pitch - command.pitch);
  if (!command.fire_advice || !config.enable_fire_gate) return result;

  const double feedback_yaw_error_rad = std::abs(command.yaw_diff);
  const double feedback_pitch_error_rad = std::abs(command.pitch_diff);
  if (result.yaw_error_rad > config.fire_yaw_tolerance_rad ||
      result.pitch_error_rad > config.fire_pitch_tolerance_rad ||
      feedback_yaw_error_rad > config.fire_yaw_tolerance_rad ||
      feedback_pitch_error_rad > config.fire_pitch_tolerance_rad) {
    command.fire_advice = false;
    result.blocked = true;
  }
  return result;
}

std::string optionValue(int argc, char** argv, int& index,
                        const std::string& arg, const std::string& name) {
  const std::string prefix = name + "=";
  if (arg.compare(0, prefix.size(), prefix) == 0) return arg.substr(prefix.size());
  if (arg == name && index + 1 < argc) return argv[++index];
  return {};
}

bool parseBool(const YAML::Node& node, bool fallback) {
  return node ? node.as<bool>() : fallback;
}

int parseInt(const YAML::Node& node, int fallback) {
  return node ? node.as<int>() : fallback;
}

double parseDouble(const YAML::Node& node, double fallback) {
  return node ? node.as<double>() : fallback;
}

std::string parseString(const YAML::Node& node, const std::string& fallback) {
  return node ? node.as<std::string>() : fallback;
}

Eigen::Vector3d readVector3(const YAML::Node& node,
                            const Eigen::Vector3d& fallback,
                            const std::string& name) {
  if (!node) return fallback;
  if (!node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(name + " 必须是 3 个数");
  }
  Eigen::Vector3d value(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!value.allFinite()) throw std::invalid_argument(name + " 里存在非有限数值");
  return value;
}

YAML::Node controllerNode(const YAML::Node& root) {
  const YAML::Node nested = root["gimbal_pipeline"]["ros__parameters"]["controller"];
  if (nested) return nested;
  if (root["controller"]) return root["controller"];
  return root;
}

double degToRad(double degrees) { return degrees * kPi / 180.0; }

CommandLimiterConfig loadCommandLimiterConfig(const std::string& path) {
  CommandLimiterConfig config;
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node controller = controllerNode(root);
  const YAML::Node output = controller["output_filter"];
  const YAML::Node planner = controller["planner"];
  const YAML::Node mpc_planner = controller["mpc_planner"];
  const YAML::Node aim_planner = controller["aim_planner"];
  const YAML::Node limiter = controller["command_limiter"];
  const YAML::Node stabilizer = controller["target_stabilizer"];
  const YAML::Node fire_gate = controller["fire_gate"];
  const YAML::Node adapter = controller["output_adapter"];
  const YAML::Node feedback_alignment = controller["feedback_alignment"];
  const YAML::Node serial_command = controller["serial_command"];

  if (output) {
    config.enable_clamping = parseBool(output["enable_clamping"], config.enable_clamping);
    config.enable_rate_limiter = parseBool(output["enable_rate_limiter"], config.enable_rate_limiter);
    config.max_yaw_diff_rad = degToRad(parseDouble(output["max_yaw_diff"], 15.0));
    config.max_pitch_diff_rad = degToRad(parseDouble(output["max_pitch_diff"], 10.0));
    config.reference_rate_hz = parseDouble(output["one_euro_freq"], config.reference_rate_hz);
    const double yaw_rate_deg_per_frame = parseDouble(output["max_yaw_rate"], 1.5);
    const double pitch_rate_deg_per_frame = parseDouble(output["max_pitch_rate"], 1.0);
    config.max_yaw_step_rad = degToRad(yaw_rate_deg_per_frame);
    config.max_pitch_step_rad = degToRad(pitch_rate_deg_per_frame);
  }

  if (planner) {
    config.planner_mode = parseString(planner["mode"], config.planner_mode);
  }

  if (aim_planner) {
    config.max_yaw_acc_rad_s2 = parseDouble(aim_planner["max_yaw_acc"], config.max_yaw_acc_rad_s2);
    config.max_pitch_acc_rad_s2 = parseDouble(aim_planner["max_pitch_acc"], config.max_pitch_acc_rad_s2);
  }

  if (plannerModeUsesMpc(config.planner_mode) && mpc_planner) {
    config.max_yaw_acc_rad_s2 = parseDouble(mpc_planner["max_yaw_acc"], config.max_yaw_acc_rad_s2);
    config.max_pitch_acc_rad_s2 = parseDouble(mpc_planner["max_pitch_acc"], config.max_pitch_acc_rad_s2);
  }

  if (limiter) {
    config.enable = parseBool(limiter["enable"], config.enable);
    config.reset_timeout_s = parseDouble(limiter["reset_timeout_s"], config.reset_timeout_s);
    config.reference_rate_hz = parseDouble(limiter["reference_rate_hz"], config.reference_rate_hz);
    config.max_yaw_step_rad = parseDouble(limiter["max_yaw_step_rad"], config.max_yaw_step_rad);
    config.max_pitch_step_rad = parseDouble(limiter["max_pitch_step_rad"], config.max_pitch_step_rad);
    if (limiter["max_yaw_rate_rad_s"]) {
      config.max_yaw_step_rad = limiter["max_yaw_rate_rad_s"].as<double>() /
                                 std::max(config.reference_rate_hz, 1.0);
    }
    if (limiter["max_pitch_rate_rad_s"]) {
      config.max_pitch_step_rad = limiter["max_pitch_rate_rad_s"].as<double>() /
                                   std::max(config.reference_rate_hz, 1.0);
    }
    config.max_yaw_acc_rad_s2 = parseDouble(limiter["max_yaw_acc_rad_s2"], config.max_yaw_acc_rad_s2);
    config.max_pitch_acc_rad_s2 = parseDouble(limiter["max_pitch_acc_rad_s2"], config.max_pitch_acc_rad_s2);
  }

  if (stabilizer) {
    config.enable_target_stabilizer = parseBool(stabilizer["enable"], config.enable_target_stabilizer);
    config.target_stabilizer_max_yaw_jump_rad = degToRad(
        parseDouble(stabilizer["max_yaw_jump"], config.target_stabilizer_max_yaw_jump_rad * kRadToDeg));
    config.target_stabilizer_max_yaw_rate_rad_s = degToRad(
        parseDouble(stabilizer["max_yaw_rate"], config.target_stabilizer_max_yaw_rate_rad_s * kRadToDeg));
    config.target_stabilizer_hold_frames =
        parseInt(stabilizer["hold_frames"], config.target_stabilizer_hold_frames);
    config.target_stabilizer_freeze_on_unstable_state = parseBool(
        stabilizer["freeze_on_unstable_state"], config.target_stabilizer_freeze_on_unstable_state);
    config.lost_target_hold_s = parseDouble(
        stabilizer["lost_target_hold_s"], config.lost_target_hold_s);
  }

  if (fire_gate) {
    config.enable_fire_gate = parseBool(fire_gate["enable"], config.enable_fire_gate);
    config.fire_yaw_tolerance_rad = degToRad(
        parseDouble(fire_gate["yaw_tolerance"], config.fire_yaw_tolerance_rad * kRadToDeg));
    config.fire_pitch_tolerance_rad = degToRad(
        parseDouble(fire_gate["pitch_tolerance"], config.fire_pitch_tolerance_rad * kRadToDeg));
  }

  if (adapter) {
    config.sp_pitch_to_command_sign =
        parseDouble(adapter["sp_pitch_to_command_sign"], config.sp_pitch_to_command_sign);
    config.feedback_yaw_to_world_sign =
        parseDouble(adapter["feedback_yaw_to_world_sign"], config.feedback_yaw_to_world_sign);
  }

  if (feedback_alignment) {
    config.enable_feedback_alignment =
        parseBool(feedback_alignment["enable"], config.enable_feedback_alignment);
    config.feedback_alignment_offset_s =
        parseDouble(feedback_alignment["timestamp_offset_ms"],
                    config.feedback_alignment_offset_s * 1000.0) / 1000.0;
    config.feedback_alignment_max_sample_age_s =
        parseDouble(feedback_alignment["max_sample_age_ms"],
                    config.feedback_alignment_max_sample_age_s * 1000.0) / 1000.0;
    config.feedback_alignment_history_size =
        parseInt(feedback_alignment["history_size"], config.feedback_alignment_history_size);
  }

  if (serial_command) {
    config.serial_command_send_velocity =
        parseBool(serial_command["send_velocity"], config.serial_command_send_velocity);
    config.serial_command_send_acceleration =
        parseBool(serial_command["send_acceleration"], config.serial_command_send_acceleration);
    config.serial_command_velocity_mode =
        parseString(serial_command["velocity_mode"], config.serial_command_velocity_mode);
    config.serial_command_yaw_error_gain =
        parseDouble(serial_command["yaw_error_gain"], config.serial_command_yaw_error_gain);
    config.serial_command_pitch_error_gain =
        parseDouble(serial_command["pitch_error_gain"], config.serial_command_pitch_error_gain);
    config.serial_command_max_yaw_velocity_rad_s = degToRad(
        parseDouble(serial_command["max_yaw_velocity"],
                    config.serial_command_max_yaw_velocity_rad_s * kRadToDeg));
    config.serial_command_max_pitch_velocity_rad_s = degToRad(
        parseDouble(serial_command["max_pitch_velocity"],
                    config.serial_command_max_pitch_velocity_rad_s * kRadToDeg));
  }

  if (config.reference_rate_hz <= 0.0) throw std::invalid_argument("controller reference_rate_hz 必须 > 0");
  if (config.planner_mode != "aimer_feedback_error" && config.planner_mode != "aimer" &&
      !plannerModeUsesMpc(config.planner_mode)) {
    throw std::invalid_argument("controller.planner.mode 必须是 aimer_feedback_error/aimer 或 mpc/tinympc");
  }
  if (config.reset_timeout_s <= 0.0) throw std::invalid_argument("controller reset_timeout_s 必须 > 0");
  if (config.max_yaw_step_rad <= 0.0) throw std::invalid_argument("controller.output_filter.max_yaw_rate 必须 > 0");
  if (config.max_pitch_step_rad <= 0.0) throw std::invalid_argument("controller.output_filter.max_pitch_rate 必须 > 0");
  if (config.max_yaw_acc_rad_s2 <= 0.0 || !std::isfinite(config.max_yaw_acc_rad_s2) ||
      config.max_pitch_acc_rad_s2 <= 0.0 || !std::isfinite(config.max_pitch_acc_rad_s2)) {
    throw std::invalid_argument("controller max_yaw/max_pitch_acc 必须 > 0");
  }
  if (config.fire_yaw_tolerance_rad <= 0.0) throw std::invalid_argument("controller fire yaw tolerance 必须 > 0");
  if (config.fire_pitch_tolerance_rad <= 0.0) throw std::invalid_argument("controller fire pitch tolerance 必须 > 0");
  if (config.target_stabilizer_max_yaw_jump_rad <= 0.0) {
    throw std::invalid_argument("controller.target_stabilizer.max_yaw_jump 必须 > 0");
  }
  if (config.target_stabilizer_max_yaw_rate_rad_s <= 0.0) {
    throw std::invalid_argument("controller.target_stabilizer.max_yaw_rate 必须 > 0");
  }
  if (config.target_stabilizer_hold_frames < 0) {
    throw std::invalid_argument("controller.target_stabilizer.hold_frames 必须 >= 0");
  }
  if (config.lost_target_hold_s < 0.0) {
    throw std::invalid_argument("controller.target_stabilizer.lost_target_hold_s 必须 >= 0");
  }
  if (std::abs(std::abs(config.sp_pitch_to_command_sign) - 1.0) > 1e-6) {
    throw std::invalid_argument("controller.output_adapter.sp_pitch_to_command_sign 必须是 1 或 -1");
  }
  if (std::abs(std::abs(config.feedback_yaw_to_world_sign) - 1.0) > 1e-6) {
    throw std::invalid_argument("controller.output_adapter.feedback_yaw_to_world_sign 必须是 1 或 -1");
  }
  if (config.feedback_alignment_offset_s < 0.0 || !std::isfinite(config.feedback_alignment_offset_s)) {
    throw std::invalid_argument("controller.feedback_alignment.timestamp_offset_ms 必须是 >= 0 的有限数");
  }
  if (config.feedback_alignment_max_sample_age_s <= 0.0 ||
      !std::isfinite(config.feedback_alignment_max_sample_age_s)) {
    throw std::invalid_argument("controller.feedback_alignment.max_sample_age_ms 必须 > 0");
  }
  if (config.feedback_alignment_history_size < 2) {
    throw std::invalid_argument("controller.feedback_alignment.history_size 必须 >= 2");
  }
  if (config.serial_command_velocity_mode != "target_rate" &&
      !velocityModeUsesFeedbackError(config.serial_command_velocity_mode)) {
    throw std::invalid_argument("controller.serial_command.velocity_mode 必须是 target_rate 或 feedback_error");
  }
  if (!std::isfinite(config.serial_command_yaw_error_gain) ||
      !std::isfinite(config.serial_command_pitch_error_gain)) {
    throw std::invalid_argument("controller.serial_command yaw/pitch_error_gain 必须是有限数");
  }
  if (config.serial_command_max_yaw_velocity_rad_s <= 0.0 ||
      !std::isfinite(config.serial_command_max_yaw_velocity_rad_s) ||
      config.serial_command_max_pitch_velocity_rad_s <= 0.0 ||
      !std::isfinite(config.serial_command_max_pitch_velocity_rad_s)) {
    throw std::invalid_argument("controller.serial_command max_yaw/pitch_velocity 必须 > 0");
  }
  return config;
}

void loadHardwareConfig(Options& options) {
  const YAML::Node file_root = YAML::LoadFile(options.hardware_config);
  const YAML::Node root = file_root["hardware"] ? file_root["hardware"]
                          : (file_root["real_vehicle"] ? file_root["real_vehicle"] : file_root);

  const YAML::Node camera = root["camera"];
  if (camera) {
    options.camera_backend = parseString(camera["backend"], options.camera_backend);
    options.camera_source = parseString(camera["source"], options.camera_source);
    options.camera_sn = parseString(camera["camera_sn"], options.camera_sn);
    options.camera_index = parseInt(camera["device_index"], options.camera_index);
    options.camera_width = parseInt(camera["width"], options.camera_width);
    options.camera_height = parseInt(camera["height"], options.camera_height);
    options.camera_fps = parseDouble(camera["fps"], options.camera_fps);
    options.camera_exposure_time_us =
        parseDouble(camera["exposure_time_us"], options.camera_exposure_time_us);
    options.camera_gain = parseDouble(camera["gain"], options.camera_gain);
    options.camera_flip_image = parseBool(camera["flip_image"], options.camera_flip_image);
    options.camera_info_path = parseString(camera["camera_info"], options.camera_info_path);
    options.calibration_mode = parseString(camera["calibration_mode"], options.calibration_mode);
    const YAML::Node extrinsics = camera["camera_to_barrel"];
    if (extrinsics) {
      options.camera_to_barrel.xyz = readVector3(
          extrinsics["xyz"], options.camera_to_barrel.xyz, "camera.camera_to_barrel.xyz");
      options.camera_to_barrel.rpy = readVector3(
          extrinsics["rpy"], options.camera_to_barrel.rpy, "camera.camera_to_barrel.rpy");
    }
  }

  const YAML::Node serial = root["serial"];
  if (serial) {
    options.serial_port = parseString(serial["port"], options.serial_port);
    options.serial_baudrate = parseInt(serial["baudrate"], options.serial_baudrate);
    options.serial_protocol = parseString(serial["protocol"], options.serial_protocol);
    options.serial_tx_protocol = options.serial_protocol;
    options.serial_rx_protocol = options.serial_protocol;
    options.serial_tx_protocol = parseString(serial["tx_protocol"], options.serial_tx_protocol);
    options.serial_rx_protocol = parseString(serial["rx_protocol"], options.serial_rx_protocol);
    options.infantry32_tail_fields = parseString(
        serial["infantry32_tail_fields"], options.infantry32_tail_fields);
    options.command_angles_in_degrees = parseBool(
        serial["command_angles_in_degrees"], options.command_angles_in_degrees);
    options.feedback_angles_in_degrees = parseBool(
        serial["feedback_angles_in_degrees"], options.feedback_angles_in_degrees);
    options.serial_read_timeout_ms =
        parseInt(serial["read_timeout_ms"], options.serial_read_timeout_ms);
    options.serial_write_timeout_ms =
        parseInt(serial["write_timeout_ms"], options.serial_write_timeout_ms);
    options.feedback_timeout_ms =
        parseInt(serial["feedback_timeout_ms"], options.feedback_timeout_ms);
    options.require_feedback = parseBool(serial["require_feedback"], options.require_feedback);
  }

  const YAML::Node detector = root["detector"];
  if (detector) options.enemy_color = parseString(detector["enemy_color"], options.enemy_color);

  const YAML::Node controller = root["controller"];
  if (controller) {
    options.bullet_speed = parseDouble(controller["bullet_speed"], options.bullet_speed);
  }

  const YAML::Node safety = root["safety"];
  if (safety) {
    options.dry_run = parseBool(safety["dry_run"], options.dry_run);
    options.enable_fire = parseBool(safety["enable_fire"], options.enable_fire);
  }
}

Options parseOptions(int argc, char** argv) {
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (auto value = optionValue(argc, argv, i, arg, "--hardware-config"); !value.empty()) {
      options.hardware_config = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--real-config"); !value.empty()) {
      options.hardware_config = value;
    }
  }

  loadHardwareConfig(options);

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      std::printf(
          "Usage: standard [options]\n"
          "  --hardware-config PATH   硬件 YAML，默认 configs/hardware.yaml\n"
          "  --sp25-config PATH       SP25 算法 YAML，默认 configs/standard3.yaml\n"
          "  --controller-config PATH 控制输出 YAML，默认 configs/controller.yaml\n"
          "  --sp25-device NAME       覆盖 OpenVINO device，例如 GPU 或 CPU\n"
          "  --camera-backend NAME    hik | opencv\n"
          "  --camera-source PATH     OpenCV 视频文件、流地址或设备路径\n"
          "  --camera-sn SN           海康相机序列号，留空取第一台\n"
          "  --camera-index N         OpenCV 设备序号\n"
          "  --camera-info PATH       ROS camera_info YAML\n"
          "  --exposure-time-us US    工业相机曝光覆盖\n"
          "  --gain VALUE             工业相机增益覆盖\n"
          "  --flip-image             旋转工业相机画面 180 度\n"
          "  --serial-port PATH       串口设备\n"
          "  --baudrate N             串口波特率\n"
          "  --serial-tx-protocol NAME  infantry | infantry_16 | infantry_32\n"
          "  --serial-rx-protocol NAME  infantry | infantry_16 | infantry_32\n"
          "  --no-serial-send       打开串口收反馈，但不下发控制包\n"
          "  --enemy-color COLOR      red | blue\n"
          "  --bullet-speed MPS       弹速覆盖\n"
          "  --dry-run                不打开串口、不下发控制\n"
          "  --enable-fire            允许把 SP25 开火判定发给下位机\n"
          "  --display                打开 OpenCV 调试窗口\n"
          "  --web-view               开启 MJPEG 调试流\n"
          "  --web-host ADDR          Web 绑定地址，默认 0.0.0.0\n"
          "  --web-port N             Web 端口，默认 8080\n"
          "  --web-frame-step N       每 N 帧推送一次 Web 图像\n"
          "  --max-frames N           处理 N 帧后退出\n");
      std::exit(0);
    } else if (arg == "--dry-run") {
      options.dry_run = true;
    } else if (arg == "--enable-fire") {
      options.enable_fire = true;
    } else if (arg == "--display") {
      options.display = true;
    } else if (arg == "--web-view") {
      options.web_view = true;
    } else if (arg == "--no-serial-send") {
      options.serial_send = false;
    } else if (arg == "--flip-image") {
      options.camera_flip_image = true;
    } else if (arg == "--hardware-config" || arg == "--real-config") {
      ++i;
    } else if (arg.rfind("--hardware-config=", 0) == 0 ||
               arg.rfind("--real-config=", 0) == 0) {
      continue;
    } else if (auto value = optionValue(argc, argv, i, arg, "--sp25-config"); !value.empty()) {
      options.sp25_config = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--controller-config"); !value.empty()) {
      options.controller_config = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--sp25-device"); !value.empty()) {
      options.sp25_device = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-backend"); !value.empty()) {
      options.camera_backend = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-source"); !value.empty()) {
      options.camera_source = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-sn"); !value.empty()) {
      options.camera_sn = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-index"); !value.empty()) {
      options.camera_index = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--camera-info"); !value.empty()) {
      options.camera_info_path = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--exposure-time-us"); !value.empty()) {
      options.camera_exposure_time_us = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--gain"); !value.empty()) {
      options.camera_gain = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-port"); !value.empty()) {
      options.serial_port = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--baudrate"); !value.empty()) {
      options.serial_baudrate = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-protocol"); !value.empty()) {
      options.serial_protocol = value;
      options.serial_tx_protocol = value;
      options.serial_rx_protocol = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-tx-protocol"); !value.empty()) {
      options.serial_tx_protocol = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--serial-rx-protocol"); !value.empty()) {
      options.serial_rx_protocol = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--infantry32-tail-fields"); !value.empty()) {
      options.infantry32_tail_fields = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--enemy-color"); !value.empty()) {
      options.enemy_color = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--bullet-speed"); !value.empty()) {
      options.bullet_speed = std::stod(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-host"); !value.empty()) {
      options.web_host = value;
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-port"); !value.empty()) {
      options.web_port = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-jpeg-quality"); !value.empty()) {
      options.web_jpeg_quality = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-max-width"); !value.empty()) {
      options.web_max_width = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--web-frame-step"); !value.empty()) {
      options.web_frame_step = std::stoi(value);
    } else if (auto value = optionValue(argc, argv, i, arg, "--max-frames"); !value.empty()) {
      options.max_frames = std::stoi(value);
    } else {
      throw std::invalid_argument("未知或不完整参数: " + arg);
    }
  }

  if (options.camera_info_path.empty()) {
    throw std::invalid_argument("缺少 camera_info，请在 hardware.yaml 或 --camera-info 中配置");
  }
  if (options.camera_backend != "opencv" && options.camera_backend != "hik") {
    throw std::invalid_argument("SP25 实车入口当前支持 hik 或 opencv，相机后端为: " + options.camera_backend);
  }
  if (options.enemy_color != "red" && options.enemy_color != "blue") {
    throw std::invalid_argument("SP25 敌方颜色当前支持 red 或 blue");
  }
  if (options.serial_read_timeout_ms < 0) {
    throw std::invalid_argument("串口读取超时必须 >= 0");
  }
  if (options.serial_write_timeout_ms < 0) {
    throw std::invalid_argument("串口写入超时必须 >= 0");
  }
  if (options.feedback_timeout_ms <= 0) {
    throw std::invalid_argument("反馈超时必须 > 0");
  }
  if (options.web_port <= 0 || options.web_port > 65535) {
    throw std::invalid_argument("Web 端口必须在 1..65535");
  }
  if (options.web_frame_step <= 0) {
    throw std::invalid_argument("web_frame_step 必须 > 0");
  }
  hfut::io::InfantryPacketLayout tx_layout;
  if (!hfut::io::parseInfantryPacketLayout(options.serial_tx_protocol, tx_layout)) {
    throw std::invalid_argument("serial tx_protocol 不合法: " + options.serial_tx_protocol);
  }
  hfut::io::InfantryPacketLayout rx_layout;
  if (!hfut::io::parseInfantryPacketLayout(options.serial_rx_protocol, rx_layout)) {
    throw std::invalid_argument("serial rx_protocol 不合法: " + options.serial_rx_protocol);
  }
  hfut::io::Infantry32TailFields fields;
  if (!hfut::io::parseInfantry32TailFields(options.infantry32_tail_fields, fields)) {
    throw std::invalid_argument("infantry32_tail_fields 不合法: " + options.infantry32_tail_fields);
  }
  return options;
}

hfut::video::CameraCalibration loadCalibration(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  hfut::video::CameraCalibration calibration;
  if (!root["image_width"] || !root["image_height"] ||
      !root["camera_matrix"] || !root["camera_matrix"]["data"]) {
    throw std::invalid_argument("camera_info YAML 缺少 image_width/image_height/camera_matrix.data");
  }
  calibration.width = root["image_width"].as<int>();
  calibration.height = root["image_height"].as<int>();
  const auto matrix = root["camera_matrix"]["data"];
  if (!matrix.IsSequence() || matrix.size() != 9) {
    throw std::invalid_argument("camera_matrix.data 必须包含 9 个数");
  }
  for (size_t i = 0; i < 9; ++i) calibration.k[i] = matrix[i].as<double>();
  const auto distortion = root["distortion_coefficients"];
  if (distortion && distortion["data"]) calibration.d = distortion["data"].as<std::vector<double>>();
  if (calibration.d.empty()) calibration.d.assign(5, 0.0);
  return calibration;
}

hfut::CameraIntrinsics toIntrinsics(const hfut::video::CameraCalibration& calibration) {
  hfut::CameraIntrinsics intrinsics;
  intrinsics.width = calibration.width;
  intrinsics.height = calibration.height;
  intrinsics.fx = calibration.k[0];
  intrinsics.fy = calibration.k[4];
  intrinsics.cx = calibration.k[2];
  intrinsics.cy = calibration.k[5];
  for (size_t i = 0; i < 5 && i < calibration.d.size(); ++i) {
    intrinsics.distortion[i] = calibration.d[i];
  }
  return intrinsics;
}

std::vector<double> matrixToVector(const Eigen::Matrix3d& matrix) {
  std::vector<double> data;
  data.reserve(9);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) data.push_back(matrix(r, c));
  }
  return data;
}

std::string writeRuntimeSp25Config(
    const Options& options, const hfut::video::CameraCalibration& calibration) {
  YAML::Node yaml = YAML::LoadFile(options.sp25_config);
  yaml["enemy_color"] = options.enemy_color;
  if (!options.sp25_device.empty()) yaml["device"] = options.sp25_device;
  yaml["image_width"] = calibration.width;
  yaml["image_height"] = calibration.height;
  yaml["camera_matrix"] = std::vector<double>(calibration.k.begin(), calibration.k.end());
  std::vector<double> distort = calibration.d;
  distort.resize(5, 0.0);
  yaml["distort_coeffs"] = distort;
  yaml["R_camera2gimbal"] = matrixToVector(options.camera_to_barrel.cameraOpticalToBarrelRotation());
  yaml["t_camera2gimbal"] = std::vector<double>{
      options.camera_to_barrel.xyz.x(),
      options.camera_to_barrel.xyz.y(),
      options.camera_to_barrel.xyz.z()};

  const std::filesystem::path output_path(options.runtime_sp25_config);
  if (!output_path.parent_path().empty()) {
    std::filesystem::create_directories(output_path.parent_path());
  }
  std::ofstream out(output_path);
  if (!out) throw std::runtime_error("无法写入 SP25 运行配置: " + output_path.string());
  out << yaml;
  return output_path.string();
}

std::unique_ptr<hfut::io::CameraSource> createCameraSource(
    const Options& options, const hfut::CameraIntrinsics& intrinsics) {
  if (options.camera_backend == "opencv") {
    hfut::io::OpenCvCameraSourceConfig config;
    config.source = options.camera_source;
    config.device_index = options.camera_index;
    config.width = options.camera_width;
    config.height = options.camera_height;
    config.fps = options.camera_fps;
    config.exposure = options.camera_exposure_time_us;
    config.gain = options.camera_gain;
    config.set_exposure = options.camera_exposure_time_us > 0.0;
    config.set_gain = options.camera_gain > 0.0;
    config.intrinsics = intrinsics;
    return std::make_unique<hfut::io::OpenCvCameraSource>(config);
  }

  if (options.camera_backend == "hik") {
#ifdef HFUT_HAS_HIK_CAMERA
    hfut::io::HikCameraSourceConfig config;
    config.camera_sn = options.camera_sn;
    config.width = options.camera_width;
    config.height = options.camera_height;
    config.fps = options.camera_fps;
    config.exposure_time_us = options.camera_exposure_time_us;
    config.gain = options.camera_gain;
    config.flip_image = options.camera_flip_image;
    config.intrinsics = intrinsics;
    return std::make_unique<hfut::io::HikCameraSource>(config);
#else
    throw std::runtime_error("当前二进制未开启 HFUT_ENABLE_HIK_CAMERA，无法使用海康相机");
#endif
  }

  throw std::runtime_error("未知相机后端: " + options.camera_backend);
}

hfut::io::InfantrySerialConfig makeSerialConfig(const Options& options) {
  hfut::io::InfantrySerialConfig config;
  config.port = options.serial_port;
  config.baudrate = options.serial_baudrate;
  config.command_angles_in_degrees = options.command_angles_in_degrees;
  config.feedback_angles_in_degrees = options.feedback_angles_in_degrees;
  config.allow_fire = options.enable_fire;
  config.read_timeout_ms = options.serial_read_timeout_ms;
  config.write_timeout_ms = options.serial_write_timeout_ms;
  hfut::io::parseInfantryPacketLayout(options.serial_tx_protocol, config.tx_layout);
  hfut::io::parseInfantryPacketLayout(options.serial_rx_protocol, config.rx_layout);
  hfut::io::parseInfantry32TailFields(options.infantry32_tail_fields, config.tail_fields);
  return config;
}

Eigen::Quaterniond feedbackQuaternion(
    const hfut::io::SerialFeedback& feedback, const CommandLimiterConfig& config) {
  const Eigen::Matrix3d gimbal_to_world =
      (Eigen::AngleAxisd(
           config.feedback_yaw_to_world_sign * feedback.yaw_rad, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-feedback.pitch_rad, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(feedback.roll_rad, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return Eigen::Quaterniond(gimbal_to_world).normalized();
}

double aimDistance(const auto_aim::Aimer& aimer, const io::Command& command) {
  if (!command.control) return 0.0;
  if (command.horizon_distance > 0.0) return command.horizon_distance;
  if (!aimer.debug_aim_point.valid) return 0.0;
  const Eigen::Vector3d xyz = aimer.debug_aim_point.xyza.head<3>();
  return std::hypot(xyz.x(), xyz.y());
}

double targetDistance(const auto_aim::Target& target) {
  double distance = 0.0;
  for (const auto& xyza : target.armor_xyza_list()) {
    const double d = std::hypot(xyza.x(), xyza.y());
    if (d > 0.0 && (distance <= 0.0 || d < distance)) distance = d;
  }
  return distance;
}

hfut::GimbalCommand convertCommand(
    const io::Command& sp_command, const auto_aim::Aimer& aimer,
    const hfut::io::SerialFeedback& feedback, bool enable_fire,
    const CommandLimiterConfig& command_config) {
  hfut::GimbalCommand command;
  command.yaw = sp_command.control
                    ? tools::limit_rad(command_config.feedback_yaw_to_world_sign * sp_command.yaw)
                    : feedback.yaw_rad;
  command.pitch = sp_command.control
                      ? command_config.sp_pitch_to_command_sign * sp_command.pitch
                      : feedback.pitch_rad;
  command.yaw_diff = tools::limit_rad(command.yaw - feedback.yaw_rad);
  command.pitch_diff = command.pitch - feedback.pitch_rad;
  command.distance = aimDistance(aimer, sp_command);
  command.fire_advice = enable_fire && sp_command.shoot;
  command.mode = sp_command.control ? hfut::GimbalMode::normal_measurement
                                    : hfut::GimbalMode::no_valid_measurement;
  return command;
}

void drawText(cv::Mat& image, int& y, const std::string& text,
              const cv::Scalar& color = cv::Scalar(80, 255, 80)) {
  if (image.empty()) return;
  constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;
  constexpr double kScale = 0.55;
  constexpr int kThickness = 1;
  int baseline = 0;
  const cv::Size size = cv::getTextSize(text, kFont, kScale, kThickness, &baseline);
  const int x = 10;
  const int top = std::max(0, y - size.height - 5);
  cv::rectangle(image, cv::Rect(x - 4, top, size.width + 8, size.height + baseline + 8),
                cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(image, text, cv::Point(x, y), kFont, kScale, color, kThickness, cv::LINE_AA);
  y += size.height + baseline + 10;
}

void drawArmors(cv::Mat& image, const std::list<auto_aim::Armor>& armors) {
  for (const auto& armor : armors) {
    if (armor.points.empty()) continue;
    const cv::Scalar color = armor.color == auto_aim::Color::red
                                 ? cv::Scalar(0, 0, 255)
                                 : cv::Scalar(255, 0, 0);
    for (size_t i = 0; i < armor.points.size(); ++i) {
      cv::line(image, armor.points[i], armor.points[(i + 1) % armor.points.size()], color, 2);
      cv::circle(image, armor.points[i], 3, cv::Scalar(0, 255, 0), cv::FILLED);
    }
    const std::string label = auto_aim::COLORS[armor.color] + " " +
        auto_aim::ARMOR_NAMES[armor.name] + " " + auto_aim::ARMOR_TYPES[armor.type];
    cv::putText(image, label, armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
}

void drawCrosshair(cv::Mat& image) {
  const cv::Point center(image.cols / 2, image.rows / 2);
  cv::line(image, {center.x - 18, center.y}, {center.x + 18, center.y}, cv::Scalar(80, 255, 80), 1);
  cv::line(image, {center.x, center.y - 18}, {center.x, center.y + 18}, cv::Scalar(80, 255, 80), 1);
}

void drawOverlay(cv::Mat& image, const hfut::io::DebugMjpegStatus& status) {
  int y = 24;
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer),
                "SP25 frames=%llu fps=%.1f armors=%d tracked=%d state=%s",
                static_cast<unsigned long long>(status.frames), status.fps,
                status.armors, status.tracked, status.track_state.c_str());
  drawText(image, y, buffer);

  std::snprintf(buffer, sizeof(buffer),
                "fb yaw=%.2f pitch=%.2f cmd yaw=%.2f pitch=%.2f mode=%d",
                status.feedback_yaw_deg, status.feedback_pitch_deg,
                status.command_yaw_deg, status.command_pitch_deg, status.mode);
  drawText(image, y, buffer, cv::Scalar(255, 220, 120));

  std::snprintf(buffer, sizeof(buffer),
                "distance=%.2fm fire=%d dry=%d fire_enabled=%d fb_age=%.0fms",
                status.distance_m, status.fire ? 1 : 0, status.dry_run ? 1 : 0,
                status.fire_enabled ? 1 : 0, status.feedback_age_ms);
  drawText(image, y, buffer, status.fire ? cv::Scalar(80, 255, 80) : cv::Scalar(180, 180, 180));
}

int run(const Options& options) {
  const auto source_calibration = loadCalibration(options.camera_info_path);
  const auto calibration_mode = hfut::video::parseCalibrationMode(options.calibration_mode);
  const int solver_width = options.camera_width > 0 ? options.camera_width : source_calibration.width;
  const int solver_height = options.camera_height > 0 ? options.camera_height : source_calibration.height;
  const auto solver_calibration = hfut::video::adaptCalibration(
      source_calibration, solver_width, solver_height, calibration_mode);
  const auto adapted_config_path = writeRuntimeSp25Config(options, solver_calibration);

  auto camera = createCameraSource(options, toIntrinsics(source_calibration));
  if (!camera->open()) throw std::runtime_error("相机打开失败: " + camera->errorMessage());

  const auto serial_config = makeSerialConfig(options);
  const auto command_limiter_config = loadCommandLimiterConfig(options.controller_config);
  hfut::io::HfutSerialGimbalConfig gimbal_config;
  gimbal_config.serial = serial_config;
  gimbal_config.history_size = static_cast<std::size_t>(command_limiter_config.feedback_alignment_history_size);
  gimbal_config.timestamp_offset_s = command_limiter_config.feedback_alignment_offset_s;
  gimbal_config.max_sample_age_s = command_limiter_config.feedback_alignment_max_sample_age_s;
  gimbal_config.send_velocity = command_limiter_config.serial_command_send_velocity;
  gimbal_config.send_acceleration = command_limiter_config.serial_command_send_acceleration;
  hfut::io::HfutSerialGimbal gimbal(gimbal_config);
  if (!options.dry_run && !gimbal.open()) {
    throw std::runtime_error("串口打开失败: " + gimbal.errorMessage());
  }
  SimpleCommandGuard command_guard(command_limiter_config);
  FinalVelocityAccelerationAdapter mpc_motion_adapter(command_limiter_config);

  auto_aim::YOLO detector(adapted_config_path, false);
  auto_aim::Solver solver(adapted_config_path);
  auto_aim::Tracker tracker(adapted_config_path, solver);
  auto_aim::Aimer aimer(adapted_config_path);
  auto_aim::Shooter shooter(adapted_config_path);
  const bool use_mpc_planner = plannerModeUsesMpc(command_limiter_config.planner_mode);
  std::unique_ptr<auto_aim::Planner> mpc_planner;
  if (use_mpc_planner) {
    mpc_planner = std::make_unique<auto_aim::Planner>(options.controller_config);
  }

  if (options.display) {
    cv::namedWindow("hfut sp25 real", cv::WINDOW_NORMAL);
    cv::resizeWindow("hfut sp25 real", 960, 720);
  }

  std::unique_ptr<hfut::io::DebugMjpegServer> web_server;
  if (options.web_view) {
    hfut::io::DebugMjpegServerConfig web_config;
    web_config.host = options.web_host;
    web_config.port = static_cast<uint16_t>(options.web_port);
    web_config.jpeg_quality = options.web_jpeg_quality;
    web_config.max_width = options.web_max_width;
    web_server = std::make_unique<hfut::io::DebugMjpegServer>(web_config);
    if (!web_server->start()) {
      throw std::runtime_error("Web 调试服务启动失败: " + web_server->errorMessage());
    }
    std::printf("[standard] web_view=%s stream=/stream.mjpg status=/status.json\n",
                web_server->url().c_str());
  }

  std::printf(
      "[standard] camera=%s serial=%s:%s@%d read_timeout=%dms write_timeout=%dms "
      "dry_run=%s serial_send=%s fire=%s enemy=%s bullet=%.2f\n",
      options.camera_backend.c_str(), hfut::io::infantryPacketLayoutName(serial_config.tx_layout),
      options.serial_port.c_str(), options.serial_baudrate, options.serial_read_timeout_ms,
      options.serial_write_timeout_ms,
      options.dry_run ? "true" : "false",
      options.serial_send ? "true" : "false",
      options.enable_fire ? "enabled" : "disabled",
      options.enemy_color.c_str(), options.bullet_speed);
  std::printf("[standard] sp25_config=%s runtime_config=%s calibration_mode=%s\n",
              options.sp25_config.c_str(), adapted_config_path.c_str(),
              hfut::video::calibrationModeName(calibration_mode));
  std::printf(
      "[standard] controller_config=%s planner=%s guard=%s yaw_step=%.2fdeg pitch_step=%.2fdeg "
      "yaw_acc=%.2f pitch_acc=%.2f fire_gate=%s yaw_tol=%.2fdeg pitch_tol=%.2fdeg "
      "sp_pitch_to_cmd=%.0f yaw_world_sign=%.0f outlier_guard=%s jump=%.1fdeg "
      "target_rate=%.1fdeg/s hold=%d lost_hold=%.2fs fb_align=%s offset=%.1fms max_age=%.1fms "
      "send_v=%s send_acc=%s vel_mode=%s vel_k=%.1f/%.1f vel_lim=%.0f/%.0fdeg/s\n",
      options.controller_config.c_str(), command_limiter_config.planner_mode.c_str(),
      command_limiter_config.enable ? "on" : "off",
      command_limiter_config.max_yaw_step_rad * kRadToDeg,
      command_limiter_config.max_pitch_step_rad * kRadToDeg,
      command_limiter_config.max_yaw_acc_rad_s2,
      command_limiter_config.max_pitch_acc_rad_s2,
      command_limiter_config.enable_fire_gate ? "on" : "off",
      command_limiter_config.fire_yaw_tolerance_rad * kRadToDeg,
      command_limiter_config.fire_pitch_tolerance_rad * kRadToDeg,
      command_limiter_config.sp_pitch_to_command_sign,
      command_limiter_config.feedback_yaw_to_world_sign,
      command_limiter_config.enable_target_stabilizer ? "on" : "off",
      command_limiter_config.target_stabilizer_max_yaw_jump_rad * kRadToDeg,
      command_limiter_config.target_stabilizer_max_yaw_rate_rad_s * kRadToDeg,
      command_limiter_config.target_stabilizer_hold_frames,
      command_limiter_config.lost_target_hold_s,
      command_limiter_config.enable_feedback_alignment ? "on" : "off",
      command_limiter_config.feedback_alignment_offset_s * 1000.0,
      command_limiter_config.feedback_alignment_max_sample_age_s * 1000.0,
      command_limiter_config.serial_command_send_velocity ? "on" : "off",
      command_limiter_config.serial_command_send_acceleration ? "on" : "off",
      command_limiter_config.serial_command_velocity_mode.c_str(),
      command_limiter_config.serial_command_yaw_error_gain,
      command_limiter_config.serial_command_pitch_error_gain,
      command_limiter_config.serial_command_max_yaw_velocity_rad_s * kRadToDeg,
      command_limiter_config.serial_command_max_pitch_velocity_rad_s * kRadToDeg);

  hfut::io::SerialFeedback latest_feedback;
  latest_feedback.bullet_speed = options.bullet_speed;
  bool have_feedback = options.dry_run || !options.require_feedback;
  auto last_feedback_time = std::chrono::steady_clock::now();
  auto last_feedback_wait_log = std::chrono::steady_clock::now();
  auto last_feedback_heartbeat_time = std::chrono::steady_clock::now();
  auto last_log = std::chrono::steady_clock::now();
  const auto run_start = std::chrono::steady_clock::now();
  uint64_t frames = 0;
  int processed = 0;
  const auto elapsedMs = [](const auto& start, const auto& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };
  const auto refreshLatestFeedback = [&]() {
    if (options.dry_run) return false;
    hfut::io::SerialFeedback feedback;
    auto feedback_time = std::chrono::steady_clock::now();
    if (!gimbal.latest(feedback, &feedback_time)) return false;
    latest_feedback = feedback;
    if (latest_feedback.bullet_speed < 14.0) latest_feedback.bullet_speed = options.bullet_speed;
    have_feedback = true;
    last_feedback_time = feedback_time;
    return true;
  };

  while (!g_stop.load() && (options.max_frames < 0 || processed < options.max_frames)) {
    const auto loop_start = std::chrono::steady_clock::now();

    const auto serial_rx_start = loop_start;
    refreshLatestFeedback();
    const auto serial_rx_end = std::chrono::steady_clock::now();

    const auto feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_feedback_time);
    const bool feedback_ready = options.dry_run || !options.require_feedback ||
        (have_feedback && feedback_age.count() <= options.feedback_timeout_ms);
    if (!feedback_ready) {
      const auto now = std::chrono::steady_clock::now();
      if (!options.dry_run && options.serial_send &&
          now - last_feedback_heartbeat_time >= std::chrono::milliseconds(20)) {
        hfut::GimbalCommand heartbeat;
        heartbeat.yaw = latest_feedback.yaw_rad;
        heartbeat.pitch = latest_feedback.pitch_rad;
        heartbeat.fire_advice = false;
        heartbeat.mode = hfut::GimbalMode::no_valid_measurement;
        gimbal.send(heartbeat);
        last_feedback_heartbeat_time = now;
      }
      if (now - last_feedback_wait_log > std::chrono::seconds(1)) {
        std::printf("[standard] 等待串口反馈 %s last_age=%lldms\n",
                    options.serial_port.c_str(),
                    static_cast<long long>(have_feedback ? feedback_age.count() : -1));
        std::fflush(stdout);
        last_feedback_wait_log = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    hfut::CameraFrame frame;
    const auto capture_start = std::chrono::steady_clock::now();
    if (!camera->read(frame, std::chrono::milliseconds(200))) {
      std::fprintf(stderr, "[standard] 相机读帧超时: %s\n", camera->errorMessage().c_str());
      continue;
    }
    const auto capture_end = std::chrono::steady_clock::now();
    const auto frame_time = frame.timestamp == std::chrono::steady_clock::time_point{}
                                ? capture_end
                                : frame.timestamp;
    refreshLatestFeedback();

    hfut::io::SerialFeedback aligned_feedback = latest_feedback;
    auto aligned_feedback_time = last_feedback_time;
    bool used_aligned_feedback = false;
    if (!options.dry_run && command_limiter_config.enable_feedback_alignment) {
      used_aligned_feedback = gimbal.sampleAt(frame_time, aligned_feedback, &aligned_feedback_time);
      if (!used_aligned_feedback) {
        aligned_feedback = latest_feedback;
        aligned_feedback_time = last_feedback_time;
      }
    }
    if (aligned_feedback.bullet_speed < 14.0) aligned_feedback.bullet_speed = options.bullet_speed;
    const auto current_feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_feedback_time);
    const double aligned_feedback_age_ms = options.dry_run
                                               ? 0.0
                                               : elapsedMs(aligned_feedback_time, frame_time);
    const double aligned_delta_yaw = tools::limit_rad(latest_feedback.yaw_rad - aligned_feedback.yaw_rad);
    const double aligned_delta_pitch = latest_feedback.pitch_rad - aligned_feedback.pitch_rad;

    const auto adapted_calibration = hfut::video::adaptCalibration(
        source_calibration, frame.image.cols, frame.image.rows, calibration_mode);
    frame.intrinsics = toIntrinsics(adapted_calibration);
    frame.gimbal_yaw = aligned_feedback.yaw_rad;
    frame.gimbal_pitch = aligned_feedback.pitch_rad;
    const auto timestamp = frame_time;
    solver.set_R_gimbal2world(feedbackQuaternion(aligned_feedback, command_limiter_config));

    const auto detect_start = std::chrono::steady_clock::now();
    auto armors = detector.detect(frame.image, static_cast<int>(frame.seq));
    const auto detect_end = std::chrono::steady_clock::now();

    auto track_armors = armors;
    const auto track_start = detect_end;
    auto targets = tracker.track(track_armors, timestamp);
    const auto track_end = std::chrono::steady_clock::now();

    const double bullet_speed = latest_feedback.bullet_speed >= 14.0
                                    ? latest_feedback.bullet_speed
                                    : options.bullet_speed;
    const auto aim_start = track_end;
    io::Command sp_command{false, false, 0.0, 0.0};
    auto_aim::Plan mpc_plan{};
    bool have_mpc_plan = false;
    if (use_mpc_planner) {
      if (!targets.empty()) {
        try {
          std::optional<auto_aim::Target> target{targets.front()};
          mpc_plan = mpc_planner->plan(target, bullet_speed);
          have_mpc_plan = mpc_plan.control;
          sp_command.control = mpc_plan.control;
          sp_command.shoot = mpc_plan.fire;
          sp_command.yaw = mpc_plan.yaw;
          sp_command.pitch = mpc_plan.pitch;
          sp_command.horizon_distance = targetDistance(*target);
        } catch (const std::exception& e) {
          std::fprintf(stderr, "[standard] MPC 规划失败: %s\n", e.what());
        }
      }
    } else {
      sp_command = aimer.aim(targets, timestamp, bullet_speed);
    }
    const Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    if (!use_mpc_planner) {
      sp_command.shoot = shooter.shoot(sp_command, aimer, targets, gimbal_pos);
    }
    hfut::GimbalCommand command = convertCommand(
        sp_command, aimer, latest_feedback, options.enable_fire, command_limiter_config);
    if (use_mpc_planner && have_mpc_plan) {
      command.yaw_vel = command_limiter_config.feedback_yaw_to_world_sign * mpc_plan.yaw_vel;
      command.pitch_vel = command_limiter_config.sp_pitch_to_command_sign * mpc_plan.pitch_vel;
      if (velocityModeUsesFeedbackError(command_limiter_config.serial_command_velocity_mode)) {
        command.yaw_vel += command_limiter_config.serial_command_yaw_error_gain * command.yaw_diff;
        command.pitch_vel += command_limiter_config.serial_command_pitch_error_gain * command.pitch_diff;
      }
      command.yaw_vel = std::clamp(command.yaw_vel,
                                   -command_limiter_config.serial_command_max_yaw_velocity_rad_s,
                                   command_limiter_config.serial_command_max_yaw_velocity_rad_s);
      command.pitch_vel = std::clamp(command.pitch_vel,
                                     -command_limiter_config.serial_command_max_pitch_velocity_rad_s,
                                     command_limiter_config.serial_command_max_pitch_velocity_rad_s);
      mpc_motion_adapter.apply(command, std::chrono::steady_clock::now());
    } else if (use_mpc_planner) {
      mpc_motion_adapter.reset();
    }
    const double raw_desired_yaw = command.yaw;
    const double raw_desired_pitch = command.pitch;
    if (!use_mpc_planner) {
      command_guard.apply(command, latest_feedback, tracker.state(), std::chrono::steady_clock::now());
    }
    const double desired_yaw = command.yaw;
    const double desired_pitch = command.pitch;
    const FireGateResult fire_gate = applyFireGate(
        command, raw_desired_yaw, raw_desired_pitch, command_limiter_config);
    const auto aim_end = std::chrono::steady_clock::now();

    const auto serial_tx_start = aim_end;
    bool serial_send_ok = true;
    if (!options.dry_run && options.serial_send) {
      serial_send_ok = gimbal.send(command);
    }
    const auto serial_tx_end = std::chrono::steady_clock::now();

    ++frames;
    ++processed;

    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - run_start).count();
    const double runtime_fps = elapsed_s > 0.0 ? static_cast<double>(frames) / elapsed_s : 0.0;

    hfut::io::DebugMjpegStatus web_status;
    web_status.frames = frames;
    web_status.fps = runtime_fps;
    web_status.latency_ms = elapsedMs(detect_start, aim_end);
    web_status.armors = static_cast<int>(armors.size());
    web_status.tracked = targets.empty() ? 0 : 1;
    web_status.track_state = tracker.state();
    web_status.mode = static_cast<int>(command.mode);
    web_status.feedback_yaw_deg = latest_feedback.yaw_rad * kRadToDeg;
    web_status.feedback_pitch_deg = latest_feedback.pitch_rad * kRadToDeg;
    web_status.aligned_feedback_yaw_deg = aligned_feedback.yaw_rad * kRadToDeg;
    web_status.aligned_feedback_pitch_deg = aligned_feedback.pitch_rad * kRadToDeg;
    web_status.feedback_alignment_delta_yaw_deg = aligned_delta_yaw * kRadToDeg;
    web_status.feedback_alignment_delta_pitch_deg = aligned_delta_pitch * kRadToDeg;
    web_status.aligned_feedback_age_ms = aligned_feedback_age_ms;
    web_status.command_yaw_deg = command.yaw * kRadToDeg;
    web_status.command_pitch_deg = command.pitch * kRadToDeg;
    web_status.command_yaw_vel_rad_s = command.yaw_vel;
    web_status.command_pitch_vel_rad_s = command.pitch_vel;
    web_status.command_yaw_acc_rad_s2 = command.yaw_acc;
    web_status.command_pitch_acc_rad_s2 = command.pitch_acc;
    web_status.raw_target_yaw_deg = raw_desired_yaw * kRadToDeg;
    web_status.raw_target_pitch_deg = raw_desired_pitch * kRadToDeg;
    web_status.target_yaw_deg = desired_yaw * kRadToDeg;
    web_status.target_pitch_deg = desired_pitch * kRadToDeg;
    web_status.yaw_error_deg = command.yaw_diff * kRadToDeg;
    web_status.pitch_error_deg = command.pitch_diff * kRadToDeg;
    web_status.limiter_yaw_error_deg = fire_gate.yaw_error_rad * kRadToDeg;
    web_status.limiter_pitch_error_deg = fire_gate.pitch_error_rad * kRadToDeg;
    web_status.distance_m = command.distance;
    web_status.feedback_age_ms = options.dry_run ? 0.0 : static_cast<double>(current_feedback_age.count());
    web_status.feedback_alignment_used = used_aligned_feedback;
    web_status.fire_advice = sp_command.shoot;
    web_status.fire = command.fire_advice;
    web_status.fire_blocked_by_limiter = fire_gate.blocked;
    web_status.dry_run = options.dry_run;
    web_status.fire_enabled = options.enable_fire;
    web_status.enemy_color = options.enemy_color;
    web_status.camera_backend = options.camera_backend;
    web_status.serial_tx = hfut::io::infantryPacketLayoutName(serial_config.tx_layout);
    web_status.serial_rx = hfut::io::infantryPacketLayoutName(serial_config.rx_layout);

    const auto visual_start = std::chrono::steady_clock::now();
    if (options.display || web_server) {
      cv::Mat visual = frame.image.clone();
      drawArmors(visual, armors);
      drawCrosshair(visual);
      drawOverlay(visual, web_status);
      if (web_server && frames % static_cast<uint64_t>(options.web_frame_step) == 0U) {
        web_server->publish(visual, web_status);
      }
      if (options.display) {
        cv::imshow("hfut sp25 real", visual);
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q') break;
      }
    }
    const auto visual_end = std::chrono::steady_clock::now();

    if (visual_end - last_log > std::chrono::seconds(1)) {
      std::printf(
          "[standard] frames=%llu fps=%.1f armors=%zu tracked=%zu state=%s "
          "fb=%.2f/%.2fdeg fb_align=%.2f/%.2fdeg fb_delta=%.2f/%.2fdeg align_age=%.1fms "
          "raw=%.2f/%.2fdeg stable=%.2f/%.2fdeg cmd=%.2f/%.2fdeg "
          "cmd_vel=%.1f/%.1fdeg/s cmd_acc=%.1f/%.1fdeg/s2 lim_err=%.2f/%.2fdeg distance=%.3f "
          "sp_fire=%d fire=%d gate=%d latency=%.1fms "
          "timing=rx %.1f cam %.1f det %.1f trk %.1f aim %.1f tx %.1f vis %.1f loop %.1fms send_ok=%d\n",
          static_cast<unsigned long long>(frames), runtime_fps, armors.size(), targets.size(),
          tracker.state().c_str(), latest_feedback.yaw_rad * kRadToDeg,
          latest_feedback.pitch_rad * kRadToDeg, aligned_feedback.yaw_rad * kRadToDeg,
          aligned_feedback.pitch_rad * kRadToDeg, aligned_delta_yaw * kRadToDeg,
          aligned_delta_pitch * kRadToDeg, aligned_feedback_age_ms, raw_desired_yaw * kRadToDeg,
          raw_desired_pitch * kRadToDeg, desired_yaw * kRadToDeg, desired_pitch * kRadToDeg,
          command.yaw * kRadToDeg, command.pitch * kRadToDeg,
          command.yaw_vel * kRadToDeg, command.pitch_vel * kRadToDeg,
          command.yaw_acc * kRadToDeg, command.pitch_acc * kRadToDeg,
          fire_gate.yaw_error_rad * kRadToDeg, fire_gate.pitch_error_rad * kRadToDeg,
          command.distance, sp_command.shoot ? 1 : 0, command.fire_advice ? 1 : 0,
          fire_gate.blocked ? 1 : 0, elapsedMs(detect_start, aim_end),
          elapsedMs(serial_rx_start, serial_rx_end), elapsedMs(capture_start, capture_end),
          elapsedMs(detect_start, detect_end), elapsedMs(track_start, track_end),
          elapsedMs(aim_start, aim_end), elapsedMs(serial_tx_start, serial_tx_end),
          elapsedMs(visual_start, visual_end), elapsedMs(loop_start, visual_end),
          serial_send_ok ? 1 : 0);
      std::fflush(stdout);
      last_log = visual_end;
    }
  }

  if (options.display) cv::destroyAllWindows();
  return processed > 0 ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif
  try {
    return run(parseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[standard] fatal: %s\n", error.what());
    return 1;
  }
}
