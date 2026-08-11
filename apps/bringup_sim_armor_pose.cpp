// bringup_sim_armor_pose: detector-free ros-free entry for armor-pose simulator
// tests. It consumes armor_pose_frame.bin, runs tracker/selector/controller,
// applies the optional RL residual, and writes gimbal_command.bin.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "hfut_auto_aim/camera_frame.hpp"
#include "hfut_auto_aim/gimbal_command.hpp"
#include "io/bridge_protocol.hpp"
#include "io/camera/webots_bridge_camera.hpp"
#include "io/gimbal/webots_bridge_gimbal.hpp"

#include "pipeline.hpp"

#include <rm_interfaces/msg/armors.hpp>

namespace {
std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop.store(true); }

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

struct CliOptions {
  std::string config_dir{"configs"};
  std::string bridge_dir;
  std::string controller_strategy_override;
  bool diagnostics_requested{false};
  std::string diagnostics_path;
  bool rl_action_enabled{false};
  std::string rl_action_path;
  std::optional<double> rl_max_yaw_override;
  std::optional<double> rl_max_pitch_override;
};

struct RlActionConfig {
  bool enabled{false};
  std::string path;
  std::string configured_file{"rl_action.json"};
  double max_delta_yaw_rad{2.0 * kDegToRad};
  double max_delta_pitch_rad{2.0 * kDegToRad};
};

struct RlActionSample {
  bool present{false};
  bool valid{false};
  uint64_t seq{0};
  double delta_yaw_rad{0.0};
  double delta_pitch_rad{0.0};
  bool fire_gate{true};
  std::string error;
};

std::string escapeJsonString(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += ch; break;
    }
  }
  return escaped;
}

std::string resolveBridgeDir(const std::string& bridge_dir) {
  if (!bridge_dir.empty()) return bridge_dir;
  const char* env = std::getenv("WEBOTS_ROS_FREE_BRIDGE_DIR");
  if (env != nullptr && env[0] != '\0') return env;
  return hfut::bridge::kDefaultBridgeDir;
}

bool isAbsolutePath(const std::string& path) {
  if (path.empty()) return false;
  if (path.front() == '/') return true;
  return path.size() > 2 && std::isalpha(static_cast<unsigned char>(path[0])) &&
         path[1] == ':';
}

std::string joinPath(std::string dir, const std::string& file) {
  if (dir.empty()) return file;
  while (!dir.empty() && (dir.back() == '/' || dir.back() == '\\')) dir.pop_back();
  return dir + "/" + file;
}

std::string resolveRlActionPath(
    const std::string& bridge_dir,
    const RlActionConfig& config) {
  const std::string file = config.path.empty() ? config.configured_file : config.path;
  if (isAbsolutePath(file)) return file;
  return joinPath(resolveBridgeDir(bridge_dir), file);
}

void ensureParentDirectory(const std::string& path) {
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

Eigen::Vector3d readVector3(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(name + " must be a three-element sequence");
  }
  Eigen::Vector3d value(
      node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!value.allFinite()) {
    throw std::invalid_argument(name + " values must be finite");
  }
  return value;
}

double finiteOrDefault(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}

double clampAbs(double value, double max_abs) {
  if (!std::isfinite(value)) return 0.0;
  return std::clamp(value, -std::abs(max_abs), std::abs(max_abs));
}

bool readFireGate(const YAML::Node& node) {
  if (!node) return true;
  try {
    return node.as<bool>();
  } catch (const YAML::Exception&) {
  }
  try {
    return node.as<double>() > 0.5;
  } catch (const YAML::Exception&) {
  }
  return true;
}

const char* simArmorName(int index) {
  switch (index) {
    case 0: return "front";
    case 1: return "left";
    case 2: return "rear";
    case 3: return "right";
    default: return "unknown";
  }
}

const char* simArmorLayer(int index) {
  if (index < 0) return "unknown";
  return (index % 2 == 0) ? "lower" : "upper";
}

void writeVector3Json(std::ostream& out, const Eigen::Vector3d& value) {
  out << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

void writeArmorPositionsJson(std::ostream& out, const std::vector<Eigen::Vector3d>& positions) {
  out << '[';
  for (size_t index = 0; index < positions.size(); ++index) {
    if (index > 0) out << ',';
    out << "{\"index\":" << index
        << ",\"name\":\"" << simArmorName(static_cast<int>(index)) << "\""
        << ",\"layer\":\"" << simArmorLayer(static_cast<int>(index)) << "\""
        << ",\"position\":";
    writeVector3Json(out, positions[index]);
    out << '}';
  }
  out << ']';
}

RlActionSample readRlActionFile(const RlActionConfig& config) {
  RlActionSample sample;
  std::ifstream probe(config.path);
  if (!probe.good()) return sample;
  sample.present = true;
  probe.close();
  try {
    const YAML::Node root = YAML::LoadFile(config.path);
    sample.seq = root["seq"] ? root["seq"].as<uint64_t>() : 0ULL;
    sample.delta_yaw_rad = clampAbs(
        finiteOrDefault(root["delta_yaw_rad"] ? root["delta_yaw_rad"].as<double>() : 0.0, 0.0),
        config.max_delta_yaw_rad);
    sample.delta_pitch_rad = clampAbs(
        finiteOrDefault(root["delta_pitch_rad"] ? root["delta_pitch_rad"].as<double>() : 0.0, 0.0),
        config.max_delta_pitch_rad);
    sample.fire_gate = readFireGate(root["fire_gate"]);
    sample.valid = true;
  } catch (const std::exception& error) {
    sample.error = error.what();
  }
  return sample;
}

rm_interfaces::msg::Armors buildDirectArmors(const hfut::CameraFrame& frame) {
  rm_interfaces::msg::Armors msg;
  msg.armors.reserve(frame.direct_armors.size());

  for (const auto& measurement : frame.direct_armors) {
    const double position_variance =
        measurement.position_noise_std_m * measurement.position_noise_std_m;
    const double yaw_variance =
        measurement.yaw_noise_std_rad * measurement.yaw_noise_std_rad;
    rm_interfaces::msg::Armor armor;
    armor.number = measurement.number;
    armor.type = measurement.type;
    armor.detection_confidence = static_cast<float>(measurement.confidence);
    armor.pose.position.x = measurement.position_control.x();
    armor.pose.position.y = measurement.position_control.y();
    armor.pose.position.z = measurement.position_control.z();
    armor.radial_yaw_valid = true;
    armor.radial_yaw = measurement.radial_yaw;

    if (measurement.surface_orientation_valid) {
      const auto q = measurement.surface_orientation_control.normalized();
      armor.pose.orientation.x = q.x();
      armor.pose.orientation.y = q.y();
      armor.pose.orientation.z = q.z();
      armor.pose.orientation.w = q.w();
      armor.pose_orientation_trusted = true;
    } else {
      const double normal_yaw = measurement.radial_yaw - M_PI;
      armor.pose.orientation.z = std::sin(normal_yaw * 0.5);
      armor.pose.orientation.w = std::cos(normal_yaw * 0.5);
    }
    armor.pose_estimate_mode = 0;
    armor.pose_quality_score = static_cast<float>(measurement.confidence);
    armor.pose_num_points = 1;
    armor.pose_num_inliers = 1;
    armor.pose_covariance_valid = position_variance > 0.0 || yaw_variance > 0.0;
    if (armor.pose_covariance_valid) {
      armor.pose_covariance_xyz_yaw.fill(0.0);
      armor.pose_covariance_xyz_yaw[0] = position_variance;
      armor.pose_covariance_xyz_yaw[5] = position_variance;
      armor.pose_covariance_xyz_yaw[10] = position_variance;
      armor.pose_covariance_xyz_yaw[15] = yaw_variance;
    }
    msg.armors.push_back(std::move(armor));
  }
  return msg;
}

CliOptions parseArgs(int argc, char** argv) {
  CliOptions options;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::printf(
          "usage: bringup_sim_armor_pose [--config-dir=DIR] [--bridge-dir=DIR] "
          "[--strategy=NAME] [--diagnostics[=PATH]] [--rl-action[=PATH]]\n");
      std::exit(0);
    } else if (arg.rfind("--config-dir=", 0) == 0) {
      options.config_dir = arg.substr(std::string("--config-dir=").size());
    } else if (arg.rfind("--bridge-dir=", 0) == 0) {
      options.bridge_dir = arg.substr(std::string("--bridge-dir=").size());
    } else if (arg.rfind("--strategy=", 0) == 0) {
      options.controller_strategy_override = arg.substr(std::string("--strategy=").size());
    } else if (arg == "--diagnostics") {
      options.diagnostics_requested = true;
    } else if (arg.rfind("--diagnostics=", 0) == 0) {
      options.diagnostics_requested = true;
      options.diagnostics_path = arg.substr(std::string("--diagnostics=").size());
    } else if (arg == "--rl-action") {
      options.rl_action_enabled = true;
    } else if (arg.rfind("--rl-action=", 0) == 0) {
      options.rl_action_enabled = true;
      options.rl_action_path = arg.substr(std::string("--rl-action=").size());
    } else if (arg.rfind("--rl-action-max-yaw=", 0) == 0) {
      options.rl_max_yaw_override =
          std::stod(arg.substr(std::string("--rl-action-max-yaw=").size()));
    } else if (arg.rfind("--rl-action-max-pitch=", 0) == 0) {
      options.rl_max_pitch_override =
          std::stod(arg.substr(std::string("--rl-action-max-pitch=").size()));
    } else if (arg == "--armor-pose" || arg == "--input-mode=armor_pose") {
      // Accepted for parity with bringup_sim; this binary is always armor_pose.
    } else {
      std::fprintf(stderr, "[bringup_sim_armor_pose] unknown argument: %s\n", arg.c_str());
      std::exit(2);
    }
  }
  return options;
}

void loadRlConfig(const std::string& rl_cfg, RlActionConfig& config) {
  std::ifstream probe(rl_cfg);
  if (!probe.good()) return;
  probe.close();
  const YAML::Node rl_root = YAML::LoadFile(rl_cfg);
  if (rl_root["action_file"] && config.path.empty()) {
    config.configured_file = rl_root["action_file"].as<std::string>();
  }
  const YAML::Node action = rl_root["action"];
  if (action && action["max_delta_yaw_rad"]) {
    config.max_delta_yaw_rad = action["max_delta_yaw_rad"].as<double>();
  }
  if (action && action["max_delta_pitch_rad"]) {
    config.max_delta_pitch_rad = action["max_delta_pitch_rad"].as<double>();
  }
}

void writeDiagnostics(
    std::ofstream& diagnostics,
    const hfut::CameraFrame& frame,
    const rm_interfaces::msg::Armors& armors,
    const hfut::GimbalCommand& out,
    const RlActionConfig& rl_config,
    const RlActionSample& rl_action,
    const hfut::pipeline::Pipeline::DebugSnapshot& dbg) {
  diagnostics << "{\"seq\":" << frame.seq
              << ",\"sim_time_s\":" << frame.sim_time_s
              << ",\"input_mode\":\"armor_pose\""
              << ",\"bridge_path\":\"webots\""
              << ",\"command_mode\":" << static_cast<int>(out.mode)
              << ",\"rl_action\":{\"enabled\":"
              << (rl_config.enabled ? "true" : "false")
              << ",\"present\":" << (rl_action.present ? "true" : "false")
              << ",\"valid\":" << (rl_action.valid ? "true" : "false")
              << ",\"seq\":" << rl_action.seq
              << ",\"delta_yaw_rad\":" << rl_action.delta_yaw_rad
              << ",\"delta_pitch_rad\":" << rl_action.delta_pitch_rad
              << ",\"fire_gate\":" << (rl_action.fire_gate ? 1 : 0)
              << ",\"error\":\"" << escapeJsonString(rl_action.error) << "\"}"
              << ",\"tracked_count\":" << dbg.num_tracked
              << ",\"selected_id\":\"" << escapeJsonString(dbg.selected_id) << "\""
              << ",\"track_state\":" << dbg.selected_track_state
              << ",\"direct_armors\":[";
  for (size_t index = 0; index < frame.direct_armors.size(); ++index) {
    if (index > 0) diagnostics << ',';
    const auto& direct = frame.direct_armors[index];
    diagnostics << "{\"index\":" << index
                << ",\"semantic_name\":\"" << simArmorName(static_cast<int>(index)) << "\""
                << ",\"layer\":\"" << simArmorLayer(static_cast<int>(index)) << "\""
                << ",\"number\":\"" << escapeJsonString(direct.number)
                << "\",\"type\":\"" << escapeJsonString(direct.type)
                << "\",\"confidence\":" << direct.confidence
                << ",\"position\":[" << direct.position_control.x() << ','
                << direct.position_control.y() << ',' << direct.position_control.z()
                << "],\"radial_yaw\":" << direct.radial_yaw
                << ",\"position_noise_std_m\":" << direct.position_noise_std_m
                << ",\"yaw_noise_std_rad\":" << direct.yaw_noise_std_rad
                << ",\"view_angle_rad\":" << direct.view_angle_rad
                << ",\"surface_orientation_valid\":"
                << (direct.surface_orientation_valid ? "true" : "false")
                << "}";
  }
  diagnostics << ']'
              << ",\"command\":{\"yaw_deg\":" << out.yaw * kRadToDeg
              << ",\"pitch_deg\":" << out.pitch * kRadToDeg
              << ",\"yaw_diff_deg\":" << out.yaw_diff * kRadToDeg
              << ",\"pitch_diff_deg\":" << out.pitch_diff * kRadToDeg
              << ",\"yaw_velocity_dps\":" << out.yaw_vel * kRadToDeg
              << ",\"pitch_velocity_dps\":" << out.pitch_vel * kRadToDeg
              << ",\"yaw_acceleration_dps2\":" << out.yaw_acc * kRadToDeg
              << ",\"pitch_acceleration_dps2\":" << out.pitch_acc * kRadToDeg
              << ",\"distance_m\":" << out.distance
              << ",\"fire_advice\":" << (out.fire_advice ? 1 : 0)
              << "}"
              << ",\"control_target\":{\"valid\":"
              << (dbg.control_target.valid ? "true" : "false")
              << ",\"tracks_center\":"
              << (dbg.control_target.tracks_center ? "true" : "false")
              << ",\"virtual_target\":"
              << (dbg.control_target.is_virtual_target ? "true" : "false")
              << ",\"selected_index\":" << dbg.control_target.selected_index
              << ",\"real_selected_index\":" << dbg.control_target.real_selected_index
              << ",\"selected_name\":\""
              << simArmorName(dbg.control_target.real_selected_index >= 0
                   ? dbg.control_target.real_selected_index : dbg.control_target.selected_index)
              << "\""
              << ",\"selected_layer\":\""
              << simArmorLayer(dbg.control_target.real_selected_index >= 0
                   ? dbg.control_target.real_selected_index : dbg.control_target.selected_index)
              << "\""
              << ",\"prediction_time_s\":" << dbg.control_target.prediction_time_s
              << ",\"yaw_velocity_rad_s\":" << dbg.control_target.yaw_velocity
              << ",\"current_center\":";
  writeVector3Json(diagnostics, dbg.control_target.current_center);
  diagnostics << ",\"predicted_center\":";
  writeVector3Json(diagnostics, dbg.control_target.predicted_center);
  diagnostics << ",\"current_selected_armor\":";
  writeVector3Json(diagnostics, dbg.control_target.current_selected_armor);
  diagnostics << ",\"control_target_position\":";
  writeVector3Json(diagnostics, dbg.control_target.control_target_position);
  diagnostics << ",\"current_armor_positions\":";
  writeArmorPositionsJson(diagnostics, dbg.control_target.current_armor_positions);
  diagnostics << ",\"predicted_armor_positions\":";
  writeArmorPositionsJson(diagnostics, dbg.control_target.predicted_armor_positions);
  diagnostics << "}"
              << ",\"delay\":{\"prediction_s\":"
              << dbg.delay_audit.total_prediction_time_s
              << ",\"flight_time_s\":" << dbg.delay_audit.flight_time_s
              << ",\"processing_delay_s\":" << dbg.delay_audit.processing_delay_s
              << ",\"control_latency_s\":" << dbg.delay_audit.control_latency_s
              << ",\"strategy\":\"" << escapeJsonString(dbg.delay_audit.strategy_name)
              << "\"}"
              << ",\"state_estimate\":{\"valid\":"
              << (dbg.selected_state_valid ? "true" : "false");
  if (dbg.selected_state_valid) {
    const auto& state = dbg.selected_state;
    diagnostics << ",\"position\":[" << state.center_position.x << ','
                << state.center_position.y << ',' << state.center_position.z << ']'
                << ",\"velocity\":[" << state.center_velocity.x << ','
                << state.center_velocity.y << ',' << state.center_velocity.z << ']'
                << ",\"yaw\":" << state.yaw
                << ",\"yaw_velocity\":" << state.yaw_velocity;
  }
  diagnostics << "}"
              << ",\"input_armor_count\":" << armors.armors.size()
              << "}\n";
}
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif

  const CliOptions cli = parseArgs(argc, argv);
  const std::string master_cfg = cli.config_dir + "/gimbal_pipeline.yaml";
  const std::string simulation_cfg = cli.config_dir + "/simulation.yaml";
  const std::string tracker_cfg = cli.config_dir + "/tracker.yaml";
  const std::string controller_cfg = cli.config_dir + "/controller.yaml";
  const std::string rl_cfg = cli.config_dir + "/rl_sim.yaml";

  std::string bridge_dir = cli.bridge_dir;
  hfut::CameraToBarrelExtrinsics camera_to_barrel;
  double bullet_speed = 22.5;
  double observation_noise_scale = 0.35;
  double max_temp_lost_prediction_s = 0.15;
  double temp_lost_coast_max_s = 0.5;
  double temp_lost_coast_min_speed_mps = 0.3;
  double id_association_max_distance_m = 0.6;
  double attitude_mount_pitch_deg = 15.0;
  double attitude_ema_alpha = 0.1;
  bool attitude_apply_to_geometry = false;
  fyt::auto_aim::TrackerMotionGuardParameters motion_guard;
  RlActionConfig rl_action_config;
  rl_action_config.enabled = cli.rl_action_enabled;
  rl_action_config.path = cli.rl_action_path;

  try {
    const YAML::Node sim_root = YAML::LoadFile(simulation_cfg);
    const YAML::Node tracker_root = YAML::LoadFile(tracker_cfg);
    loadRlConfig(rl_cfg, rl_action_config);
    if (cli.rl_max_yaw_override) {
      rl_action_config.max_delta_yaw_rad = *cli.rl_max_yaw_override;
    }
    if (cli.rl_max_pitch_override) {
      rl_action_config.max_delta_pitch_rad = *cli.rl_max_pitch_override;
    }

    const YAML::Node tracking = tracker_root["tracking"];
    if (tracking && tracking["observation_noise_scale"]) {
      const YAML::Node& noise_scale = tracking["observation_noise_scale"];
      observation_noise_scale = noise_scale.IsMap()
          ? noise_scale["armor_pose"].as<double>()
          : noise_scale.as<double>();
    }
    if (tracking && tracking["max_temp_lost_prediction_s"]) {
      max_temp_lost_prediction_s = tracking["max_temp_lost_prediction_s"].as<double>();
    }
    if (tracking && tracking["temp_lost_coast_max_s"]) {
      temp_lost_coast_max_s = tracking["temp_lost_coast_max_s"].as<double>();
    }
    if (tracking && tracking["temp_lost_coast_min_speed_mps"]) {
      temp_lost_coast_min_speed_mps = tracking["temp_lost_coast_min_speed_mps"].as<double>();
    }
    if (tracking && tracking["id_association_max_distance_m"]) {
      id_association_max_distance_m = tracking["id_association_max_distance_m"].as<double>();
    }
    const YAML::Node attitude = tracking ? tracking["attitude"] : YAML::Node{};
    if (attitude) {
      if (attitude["mount_pitch_deg"]) {
        attitude_mount_pitch_deg = attitude["mount_pitch_deg"].as<double>();
      }
      if (attitude["ema_alpha"]) {
        attitude_ema_alpha = attitude["ema_alpha"].as<double>();
      }
      if (attitude["apply_to_geometry"]) {
        attitude_apply_to_geometry = attitude["apply_to_geometry"].as<bool>();
      }
    }
    const YAML::Node motion_guard_node = tracking ? tracking["motion_guard"] : YAML::Node{};
    if (motion_guard_node) {
      if (motion_guard_node["enabled"]) {
        motion_guard.enabled = motion_guard_node["enabled"].as<bool>();
      }
      if (motion_guard_node["stationary_speed_deadband_mps"]) {
        motion_guard.stationary_speed_deadband_mps =
            motion_guard_node["stationary_speed_deadband_mps"].as<double>();
      }
      if (motion_guard_node["max_linear_speed_mps"]) {
        motion_guard.max_linear_speed_mps =
            motion_guard_node["max_linear_speed_mps"].as<double>();
      }
      if (motion_guard_node["max_linear_acceleration_mps2"]) {
        motion_guard.max_linear_acceleration_mps2 =
            motion_guard_node["max_linear_acceleration_mps2"].as<double>();
      }
      if (motion_guard_node["temp_lost_velocity_half_life_s"]) {
        motion_guard.temp_lost_velocity_half_life_s =
            motion_guard_node["temp_lost_velocity_half_life_s"].as<double>();
      }
      if (motion_guard_node["velocity_reset_std_mps"]) {
        motion_guard.velocity_reset_std_mps =
            motion_guard_node["velocity_reset_std_mps"].as<double>();
      }
      if (motion_guard_node["acceleration_reset_std_mps2"]) {
        motion_guard.acceleration_reset_std_mps2 =
            motion_guard_node["acceleration_reset_std_mps2"].as<double>();
      }
      if (motion_guard_node["max_yaw_rate_rad_s"]) {
        motion_guard.max_yaw_rate_rad_s =
            motion_guard_node["max_yaw_rate_rad_s"].as<double>();
      }
      if (motion_guard_node["yaw_rate_deadband_rad_s"]) {
        motion_guard.yaw_rate_deadband_rad_s =
            motion_guard_node["yaw_rate_deadband_rad_s"].as<double>();
      }
      if (motion_guard_node["yaw_rate_reset_std_rad_s"]) {
        motion_guard.yaw_rate_reset_std_rad_s =
            motion_guard_node["yaw_rate_reset_std_rad_s"].as<double>();
      }
    }

    const YAML::Node camera_root = sim_root["camera_to_barrel"];
    const YAML::Node camera_extrinsics =
        camera_root && camera_root["webots"] ? camera_root["webots"] : camera_root;
    if (camera_extrinsics && camera_extrinsics["xyz"]) {
      camera_to_barrel.xyz = readVector3(camera_extrinsics["xyz"], "camera_to_barrel.webots.xyz");
    }
    if (camera_extrinsics && camera_extrinsics["rpy"]) {
      camera_to_barrel.rpy = readVector3(camera_extrinsics["rpy"], "camera_to_barrel.webots.rpy");
    }

    const YAML::Node controller = sim_root["controller"];
    const YAML::Node bullet_speed_node = controller ? controller["bullet_speed"] : YAML::Node{};
    if (bullet_speed_node) {
      bullet_speed = bullet_speed_node.IsMap()
          ? bullet_speed_node["webots"].as<double>()
          : bullet_speed_node.as<double>();
    }

    const YAML::Node bridge = sim_root["bridge"];
    if (bridge_dir.empty()) {
      bridge_dir = bridge && bridge["dir"] ? bridge["dir"].as<std::string>() : "";
    }
    if (rl_action_config.enabled) {
      rl_action_config.path = resolveRlActionPath(bridge_dir, rl_action_config);
    }

    if (!std::isfinite(observation_noise_scale) || observation_noise_scale <= 0.0) {
      throw std::invalid_argument("tracking.observation_noise_scale.armor_pose must be finite and > 0");
    }
    if (!std::isfinite(bullet_speed) || bullet_speed <= 0.0) {
      throw std::invalid_argument("controller.bullet_speed.webots must be finite and > 0");
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[bringup_sim_armor_pose] invalid config in %s: %s\n",
                 cli.config_dir.c_str(), error.what());
    return 1;
  }

  const std::string active_bridge_dir = resolveBridgeDir(bridge_dir);
  std::string diagnostics_path = cli.diagnostics_path;
  if (cli.diagnostics_requested && diagnostics_path.empty()) {
    diagnostics_path = joinPath(active_bridge_dir, "tracking_diagnostics.jsonl");
  }

  hfut::pipeline::PipelineOverrides pipeline_overrides;
  pipeline_overrides.bullet_speed = bullet_speed;
  pipeline_overrides.controller_strategy = cli.controller_strategy_override;
  pipeline_overrides.observation_noise_scale = observation_noise_scale;
  pipeline_overrides.max_temp_lost_prediction_s = max_temp_lost_prediction_s;
  pipeline_overrides.temp_lost_coast_max_s = temp_lost_coast_max_s;
  pipeline_overrides.temp_lost_coast_min_speed_mps = temp_lost_coast_min_speed_mps;
  pipeline_overrides.attitude_mount_pitch_deg = attitude_mount_pitch_deg;
  pipeline_overrides.attitude_ema_alpha = attitude_ema_alpha;
  pipeline_overrides.attitude_apply_to_geometry = attitude_apply_to_geometry;
  pipeline_overrides.id_association_max_distance_m = id_association_max_distance_m;
  pipeline_overrides.motion_guard = motion_guard;

  hfut::pipeline::Pipeline pipeline(
      {tracker_cfg, controller_cfg, master_cfg},
      "gimbal_pipeline", pipeline_overrides);
  hfut::io::WebotsBridgeCamera camera(bridge_dir, hfut::io::BridgeInputMode::armor_pose);
  hfut::io::WebotsBridgeGimbal gimbal(bridge_dir);

  std::ofstream diagnostics;
  if (!diagnostics_path.empty()) {
    try {
      ensureParentDirectory(diagnostics_path);
    } catch (const std::exception& error) {
      std::fprintf(stderr, "[bringup_sim_armor_pose] failed to create diagnostics dir: %s\n",
                   error.what());
      return 1;
    }
    diagnostics.open(diagnostics_path, std::ios::out | std::ios::trunc);
    if (!diagnostics) {
      std::fprintf(stderr, "[bringup_sim_armor_pose] failed to open diagnostics: %s\n",
                   diagnostics_path.c_str());
      return 1;
    }
  }

  std::printf("[bringup_sim_armor_pose] bridge dir: %s\n", active_bridge_dir.c_str());
  std::printf("[bringup_sim_armor_pose] input mode: armor_pose (detector disabled)\n");
  std::printf(
      "[bringup_sim_armor_pose] camera_to_barrel: xyz=[%.4f,%.4f,%.4f]m "
      "rpy=[%.4f,%.4f,%.4f]rad\n",
      camera_to_barrel.xyz.x(), camera_to_barrel.xyz.y(), camera_to_barrel.xyz.z(),
      camera_to_barrel.rpy.x(), camera_to_barrel.rpy.y(), camera_to_barrel.rpy.z());
  std::printf(
      "[bringup_sim_armor_pose] ballistics/tracking: bullet_speed=%.2fm/s "
      "observation_noise_scale=%.3f motion_guard=%s\n",
      bullet_speed, observation_noise_scale, motion_guard.enabled ? "enabled" : "disabled");
  if (!diagnostics_path.empty()) {
    std::printf("[bringup_sim_armor_pose] diagnostics: %s\n", diagnostics_path.c_str());
  }
  if (rl_action_config.enabled) {
    std::printf(
        "[bringup_sim_armor_pose] rl residual: action=%s limit(yaw=%.3fdeg pitch=%.3fdeg)\n",
        rl_action_config.path.c_str(),
        rl_action_config.max_delta_yaw_rad * kRadToDeg,
        rl_action_config.max_delta_pitch_rad * kRadToDeg);
  }
  std::printf("[bringup_sim_armor_pose] ready, waiting for armor_pose_frame.bin...\n");
  std::fflush(stdout);

  hfut::CameraFrame frame;
  uint64_t frames = 0;
  double previous_source_time_s = 0.0;
  double smoothed_source_fps = 0.0;
  double smoothed_loop_fps = 0.0;
  auto last_log = std::chrono::steady_clock::now();
  auto last_frame_done = std::chrono::steady_clock::now();

  while (!g_stop.load()) {
    if (!camera.read(frame, std::chrono::milliseconds(2000))) {
      std::printf("[bringup_sim_armor_pose] no armor-pose frame for 2s\n");
      std::fflush(stdout);
      continue;
    }
    if (!hfut::applyCameraToBarrelExtrinsics(frame, camera_to_barrel)) {
      std::fprintf(stderr, "[bringup_sim_armor_pose] invalid camera pose in frame %llu\n",
                   static_cast<unsigned long long>(frame.seq));
      continue;
    }

    ++frames;
    if (previous_source_time_s > 0.0) {
      const double source_interval = frame.sim_time_s - previous_source_time_s;
      if (source_interval > 1e-6 && source_interval < 1.0) {
        const double source_fps = 1.0 / source_interval;
        smoothed_source_fps = smoothed_source_fps <= 0.0
            ? source_fps
            : 0.9 * smoothed_source_fps + 0.1 * source_fps;
      }
    }
    previous_source_time_s = frame.sim_time_s;
    const auto processing_started = std::chrono::steady_clock::now();

    const auto armors = buildDirectArmors(frame);
    pipeline.updateTrackingControlFrame(armors, frame.sim_time_s);

    const double processing_elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - processing_started).count();
    auto cmd_deg = pipeline.computeCommand(
        frame.gimbal_yaw, frame.gimbal_pitch,
        frame.sim_time_s + processing_elapsed_s);

    hfut::GimbalCommand out;
    out.yaw = cmd_deg.yaw * kDegToRad;
    out.pitch = cmd_deg.pitch * kDegToRad;
    out.yaw_diff = cmd_deg.yaw_diff * kDegToRad;
    out.pitch_diff = cmd_deg.pitch_diff * kDegToRad;
    out.yaw_vel = cmd_deg.yaw_v * kDegToRad;
    out.pitch_vel = cmd_deg.pitch_v * kDegToRad;
    out.yaw_acc = cmd_deg.yaw_a * kDegToRad;
    out.pitch_acc = cmd_deg.pitch_a * kDegToRad;
    out.distance = cmd_deg.distance;
    out.fire_advice = cmd_deg.fire_advice;
    out.target_id = cmd_deg.target_id;
    out.mode = static_cast<hfut::GimbalMode>(cmd_deg.mode);

    RlActionSample rl_action;
    if (rl_action_config.enabled) {
      rl_action = readRlActionFile(rl_action_config);
      if (rl_action.valid) {
        out.yaw += rl_action.delta_yaw_rad;
        out.pitch += rl_action.delta_pitch_rad;
        out.yaw_diff += rl_action.delta_yaw_rad;
        out.pitch_diff += rl_action.delta_pitch_rad;
        out.fire_advice = out.fire_advice && rl_action.fire_gate;
      }
    }

    if (!gimbal.send(out, frame.sim_time_s)) {
      std::fprintf(stderr, "[bringup_sim_armor_pose] failed to write gimbal_command.bin\n");
      continue;
    }

    const auto& dbg = pipeline.lastDebug();
    if (diagnostics) {
      writeDiagnostics(diagnostics, frame, armors, out, rl_action_config, rl_action, dbg);
      if ((frames & 0x0fU) == 0) diagnostics.flush();
    }

    const auto frame_done = std::chrono::steady_clock::now();
    const double frame_interval = std::chrono::duration<double>(
        frame_done - last_frame_done).count();
    if (frame_interval > 1e-6) {
      const double instantaneous_fps = 1.0 / frame_interval;
      smoothed_loop_fps = smoothed_loop_fps <= 0.0
          ? instantaneous_fps
          : 0.9 * smoothed_loop_fps + 0.1 * instantaneous_fps;
    }
    last_frame_done = frame_done;
    const double loop_latency_ms = std::chrono::duration<double, std::milli>(
        frame_done - processing_started).count();

    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::milliseconds(500)) {
      std::printf(
          "[bringup_sim_armor_pose] seq=%llu t=%.2f armors=%zu "
          "cmd(yaw=%.2f pitch=%.2f deg) fire=%d mode=%d | tracked=%d "
          "sel=%s state=%d | source=%.1fHz loop=%.1fHz process=%.1fms\n",
          static_cast<unsigned long long>(frame.seq), frame.sim_time_s,
          armors.armors.size(), out.yaw * kRadToDeg, out.pitch * kRadToDeg,
          out.fire_advice ? 1 : 0, static_cast<int>(out.mode), dbg.num_tracked,
          dbg.selected_id.c_str(), dbg.selected_track_state,
          smoothed_source_fps, smoothed_loop_fps, loop_latency_ms);
      std::fflush(stdout);
      last_log = now;
    }
  }

  return 0;
}
