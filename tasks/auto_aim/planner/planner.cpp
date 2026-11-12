#include "planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
namespace
{
YAML::Node controller_node(const YAML::Node & yaml)
{
  const auto nested = yaml["gimbal_pipeline"]["ros__parameters"]["controller"];
  if (nested) return nested;
  const auto direct = yaml["controller"];
  if (direct) return direct;
  return yaml;
}

YAML::Node planner_node(const YAML::Node & yaml, const std::string & profile)
{
  return controller_node(yaml)[profile];
}

template <typename T>
T read_planner_value(
  const YAML::Node & primary, const YAML::Node & fallback, const std::string & key)
{
  if (primary && primary[key]) return primary[key].as<T>();
  return tools::read<T>(fallback, key);
}
}  // namespace

Planner::Planner(const std::string & config_path, const std::string & planner_profile)
: planner_profile_(planner_profile)
{
  const auto root = tools::load(config_path);
  const auto yaml = planner_node(root, planner_profile_);
  const auto fallback = planner_node(root, "mpc_planner");
  yaw_offset_ = read_planner_value<double>(yaml, fallback, "yaw_offset") / 57.3;
  pitch_offset_ = read_planner_value<double>(yaml, fallback, "pitch_offset") / 57.3;
  fire_thresh_ = read_planner_value<double>(yaml, fallback, "fire_thresh");
  decision_speed_ = read_planner_value<double>(yaml, fallback, "decision_speed");
  high_speed_delay_time_ = read_planner_value<double>(yaml, fallback, "high_speed_delay_time");
  low_speed_delay_time_ = read_planner_value<double>(yaml, fallback, "low_speed_delay_time");
  const auto spin_profile = yaml["ordinary_spin_profile"]
    ? yaml["ordinary_spin_profile"]
    : fallback["ordinary_spin_profile"];
  ordinary_spin_profile_ = SpinSpeedProfile::fromYaml(spin_profile);

  // 高速反小陀螺参数只作为普通目标的备用策略，前哨站在调用层直接排除。
  const auto anti_spin = yaml["anti_spin"] ? yaml["anti_spin"] : fallback["anti_spin"];
  const int enter_frames = anti_spin ? anti_spin["enter_frames"].as<int>(3) : 3;
  const int exit_frames = anti_spin ? anti_spin["exit_frames"].as<int>(5) : 5;
  if (anti_spin) {
    anti_spin_enabled_ = anti_spin["enable"].as<bool>(anti_spin_enabled_);
    anti_spin_exit_hysteresis_rad_s_ =
      anti_spin["exit_hysteresis_rad_s"].as<double>(anti_spin_exit_hysteresis_rad_s_);
    anti_spin_guard_stable_time_s_ =
      anti_spin["guard_stable_time_ms"].as<double>(anti_spin_guard_stable_time_s_ * 1000.0) /
      1000.0;
    anti_spin_front_face_tolerance_rad_ =
      anti_spin["front_face_tolerance_deg"].as<double>(
        anti_spin_front_face_tolerance_rad_ * 57.3) / 57.3;
    // 高速模式单独使用稍宽的反馈角门限，避免云台刚好落后时把所有候选拦掉。
    // 不配置时保持普通 fire_gate 的门限。
    if (anti_spin["fire_yaw_tolerance_deg"]) {
      const double tolerance_deg = anti_spin["fire_yaw_tolerance_deg"].as<double>();
      if (!std::isfinite(tolerance_deg) || tolerance_deg <= 0.0) {
        throw std::invalid_argument(
          "controller.mpc_planner.anti_spin.fire_yaw_tolerance_deg 必须 > 0");
      }
      anti_spin_fire_yaw_tolerance_rad_ = tolerance_deg / 57.3;
    }
    if (anti_spin["fire_pitch_tolerance_deg"]) {
      const double tolerance_deg = anti_spin["fire_pitch_tolerance_deg"].as<double>();
      if (!std::isfinite(tolerance_deg) || tolerance_deg <= 0.0) {
        throw std::invalid_argument(
          "controller.mpc_planner.anti_spin.fire_pitch_tolerance_deg 必须 > 0");
      }
      anti_spin_fire_pitch_tolerance_rad_ = tolerance_deg / 57.3;
    }
  }
  anti_spin_exit_hysteresis_rad_s_ = std::max(anti_spin_exit_hysteresis_rad_s_, 0.0);
  anti_spin_guard_stable_time_s_ = std::max(anti_spin_guard_stable_time_s_, 0.0);
  anti_spin_front_face_tolerance_rad_ = std::max(anti_spin_front_face_tolerance_rad_, 0.0);
  anti_spin_mode_selector_.configure(
    anti_spin_enabled_, decision_speed_, enter_frames, exit_frames,
    anti_spin_exit_hysteresis_rad_s_);

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(const Trajectory & trajectory, double yaw0, double distance)
{
  if (!trajectory.allFinite() || !std::isfinite(yaw0) || !std::isfinite(distance)) {
    tools::logger()->warn("[Planner] 打符参考轨迹无效");
    return {false};
  }

  // 打符跟踪器已经生成了完整参考轨迹，这里只负责复用通用云台 MPC。
  Eigen::VectorXd x0(2);
  x0 << trajectory(0, 0), trajectory(1, 0);
  if (tiny_set_x0(yaw_solver_, x0) != 0) return {false};
  yaw_solver_->work->Xref = trajectory.block(0, 0, 2, HORIZON);
  const int yaw_status = tiny_solve(yaw_solver_);

  x0 << trajectory(2, 0), trajectory(3, 0);
  if (tiny_set_x0(pitch_solver_, x0) != 0) return {false};
  pitch_solver_->work->Xref = trajectory.block(2, 0, 2, HORIZON);
  const int pitch_status = tiny_solve(pitch_solver_);
  if (yaw_status != 0 || pitch_status != 0) {
    if (!warned_solver_status_) {
      tools::logger()->warn(
        "[Planner] 打符 TinyMPC 未在最大迭代次数内收敛 yaw={} pitch={}", yaw_status,
        pitch_status);
      warned_solver_status_ = true;
    }
  }

  Plan result{};
  result.control = true;
  result.fire = false;
  result.target_yaw = tools::limit_rad(trajectory(0, HALF_HORIZON) + yaw0);
  result.target_pitch = trajectory(2, HALF_HORIZON);
  result.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  result.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  result.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
  result.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  result.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  result.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);
  result.distance = static_cast<float>(distance);

  if (
    !std::isfinite(result.target_yaw) || !std::isfinite(result.target_pitch) ||
    !std::isfinite(result.yaw) || !std::isfinite(result.yaw_vel) ||
    !std::isfinite(result.yaw_acc) || !std::isfinite(result.pitch) ||
    !std::isfinite(result.pitch_vel) || !std::isfinite(result.pitch_acc)) {
    tools::logger()->warn("[Planner] 打符 TinyMPC 输出非有限值");
    return {false};
  }
  return result;
}

void Planner::setArmorAvoidanceConfig(const ArmorAvoidanceConfig & config)
{
  if (sameArmorAvoidanceConfig(armor_avoidance_config_, config)) return;
  armor_avoidance_config_ = config;
  armor_avoidance_result_ = optimizeArmorAvoidance(config);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. Check bullet speed
  if (!std::isfinite(bullet_speed) || bullet_speed < 14) {
    bullet_speed = 23;
  }

  // 先判断是否进入高速反小陀螺模式。前哨站永远走原来的固定模型。
  if (updateAntiSpinState(target)) return planAntiSpin(target, bullet_speed);

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  double selected_yaw = 0.0;
  const auto armor_xyza_list = target.armor_xyza_list();
  if (armor_xyza_list.empty()) return {false};
  for (auto & xyza : armor_xyza_list) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      selected_yaw = xyza[3];
    }
  }
  if (armor_avoidance_config_.enabled) {
    if (!armor_avoidance_result_.valid) return {false};
    applyArmorAvoidanceOffset(
      armor_avoidance_result_, selected_yaw, xyz.x(), xyz.y(), xyz.z());
    min_dist = xyz.head<2>().norm();
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) return {false};
  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  if (tiny_set_x0(yaw_solver_, x0) != 0) return {false};

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  const int yaw_status = tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  if (tiny_set_x0(pitch_solver_, x0) != 0) return {false};

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  const int pitch_status = tiny_solve(pitch_solver_);
  if (yaw_status != 0 || pitch_status != 0) {
    if (!warned_solver_status_) {
      tools::logger()->warn(
        "[Planner] TinyMPC did not converge within max_iter yaw={} pitch={}", yaw_status,
        pitch_status);
      warned_solver_status_ = true;
    }
  }

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);
  plan.target_spin_speed_rad_s = target.ekf_x()[7];
  plan.target_spin_speed_valid = std::isfinite(plan.target_spin_speed_rad_s);
  if (target.name != ArmorName::outpost && !ordinary_spin_profile_.empty()) {
    const double spin_speed = plan.target_spin_speed_rad_s;
    const auto profile = ordinary_spin_profile_.sample(std::abs(spin_speed));
    plan.fire_alignment_hold_s = profile.aligned_hold_s;
  }

  if (
    !std::isfinite(plan.yaw) || !std::isfinite(plan.yaw_vel) || !std::isfinite(plan.yaw_acc) ||
    !std::isfinite(plan.pitch) || !std::isfinite(plan.pitch_vel) ||
    !std::isfinite(plan.pitch_acc)) {
    tools::logger()->warn("[Planner] TinyMPC produced non-finite output");
    return {false};
  }

  // 发弹延迟已计入参考轨迹；这里保留原有两步控制误差前视，不再重复加发弹延迟。
  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

bool Planner::updateAntiSpinState(const Target & target)
{
  if (target.ekf_x().size() <= 7) {
    anti_spin_mode_selector_.reset();
    return false;
  }
  return anti_spin_mode_selector_.update(
    target.ekf_x()[7], target.name != ArmorName::outpost);
}

Plan Planner::planAntiSpin(Target target, double bullet_speed)
{
  Plan plan;
  plan.control = true;
  plan.anti_spin_active = true;
  plan.target_spin_speed_rad_s = target.ekf_x()[7];
  plan.target_spin_speed_valid = std::isfinite(plan.target_spin_speed_rad_s);
  plan.fire_alignment_hold_s = anti_spin_guard_stable_time_s_;
  plan.fire_gate_yaw_tolerance_override_rad = anti_spin_fire_yaw_tolerance_rad_;
  plan.fire_gate_pitch_tolerance_override_rad = anti_spin_fire_pitch_tolerance_rad_;

  const Eigen::VectorXd state = target.ekf_x();
  if (state.size() <= 10 || !state.allFinite()) return {false};

  const double radius = std::clamp(state[8], 0.05, 0.5);
  const double initial_center_yaw = std::atan2(state[2], state[0]);
  Eigen::Vector3d guard_xyz{
    state[0] - radius * std::cos(initial_center_yaw),
    state[2] - radius * std::sin(initial_center_yaw),
    state[4]};
  double guard_distance = std::hypot(guard_xyz.x(), guard_xyz.y());
  tools::Trajectory guard_bullet(bullet_speed, guard_distance, guard_xyz.z());
  if (guard_bullet.unsolvable) return {false};

  // 用弹丸飞行时间预测命中时刻的目标中心，再选择届时最接近正面朝向的一块装甲板。
  Target hit_target = target;
  std::vector<Eigen::Vector4d> hit_armors;
  Eigen::VectorXd hit_state;
  int selected_id = 0;
  for (int iteration = 0; iteration < 5; ++iteration) {
    hit_target = target;
    hit_target.predict(guard_bullet.fly_time);
    hit_state = hit_target.ekf_x();
    hit_armors = hit_target.armor_xyza_list();
    if (hit_armors.empty()) return {false};

    const double hit_center_yaw = std::atan2(hit_state[2], hit_state[0]);
    double smallest_face_error = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < hit_armors.size(); ++i) {
      const double face_error = std::abs(
        tools::limit_rad(hit_armors[i][3] - hit_center_yaw));
      if (face_error < smallest_face_error) {
        smallest_face_error = face_error;
        selected_id = static_cast<int>(i);
      }
    }

    const auto & predicted_front_plate = hit_armors[selected_id];
    guard_xyz = predicted_front_plate.head<3>();
    guard_distance = std::hypot(guard_xyz.x(), guard_xyz.y());
    guard_bullet = tools::Trajectory(bullet_speed, guard_distance, guard_xyz.z());
    if (guard_bullet.unsolvable) return {false};
  }

  // 守候方向取目标中心的预测方位，俯仰取下一块正面装甲板的高度。
  const double guard_yaw_world = std::atan2(hit_state[2], hit_state[0]);
  const double guard_yaw = tools::limit_rad(guard_yaw_world + yaw_offset_);
  const double guard_pitch = -guard_bullet.pitch - pitch_offset_;
  debug_xyza = Eigen::Vector4d(guard_xyz.x(), guard_xyz.y(), guard_xyz.z(), guard_yaw_world);

  // 用固定守候角生成参考轨迹，避免 MPC 继续追着高速装甲板旋转。
  Trajectory guard_traj = Trajectory::Zero();
  for (int i = 0; i < HORIZON; ++i) {
    guard_traj.col(i) << 0.0, 0.0, guard_pitch, 0.0;
  }

  Eigen::VectorXd x0(2);
  x0 << guard_traj(0, 0), guard_traj(1, 0);
  if (tiny_set_x0(yaw_solver_, x0) != 0) return {false};
  yaw_solver_->work->Xref = guard_traj.block(0, 0, 2, HORIZON);
  const int yaw_status = tiny_solve(yaw_solver_);

  x0 << guard_traj(2, 0), guard_traj(3, 0);
  if (tiny_set_x0(pitch_solver_, x0) != 0) return {false};
  pitch_solver_->work->Xref = guard_traj.block(2, 0, 2, HORIZON);
  const int pitch_status = tiny_solve(pitch_solver_);
  if (yaw_status != 0 || pitch_status != 0) {
    tools::logger()->debug(
      "[Planner] 高速反小陀螺守候 MPC 未完全收敛 yaw={} pitch={}", yaw_status, pitch_status);
  }

  plan.target_yaw = guard_yaw;
  plan.target_pitch = guard_pitch;
  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + guard_yaw);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);
  plan.distance = static_cast<float>(guard_distance);

  if (!std::isfinite(plan.yaw) || !std::isfinite(plan.yaw_vel) ||
      !std::isfinite(plan.yaw_acc) || !std::isfinite(plan.pitch) ||
      !std::isfinite(plan.pitch_vel) || !std::isfinite(plan.pitch_acc)) {
    return {false};
  }

  // 同时检查板面朝向和装甲板中心是否落在守候弹道的有效角宽内。
  const double hit_center_yaw = std::atan2(hit_state[2], hit_state[0]);
  constexpr double kSmallArmorWidthM = 0.135;
  constexpr double kBigArmorWidthM = 0.230;
  constexpr double kArmorHeightM = 0.056;
  const double armor_width = target.armor_type == ArmorType::big
    ? kBigArmorWidthM
    : kSmallArmorWidthM;
  const double yaw_hit_tolerance =
    std::atan2(armor_width * 0.5, std::max(guard_distance, 0.1));
  const double pitch_hit_tolerance =
    std::atan2(kArmorHeightM * 0.5, std::max(guard_distance, 0.1));
  plan.fire = false;
  for (const auto & armor : hit_armors) {
    const double armor_yaw = std::atan2(armor.y(), armor.x());
    const double armor_distance = std::hypot(armor.x(), armor.y());
    const tools::Trajectory armor_bullet(bullet_speed, armor_distance, armor.z());
    if (armor_bullet.unsolvable) continue;
    const double armor_pitch = -armor_bullet.pitch - pitch_offset_;
    plan.fire = plan.fire || antiSpinShotCandidate(
      armor[3], hit_center_yaw, armor_yaw, guard_yaw_world,
      armor_pitch, guard_pitch, anti_spin_front_face_tolerance_rad_,
      yaw_hit_tolerance, pitch_hit_tolerance);
    if (plan.fire) break;
  }
  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed, double fire_delay_s)
{
  if (!target.has_value() || !std::isfinite(fire_delay_s) || fire_delay_s < 0.0) return {false};

  double delay_time;
  if (target->name == ArmorName::outpost) {
    // 前哨站保留既有固定模型和高低速延迟处理，不使用普通目标转速表。
    delay_time = std::abs(target->ekf_x()[7]) > decision_speed_
      ? high_speed_delay_time_
      : low_speed_delay_time_;
  } else if (!ordinary_spin_profile_.empty()) {
    delay_time = ordinary_spin_profile_.sample(std::abs(target->ekf_x()[7])).extra_prediction_s;
  } else {
    // 兼容没有转速标定表的旧配置，继续使用原来的高低速延迟。
    delay_time = std::abs(target->ekf_x()[7]) > decision_speed_
      ? high_speed_delay_time_
      : low_speed_delay_time_;
  }
  // 先预测到出膛时刻，再由下层加入弹丸飞行时间；高低速配置仅作为额外补偿。
  delay_time += fire_delay_s;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  return plan(*target, bullet_speed);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  const auto root = tools::load(config_path);
  const auto yaml = planner_node(root, planner_profile_);
  const auto fallback = planner_node(root, "mpc_planner");
  auto max_yaw_acc = read_planner_value<double>(yaml, fallback, "max_yaw_acc");
  auto Q_yaw = read_planner_value<std::vector<double>>(yaml, fallback, "Q_yaw");
  auto R_yaw = read_planner_value<std::vector<double>>(yaml, fallback, "R_yaw");
  if (Q_yaw.size() != 2 || R_yaw.size() != 1) {
    throw std::runtime_error("Invalid yaw MPC weight dimensions");
  }

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  if (tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0) != 0) {
    throw std::runtime_error("Failed to setup yaw MPC solver");
  }

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  if (tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max) != 0) {
    throw std::runtime_error("Failed to setup yaw MPC bounds");
  }

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  const auto root = tools::load(config_path);
  const auto yaml = planner_node(root, planner_profile_);
  const auto fallback = planner_node(root, "mpc_planner");
  auto max_pitch_acc = read_planner_value<double>(yaml, fallback, "max_pitch_acc");
  auto Q_pitch = read_planner_value<std::vector<double>>(yaml, fallback, "Q_pitch");
  auto R_pitch = read_planner_value<std::vector<double>>(yaml, fallback, "R_pitch");
  if (Q_pitch.size() != 2 || R_pitch.size() != 1) {
    throw std::runtime_error("Invalid pitch MPC weight dimensions");
  }

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  if (tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0) != 0) {
    throw std::runtime_error("Failed to setup pitch MPC solver");
  }

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  if (tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max) != 0) {
    throw std::runtime_error("Failed to setup pitch MPC bounds");
  }

  pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  const auto armor_xyza_list = target.armor_xyza_list();
  if (armor_xyza_list.empty()) throw std::runtime_error("Empty armor list!");
  for (auto & xyza : armor_xyza_list) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  if (armor_avoidance_config_.enabled) {
    if (!armor_avoidance_result_.valid) throw std::runtime_error("装甲板没有可用的避区瞄准点");
    applyArmorAvoidanceOffset(
      armor_avoidance_result_, yaw, xyz.x(), xyz.y(), xyz.z());
    min_dist = xyz.head<2>().norm();
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;
  Eigen::Vector4d horizon_xyza = Eigen::Vector4d::Zero();
  bool captured_horizon_xyza = false;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    // 命令时域中心对应的预测瞄准点用于距离输出，避免留下最后一帧预测点。
    if (i == HALF_HORIZON - 1) {
      horizon_xyza = debug_xyza;
      captured_horizon_xyza = true;
    }

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  if (captured_horizon_xyza) debug_xyza = horizon_xyza;

  return traj;
}

}  // namespace auto_aim
