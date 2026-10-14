#include "buff_mpc_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "tools/math_tools.hpp"
#include "tools/logger.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

YAML::Node controllerNode(const YAML::Node & root)
{
  const auto nested = root["gimbal_pipeline"]["ros__parameters"]["controller"];
  if (nested) return nested;
  const auto direct = root["controller"];
  if (direct) return direct;
  return root;
}

YAML::Node mpcNode(const std::string & config_path)
{
  const auto root = YAML::LoadFile(config_path);
  const auto node = controllerNode(root)["mpc_planner"];
  if (!node) throw std::invalid_argument("controller.mpc_planner 配置不存在");
  return node;
}

double readMpcDouble(const YAML::Node & node, const char * key)
{
  if (!node[key]) throw std::invalid_argument(std::string("controller.mpc_planner 缺少 ") + key);
  const double value = node[key].as<double>();
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("controller.mpc_planner.") + key + " 必须是有限数");
  }
  return value;
}

std::vector<double> readMpcVector(const YAML::Node & node, const char * key, std::size_t size)
{
  if (!node[key]) throw std::invalid_argument(std::string("controller.mpc_planner 缺少 ") + key);
  const auto values = node[key].as<std::vector<double>>();
  if (values.size() != size) {
    throw std::invalid_argument(std::string("controller.mpc_planner.") + key + " 维度错误");
  }
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string("controller.mpc_planner.") + key + " 必须是有限数");
    }
  }
  return values;
}

void setupSolver(
  TinySolver ** solver, const YAML::Node & node, const char * max_acc_key,
  const char * q_key, const char * r_key,
  const std::optional<double> & max_acceleration_override)
{
  const double max_acc = max_acceleration_override.has_value()
                           ? *max_acceleration_override
                           : readMpcDouble(node, max_acc_key);
  if (max_acc <= 0.0) {
    throw std::invalid_argument(std::string("controller.mpc_planner.") + max_acc_key + " 必须 > 0");
  }
  const auto q_values = readMpcVector(node, q_key, 2);
  const auto r_values = readMpcVector(node, r_key, 1);

  Eigen::MatrixXd A{{1.0, auto_aim::DT}, {0.0, 1.0}};
  Eigen::MatrixXd B{{0.0}, {auto_aim::DT}};
  Eigen::VectorXd f{{0.0, 0.0}};
  Eigen::Matrix<double, 2, 1> Q(q_values.data());
  Eigen::Matrix<double, 1, 1> R(r_values.data());
  if (tiny_setup(
        solver, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1,
        auto_aim::HORIZON, 0) != 0) {
    throw std::runtime_error("打符 TinyMPC 初始化失败");
  }

  const Eigen::MatrixXd x_min =
    Eigen::MatrixXd::Constant(2, auto_aim::HORIZON, -1e17);
  const Eigen::MatrixXd x_max =
    Eigen::MatrixXd::Constant(2, auto_aim::HORIZON, 1e17);
  const Eigen::MatrixXd u_min =
    Eigen::MatrixXd::Constant(1, auto_aim::HORIZON - 1, -max_acc);
  const Eigen::MatrixXd u_max =
    Eigen::MatrixXd::Constant(1, auto_aim::HORIZON - 1, max_acc);
  if (tiny_set_bound_constraints(*solver, x_min, x_max, u_min, u_max) != 0) {
    throw std::runtime_error("打符 TinyMPC 约束初始化失败");
  }
  (*solver)->settings->max_iter = 10;
}
}  // namespace

SmallBuffMpcOverrides loadSmallBuffMpcOverrides(const std::string & buff_config)
{
  SmallBuffMpcOverrides result;
  const auto root = YAML::LoadFile(buff_config);
  const auto node = root["smallbuff_mpc"];
  if (!node) return result;

  const auto readPositive = [&](const char * key, const char * unit) -> std::optional<double> {
    if (!node[key]) return std::nullopt;
    const double value = node[key].as<double>();
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
        std::string("buff.smallbuff_mpc.") + key + " 必须是有限正数 (" + unit + ")");
    }
    return value;
  };
  const auto readNonNegative = [&](const char * key) -> std::optional<double> {
    if (!node[key]) return std::nullopt;
    const double value = node[key].as<double>();
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(
        std::string("buff.smallbuff_mpc.") + key + " 必须是有限非负数");
    }
    return value;
  };

  result.max_yaw_acceleration_rad_s2 = readPositive("max_yaw_acceleration", "rad/s^2");
  result.max_pitch_acceleration_rad_s2 = readPositive("max_pitch_acceleration", "rad/s^2");
  result.yaw_error_gain = readNonNegative("yaw_error_gain");
  result.pitch_error_gain = readNonNegative("pitch_error_gain");
  const auto max_yaw_velocity_deg_s = readPositive("max_yaw_velocity", "deg/s");
  const auto max_pitch_velocity_deg_s = readPositive("max_pitch_velocity", "deg/s");
  if (max_yaw_velocity_deg_s.has_value()) {
    result.max_yaw_velocity_rad_s = *max_yaw_velocity_deg_s * kDegToRad;
  }
  if (max_pitch_velocity_deg_s.has_value()) {
    result.max_pitch_velocity_rad_s = *max_pitch_velocity_deg_s * kDegToRad;
  }
  return result;
}

MpcPlanner::MpcPlanner(
  const std::string & controller_config, const std::string & buff_config,
  bool use_smallbuff_overrides)
{
  const auto overrides = use_smallbuff_overrides
                           ? loadSmallBuffMpcOverrides(buff_config)
                           : SmallBuffMpcOverrides{};
  setupYawSolver(controller_config, overrides.max_yaw_acceleration_rad_s2);
  setupPitchSolver(controller_config, overrides.max_pitch_acceleration_rad_s2);

  const auto buff = YAML::LoadFile(buff_config);
  yaw_offset_ = buff["yaw_offset"].as<double>(0.0) * kDegToRad;
  pitch_offset_ = buff["pitch_offset"].as<double>(0.0) * kDegToRad;
  const char * prediction_key = use_smallbuff_overrides
                                  ? "smallbuff_predict_time"
                                  : "bigbuff_predict_time";
  predict_time_ = buff[prediction_key]
                    ? buff[prediction_key].as<double>()
                    : buff["predict_time"].as<double>(predict_time_);
  constexpr double kMaxAbsPredictionTime = 0.5;
  if (!std::isfinite(yaw_offset_) || !std::isfinite(pitch_offset_) ||
      !std::isfinite(predict_time_) || std::abs(predict_time_) > kMaxAbsPredictionTime) {
    throw std::invalid_argument(
      std::string("打符 MPC 的偏移必须是有限数，") + prediction_key +
      " 必须是 [-0.5, 0.5] 秒范围内的有限数");
  }
}

void MpcPlanner::setupYawSolver(
  const std::string & controller_config,
  const std::optional<double> & max_acceleration_override)
{
  const auto node = mpcNode(controller_config);
  setupSolver(
    &yaw_solver_, node, "max_yaw_acc", "Q_yaw", "R_yaw", max_acceleration_override);
}

void MpcPlanner::setupPitchSolver(
  const std::string & controller_config,
  const std::optional<double> & max_acceleration_override)
{
  const auto node = mpcNode(controller_config);
  setupSolver(
    &pitch_solver_, node, "max_pitch_acc", "Q_pitch", "R_pitch", max_acceleration_override);
}

MpcPlanner::AimSample MpcPlanner::aimAt(
  SmallTarget target, double prediction_time, double bullet_speed) const
{
  AimSample result;
  if (target.is_unsolve() || !std::isfinite(prediction_time)) return result;
  if (!std::isfinite(bullet_speed) || bullet_speed < 14.0) bullet_speed = 23.0;

  target.predict(prediction_time);
  auto aim_point = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  double distance = std::hypot(aim_point.x(), aim_point.y());
  double height = aim_point.z();
  tools::Trajectory trajectory0(bullet_speed, distance, height);
  if (trajectory0.unsolvable) return result;

  target.predict(trajectory0.fly_time);
  aim_point = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  distance = std::hypot(aim_point.x(), aim_point.y());
  height = aim_point.z();
  tools::Trajectory trajectory1(bullet_speed, distance, height);
  if (trajectory1.unsolvable || std::abs(trajectory1.fly_time - trajectory0.fly_time) > 0.01) {
    return result;
  }

  result.yaw = std::atan2(aim_point.y(), aim_point.x()) + yaw_offset_;
  result.pitch = -(trajectory1.pitch + pitch_offset_);
  result.distance = distance;
  result.valid = std::isfinite(result.yaw) && std::isfinite(result.pitch) &&
                 std::isfinite(result.distance) && result.distance > 0.0;
  return result;
}

MpcPlanner::AimSample MpcPlanner::aimAt(
  BigTarget target, double prediction_time, double bullet_speed) const
{
  AimSample result;
  if (target.is_unsolve() || !std::isfinite(prediction_time)) return result;
  if (!std::isfinite(bullet_speed) || bullet_speed < 14.0) bullet_speed = 23.0;

  target.predict(prediction_time);
  auto aim_point = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  double distance = std::hypot(aim_point.x(), aim_point.y());
  double height = aim_point.z();
  tools::Trajectory trajectory0(bullet_speed, distance, height);
  if (trajectory0.unsolvable) return result;

  target.predict(trajectory0.fly_time);
  aim_point = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  distance = std::hypot(aim_point.x(), aim_point.y());
  height = aim_point.z();
  tools::Trajectory trajectory1(bullet_speed, distance, height);
  if (trajectory1.unsolvable || std::abs(trajectory1.fly_time - trajectory0.fly_time) > 0.01) {
    return result;
  }

  result.yaw = std::atan2(aim_point.y(), aim_point.x()) + yaw_offset_;
  result.pitch = -(trajectory1.pitch + pitch_offset_);
  result.distance = distance;
  result.valid = std::isfinite(result.yaw) && std::isfinite(result.pitch) &&
                 std::isfinite(result.distance) && result.distance > 0.0;
  return result;
}

MpcPlanner::AimSample MpcPlanner::aimAtStaticPoint(
  const Eigen::Vector3d & point_world, double bullet_speed) const
{
  AimSample result;
  if (!point_world.allFinite()) return result;
  if (!std::isfinite(bullet_speed) || bullet_speed < 14.0) bullet_speed = 23.0;

  const double distance = std::hypot(point_world.x(), point_world.y());
  if (!std::isfinite(distance) || distance <= 1e-6) return result;

  tools::Trajectory trajectory(bullet_speed, distance, point_world.z());
  if (trajectory.unsolvable || !std::isfinite(trajectory.pitch)) return result;

  result.yaw = std::atan2(point_world.y(), point_world.x()) + yaw_offset_;
  result.pitch = -(trajectory.pitch + pitch_offset_);
  result.distance = distance;
  result.valid = std::isfinite(result.yaw) && std::isfinite(result.pitch) &&
                 std::isfinite(result.distance);
  return result;
}

BuffMpcPlan MpcPlanner::plan(
  const SmallTarget & target, double bullet_speed,
  std::chrono::steady_clock::time_point timestamp,
  double current_yaw, double current_pitch)
{
  BuffMpcPlan result;
  if (target.is_unsolve()) return result;

  double frame_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - timestamp).count();
  if (!std::isfinite(frame_age)) frame_age = 0.0;
  frame_age = std::clamp(frame_age, 0.0, 0.5);
  // 与普通自瞄 Planner 保持同一时间组织：center_prediction 对应输出中点，
  // 轨迹在它前后各取半个 horizon。小符的状态模型是匀速模型，允许对复制的
  // 状态做小范围负时间预测；这样速度用中心差分，不会因为窗口起点的人为
  // 单边差分而给 MPC 注入一个额外的速度跳变。
  const double center_prediction = frame_age + predict_time_;

  std::array<AimSample, auto_aim::HORIZON + 2> samples;
  for (int i = 0; i <= auto_aim::HORIZON + 1; ++i) {
    const double offset =
      static_cast<double>(i - 1 - auto_aim::HALF_HORIZON) * auto_aim::DT;
    const double sample_prediction = center_prediction + offset;
    samples[static_cast<std::size_t>(i)] =
      aimAt(target, sample_prediction, bullet_speed);
    if (!samples[static_cast<std::size_t>(i)].valid) return result;
  }

  return solveSamples(samples, current_yaw, current_pitch, false);
}

BuffMpcPlan MpcPlanner::plan(
  const BigTarget & target, double bullet_speed,
  std::chrono::steady_clock::time_point timestamp,
  double current_yaw, double current_pitch)
{
  BuffMpcPlan result;
  if (target.is_unsolve()) return result;

  double frame_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - timestamp).count();
  if (!std::isfinite(frame_age)) frame_age = 0.0;
  frame_age = std::clamp(frame_age, 0.0, 0.5);
  // 大符使用自身的变速模型逐点生成动态参考；时间窗口和小符旋转路径
  // 一致，中心差分避免单边采样把模型速度变成额外的控制跳变。
  const double center_prediction = frame_age + predict_time_;

  std::array<AimSample, auto_aim::HORIZON + 2> samples;
  for (int i = 0; i <= auto_aim::HORIZON + 1; ++i) {
    const double offset =
      static_cast<double>(i - 1 - auto_aim::HALF_HORIZON) * auto_aim::DT;
    const double sample_prediction = center_prediction + offset;
    samples[static_cast<std::size_t>(i)] =
      aimAt(target, sample_prediction, bullet_speed);
    if (!samples[static_cast<std::size_t>(i)].valid) return result;
  }

  return solveSamples(samples, current_yaw, current_pitch, false);
}

BuffMpcPlan MpcPlanner::planStaticPoint(
  const Eigen::Vector3d & point_world, double bullet_speed,
  double current_yaw, double current_pitch)
{
  BuffMpcPlan result;
  const AimSample sample = aimAtStaticPoint(point_world, bullet_speed);
  if (!sample.valid) return result;

  std::array<AimSample, auto_aim::HORIZON + 2> samples;
  samples.fill(sample);
  return solveSamples(samples, current_yaw, current_pitch, true);
}

BuffMpcPlan MpcPlanner::solveSamples(
  const std::array<AimSample, auto_aim::HORIZON + 2> & samples,
  double current_yaw, double current_pitch, bool use_feedback_initial_state)
{
  BuffMpcPlan result;
  const double yaw0 = samples[auto_aim::HALF_HORIZON + 1].yaw;
  auto_aim::Trajectory trajectory;
  for (int i = 0; i < auto_aim::HORIZON; ++i) {
    const auto & before = samples[static_cast<std::size_t>(i)];
    const auto & center = samples[static_cast<std::size_t>(i + 1)];
    const auto & after = samples[static_cast<std::size_t>(i + 2)];
    trajectory(0, i) = tools::limit_rad(center.yaw - yaw0);
    trajectory(1, i) = tools::limit_rad(after.yaw - before.yaw) / (2.0 * auto_aim::DT);
    trajectory(2, i) = center.pitch;
    trajectory(3, i) = (after.pitch - before.pitch) / (2.0 * auto_aim::DT);
  }

  Eigen::VectorXd x0(2);
  if (!std::isfinite(current_yaw) || !std::isfinite(current_pitch)) return result;
  if (use_feedback_initial_state) {
    // 静止固定点已经在实机验证过，保留它原来的反馈初态。
    x0 << tools::limit_rad(current_yaw - yaw0), trajectory(1, 0);
  } else {
    // 旋转小符与普通自瞄 MPC 一致，求解器初态使用参考窗口起点；真实
    // 云台反馈只在 standard.cpp 的 feedback_error 速度环中闭合，避免双重放大。
    x0 << trajectory(0, 0), trajectory(1, 0);
  }
  if (tiny_set_x0(yaw_solver_, x0) != 0) return result;
  yaw_solver_->work->Xref = trajectory.block(0, 0, 2, auto_aim::HORIZON);
  const int yaw_status = tiny_solve(yaw_solver_);

  x0 << (use_feedback_initial_state ? current_pitch : trajectory(2, 0)), trajectory(3, 0);
  if (tiny_set_x0(pitch_solver_, x0) != 0) return result;
  pitch_solver_->work->Xref = trajectory.block(2, 0, 2, auto_aim::HORIZON);
  const int pitch_status = tiny_solve(pitch_solver_);
  // 与普通自瞄 Planner 保持一致：求解器达到迭代上限时仍使用有限的最近解，
  // 由后面的有限性检查兜底，避免偶发一次未收敛就退回抖动更大的旧链路。
  if ((yaw_status != 0 || pitch_status != 0) && !warned_solver_status_) {
    tools::logger()->warn(
      "[BuffMpcPlanner] TinyMPC 未在迭代上限内收敛 yaw={} pitch={}，继续使用有限解",
      yaw_status, pitch_status);
    warned_solver_status_ = true;
  }

  result.plan.control = true;
  // 开火节拍由 Aimer 的独立状态机决定，规划器只输出轨迹。
  result.plan.fire = false;
  const int kOutputIndex = use_feedback_initial_state ? 1 : auto_aim::HALF_HORIZON;
  result.plan.target_yaw = static_cast<float>(tools::limit_rad(
    trajectory(0, kOutputIndex) + yaw0));
  result.plan.target_pitch = static_cast<float>(trajectory(2, kOutputIndex));
  result.plan.yaw = static_cast<float>(tools::limit_rad(
    yaw_solver_->work->x(0, kOutputIndex) + yaw0));
  result.plan.yaw_vel = static_cast<float>(yaw_solver_->work->x(1, kOutputIndex));
  result.plan.yaw_acc = static_cast<float>(yaw_solver_->work->u(0, kOutputIndex));
  result.plan.pitch = static_cast<float>(pitch_solver_->work->x(0, kOutputIndex));
  result.plan.pitch_vel = static_cast<float>(pitch_solver_->work->x(1, kOutputIndex));
  result.plan.pitch_acc = static_cast<float>(pitch_solver_->work->u(0, kOutputIndex));
  result.distance = samples[kOutputIndex + 1].distance;

  if (!std::isfinite(result.plan.yaw) || !std::isfinite(result.plan.yaw_vel) ||
      !std::isfinite(result.plan.yaw_acc) || !std::isfinite(result.plan.pitch) ||
      !std::isfinite(result.plan.pitch_vel) || !std::isfinite(result.plan.pitch_acc) ||
      !std::isfinite(result.distance)) {
    return BuffMpcPlan{};
  }
  result.valid = true;
  return result;
}

SmallBuffController::SmallBuffController(
  const std::string & controller_config, const std::string & buff_config, Aimer & aimer)
: planner_(controller_config, buff_config), aimer_(aimer)
{
}

BuffMpcPlan SmallBuffController::update(
  const SmallTarget & target, double bullet_speed,
  std::chrono::steady_clock::time_point timestamp,
  double current_yaw, double current_pitch)
{
  if (!target.is_tracking_ready()) {
    reset();
    return BuffMpcPlan{};
  }

  auto result = planner_.plan(target, bullet_speed, timestamp, current_yaw, current_pitch);
  if (!result.valid) {
    reset();
    return result;
  }

  // MpcPlanner 的 pitch 是世界坐标约定（向上为负），Aimer 状态机沿用
  // 原来的正俯仰角约定，因此只在状态机边界做一次取反。
  const auto fire = aimer_.fireAdvice(
    SMALL, result.plan.target_yaw, -result.plan.target_pitch,
    std::chrono::steady_clock::now());
  result.plan.fire = fire.shoot;
  return result;
}

BuffMpcPlan SmallBuffController::updateStaticPoint(
  const Eigen::Vector3d & point_world, double bullet_speed,
  double current_yaw, double current_pitch)
{
  auto result = planner_.planStaticPoint(
    point_world, bullet_speed, current_yaw, current_pitch);
  if (!result.valid) {
    reset();
    return result;
  }

  const auto fire = aimer_.fireAdvice(
    SMALL, result.plan.target_yaw, -result.plan.target_pitch,
    std::chrono::steady_clock::now());
  result.plan.fire = fire.shoot;
  return result;
}

void SmallBuffController::reset()
{
  aimer_.resetFireState();
}

BigBuffController::BigBuffController(
  const std::string & controller_config, const std::string & buff_config, Aimer & aimer)
: planner_(controller_config, buff_config, false), aimer_(aimer)
{
}

BuffMpcPlan BigBuffController::update(
  const BigTarget & target, double bullet_speed,
  std::chrono::steady_clock::time_point timestamp,
  double current_yaw, double current_pitch)
{
  if (!target.is_tracking_ready()) {
    reset();
    return BuffMpcPlan{};
  }

  auto result = planner_.plan(target, bullet_speed, timestamp, current_yaw, current_pitch);
  if (!result.valid) {
    reset();
    return result;
  }

  const auto fire = aimer_.fireAdvice(
    BIG, result.plan.target_yaw, -result.plan.target_pitch,
    std::chrono::steady_clock::now());
  result.plan.fire = fire.shoot;
  return result;
}

void BigBuffController::reset()
{
  aimer_.resetFireState();
}

}  // namespace auto_buff
