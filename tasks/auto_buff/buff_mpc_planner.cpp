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
  const char * q_key, const char * r_key)
{
  const double max_acc = readMpcDouble(node, max_acc_key);
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
    throw std::runtime_error("小符 TinyMPC 初始化失败");
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
    throw std::runtime_error("小符 TinyMPC 约束初始化失败");
  }
  (*solver)->settings->max_iter = 10;
}
}  // namespace

MpcPlanner::MpcPlanner(
  const std::string & controller_config, const std::string & buff_config)
{
  setupYawSolver(controller_config);
  setupPitchSolver(controller_config);

  const auto buff = YAML::LoadFile(buff_config);
  yaw_offset_ = buff["yaw_offset"].as<double>(0.0) * kDegToRad;
  pitch_offset_ = buff["pitch_offset"].as<double>(0.0) * kDegToRad;
  predict_time_ = buff["predict_time"].as<double>(predict_time_);
  if (!std::isfinite(yaw_offset_) || !std::isfinite(pitch_offset_) ||
      !std::isfinite(predict_time_) || predict_time_ < 0.0) {
    throw std::invalid_argument("小符 MPC 的偏移和预测时间必须是有限数，预测时间不能为负");
  }
}

void MpcPlanner::setupYawSolver(const std::string & controller_config)
{
  const auto node = mpcNode(controller_config);
  setupSolver(&yaw_solver_, node, "max_yaw_acc", "Q_yaw", "R_yaw");
}

void MpcPlanner::setupPitchSolver(const std::string & controller_config)
{
  const auto node = mpcNode(controller_config);
  setupSolver(&pitch_solver_, node, "max_pitch_acc", "Q_pitch", "R_pitch");
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

BuffMpcPlan MpcPlanner::plan(
  const SmallTarget & target, double bullet_speed,
  std::chrono::steady_clock::time_point timestamp, bool fire_advice)
{
  BuffMpcPlan result;
  if (target.is_unsolve()) return result;

  double frame_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - timestamp).count();
  if (!std::isfinite(frame_age)) frame_age = 0.0;
  frame_age = std::clamp(frame_age, 0.0, 0.5);
  const double center_prediction = frame_age + predict_time_;

  std::array<AimSample, auto_aim::HORIZON + 2> samples;
  for (int i = 0; i <= auto_aim::HORIZON + 1; ++i) {
    const double offset =
      (static_cast<double>(i - 1 - auto_aim::HALF_HORIZON)) * auto_aim::DT;
    // 参考轨迹的历史半窗不能把小符 EKF 回推到观测时刻以前。
    // 小符的角速度模型只保证正向预测，负 dt 会放大协方差并造成重获后的角度尖峰。
    const double sample_prediction = std::max(0.0, center_prediction + offset);
    samples[static_cast<std::size_t>(i)] =
      aimAt(target, sample_prediction, bullet_speed);
    if (!samples[static_cast<std::size_t>(i)].valid) return result;
  }

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
  x0 << trajectory(0, 0), trajectory(1, 0);
  if (tiny_set_x0(yaw_solver_, x0) != 0) return result;
  yaw_solver_->work->Xref = trajectory.block(0, 0, 2, auto_aim::HORIZON);
  const int yaw_status = tiny_solve(yaw_solver_);

  x0 << trajectory(2, 0), trajectory(3, 0);
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
  result.plan.fire = fire_advice;
  result.plan.target_yaw = static_cast<float>(tools::limit_rad(
    trajectory(0, auto_aim::HALF_HORIZON) + yaw0));
  result.plan.target_pitch = static_cast<float>(trajectory(2, auto_aim::HALF_HORIZON));
  result.plan.yaw = static_cast<float>(tools::limit_rad(
    yaw_solver_->work->x(0, auto_aim::HALF_HORIZON) + yaw0));
  result.plan.yaw_vel = static_cast<float>(yaw_solver_->work->x(1, auto_aim::HALF_HORIZON));
  result.plan.yaw_acc = static_cast<float>(yaw_solver_->work->u(0, auto_aim::HALF_HORIZON));
  result.plan.pitch = static_cast<float>(pitch_solver_->work->x(0, auto_aim::HALF_HORIZON));
  result.plan.pitch_vel = static_cast<float>(pitch_solver_->work->x(1, auto_aim::HALF_HORIZON));
  result.plan.pitch_acc = static_cast<float>(pitch_solver_->work->u(0, auto_aim::HALF_HORIZON));
  result.distance = samples[auto_aim::HALF_HORIZON + 1].distance;

  if (!std::isfinite(result.plan.yaw) || !std::isfinite(result.plan.yaw_vel) ||
      !std::isfinite(result.plan.yaw_acc) || !std::isfinite(result.plan.pitch) ||
      !std::isfinite(result.plan.pitch_vel) || !std::isfinite(result.plan.pitch_acc) ||
      !std::isfinite(result.distance)) {
    return BuffMpcPlan{};
  }
  result.valid = true;
  return result;
}

}  // namespace auto_buff
