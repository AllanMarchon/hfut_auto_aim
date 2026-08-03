// Copyright (C) FYT Vision Group. Licensed under the Apache License 2.0.

#include "gimbal_controller/aim_trajectory_planner.hpp"

#include <cmath>

namespace gimbal_controller
{

void AimTrajectoryPlanner::setConfig(const AimTrajectoryPlannerConfig & cfg)
{
  cfg_ = cfg;
  initSolvers();
}

void AimTrajectoryPlanner::initSolvers()
{
  yaw_solver_.init(100, 0.004);
  pitch_solver_.init(100, 0.004);

  // 预计算双积分器传播的稠密块：x_k = A^k x0 + M_k U
  const int N = 2 * cfg_.half_horizon;
  const double dt = cfg_.dt;
  M_cache_.setZero(2 * N, N);
  A_pow_cache_.setZero(2 * N, 2);
  for (int k = 0; k < N; ++k) {
    // x_k = A^(k+1) x0 + Σ_{j=0..k} A^(k-j) B u_j
    const int p = k + 1;
    A_pow_cache_(2 * k, 0) = 1.0;
    A_pow_cache_(2 * k, 1) = p * dt;
    A_pow_cache_(2 * k + 1, 0) = 0.0;
    A_pow_cache_(2 * k + 1, 1) = 1.0;
    for (int j = 0; j <= k; ++j) {
      const int q = k - j;  // A^q B = [(q dt)^2/2 + ... ] 精确离散: [(q+1)dt^2/2 - ...]
      // 精确离散双积分器: angle += (q*dt)*dt + dt²/2, rate += dt
      M_cache_(2 * k, j) = (q * dt) * dt + 0.5 * dt * dt;
      M_cache_(2 * k + 1, j) = dt;
    }
  }
  // H = q·MᵀM + rI（角/速率块分别加权）
  Eigen::MatrixXd W = Eigen::MatrixXd::Zero(2 * N, 2 * N);
  for (int k = 0; k < N; ++k) {
    W(2 * k, 2 * k) = cfg_.q_angle;
    W(2 * k + 1, 2 * k + 1) = cfg_.q_rate;
  }
  H_cache_ = M_cache_.transpose() * W * M_cache_ +
             cfg_.r_acc * Eigen::MatrixXd::Identity(N, N);
  H_cache_ = 0.5 * (H_cache_ + H_cache_.transpose()) +
             1e-9 * Eigen::MatrixXd::Identity(N, N);
  cache_N_ = N;
}

bool AimTrajectoryPlanner::solveAxis(
  mpc::QPSolver & solver, double max_acc,
  const std::vector<double> & ref_angle,
  const std::vector<double> & ref_rate,
  double & out_angle, double & out_rate, double & out_acc,
  double & out_angle_at_fire) const
{
  const int N = cache_N_;
  // f = -Mᵀ W (r - A_pow x0)，x0 = 参考起点（轨迹视角下规划自参考窗口起点出发，
  // 保证整窗可行；输出取窗口中点=当前）
  Eigen::VectorXd r(2 * N), x0(2);
  x0 << ref_angle[0], ref_rate[0];
  for (int k = 0; k < N; ++k) {
    r(2 * k) = ref_angle[k];
    r(2 * k + 1) = ref_rate[k];
  }
  Eigen::VectorXd W_err(2 * N);
  for (int k = 0; k < N; ++k) {
    W_err(2 * k) = cfg_.q_angle * (r(2 * k) - (A_pow_cache_(2 * k, 0) * x0(0) +
                                               A_pow_cache_(2 * k, 1) * x0(1)));
    W_err(2 * k + 1) = cfg_.q_rate * (r(2 * k + 1) - (A_pow_cache_(2 * k + 1, 0) * x0(0) +
                                                      A_pow_cache_(2 * k + 1, 1) * x0(1)));
  }
  Eigen::VectorXd f = -M_cache_.transpose() * W_err;

  Eigen::VectorXd lb = Eigen::VectorXd::Constant(N, -max_acc);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(N, max_acc);
  auto res = solver.solve(H_cache_, f, lb, ub);
  if (!res.success) return false;

  // 由 U 恢复状态：x_k = A^k x0 + M_k U
  Eigen::VectorXd X = A_pow_cache_ * x0 + M_cache_ * res.U;
  const int now = cfg_.half_horizon;
  out_angle = X(2 * now);
  out_rate = X(2 * now + 1);
  out_acc = res.U(now);
  int fire_idx = now + static_cast<int>(std::lround(cfg_.fire_delay_s / cfg_.dt));
  fire_idx = std::clamp(fire_idx, 0, N - 1);
  out_angle_at_fire = X(2 * fire_idx);
  return true;
}

AimPlan AimTrajectoryPlanner::plan(const std::vector<double> & ref_yaw,
                                   const std::vector<double> & ref_pitch)
{
  AimPlan plan;
  const int N = cache_N_;
  if (static_cast<int>(ref_yaw.size()) != N ||
      static_cast<int>(ref_pitch.size()) != N) {
    return plan;
  }

  // 角速度参考：中心差分
  std::vector<double> ref_yaw_rate(N), ref_pitch_rate(N);
  for (int k = 0; k < N; ++k) {
    const int km = std::max(0, k - 1), kp = std::min(N - 1, k + 1);
    ref_yaw_rate[k] = (ref_yaw[kp] - ref_yaw[km]) / ((kp - km) * cfg_.dt);
    ref_pitch_rate[k] = (ref_pitch[kp] - ref_pitch[km]) / ((kp - km) * cfg_.dt);
  }

  double yaw_at_fire = 0, pitch_at_fire = 0;
  if (!solveAxis(yaw_solver_, cfg_.max_yaw_acc, ref_yaw, ref_yaw_rate,
                 plan.yaw, plan.yaw_rate, plan.yaw_acc, yaw_at_fire)) {
    return plan;
  }
  if (!solveAxis(pitch_solver_, cfg_.max_pitch_acc, ref_pitch, ref_pitch_rate,
                 plan.pitch, plan.pitch_rate, plan.pitch_acc, pitch_at_fire)) {
    return plan;
  }

  // 开火：t_fire 处 规划轨迹 与 射击轨迹 的偏差（轨迹视角的核心门控）
  const int fire_idx = std::clamp(
    cfg_.half_horizon + static_cast<int>(std::lround(cfg_.fire_delay_s / cfg_.dt)),
    0, N - 1);
  plan.track_err = std::hypot(ref_yaw[fire_idx] - yaw_at_fire,
                              ref_pitch[fire_idx] - pitch_at_fire);
  plan.fire = plan.track_err < cfg_.fire_thresh;
  plan.valid = true;
  return plan;
}

}  // namespace gimbal_controller
