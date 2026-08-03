// Copyright (C) FYT Vision Group. Licensed under the Apache License 2.0.
#ifndef GIMBAL_CONTROLLER__AIM_TRAJECTORY_PLANNER_HPP_
#define GIMBAL_CONTROLLER__AIM_TRAJECTORY_PLANNER_HPP_

#include <Eigen/Dense>

#include <vector>

#include "gimbal_controller/mpc/qp_solver.hpp"

namespace gimbal_controller
{

/**
 * @brief 自瞄轨迹规划器（轨迹视角，参考 sp_vision_25 §4 与其 planner 实现）
 *
 * 云台轨迹与射击轨迹（目标轨迹提前飞行时间）的重合度决定自瞄效果：
 * DPS = 射击窗口占比 × 射频 × 单发伤害，命中率 ≤ 重合度。
 * 装甲板切换使射击轨迹突变；本规划器以角加速度为决策变量、跟踪整条
 * 射击轨迹为参考做 QP，云台能力上限内的突变被自然平滑为可跟随的
 * 过渡段（平滑 ≠ 滞后：其余跟随段与射击轨迹完全重合）。
 *
 * 兵种适配只依赖物理参数：云台角加速度上限（配置项）。
 */
struct AimTrajectoryPlannerConfig
{
  bool enable = false;
  double max_yaw_acc = 80.0;    // yaw 角加速度上限 (rad/s²) —— 兵种适配核心参数
  double max_pitch_acc = 80.0;  // pitch 角加速度上限 (rad/s²)
  double q_angle = 100.0;       // 参考跟踪权重（角度）
  double q_rate = 1.0;          // 参考跟踪权重（角速度）
  double r_acc = 0.001;         // 输入（角加速度）权重
  int half_horizon = 25;        // 半程点数（参考总长 2*half_horizon）
  double dt = 0.02;             // 步长 (s)
  double fire_delay_s = 0.0;    // 开火延迟 t_fire (s)
  double fire_thresh = 0.02;    // 开火门控：t_fire 处 |规划轨迹-射击轨迹| (rad)
};

struct AimPlan
{
  bool valid = false;
  bool fire = false;
  double yaw = 0;        // 当前时刻规划瞄准角 (rad，相对参考零点)
  double yaw_rate = 0;   // 前馈角速度 (rad/s)
  double yaw_acc = 0;    // 前馈角加速度 (rad/s²)
  double pitch = 0;
  double pitch_rate = 0;
  double pitch_acc = 0;
  double track_err = 0;  // 诊断：t_fire 处轨迹偏差 (rad)
};

class AimTrajectoryPlanner
{
public:
  AimTrajectoryPlanner() { initSolvers(); }
  void setConfig(const AimTrajectoryPlannerConfig & cfg);

  /**
   * @brief 对一条射击轨迹参考做规划
   * @param ref_yaw   yaw 参考 (rad，相对任意零点的连续角度)，长度 2*half_horizon
   * @param ref_pitch pitch 参考 (rad)，同长；index=half_horizon 对应当前时刻
   * @return 当前时刻的规划角与前馈，以及开火判断
   */
  AimPlan plan(const std::vector<double> & ref_yaw,
               const std::vector<double> & ref_pitch);

  const AimTrajectoryPlannerConfig & cfg() const { return cfg_; }

private:
  void initSolvers();
  /// 单轴 QP：参考序列 ref（角/角速度交错 [angle, rate] × N）→ 当前点状态
  bool solveAxis(mpc::QPSolver & solver, double max_acc,
                 const std::vector<double> & ref_angle,
                 const std::vector<double> & ref_rate,
                 double & out_angle, double & out_rate, double & out_acc,
                 double & out_angle_at_fire) const;

  AimTrajectoryPlannerConfig cfg_;
  mpc::QPSolver yaw_solver_;
  mpc::QPSolver pitch_solver_;
  // 稠密化 H = q·MᵀM + rI（双积分器，与参考无关，配置变化时重建）
  Eigen::MatrixXd H_cache_;
  // 传播矩阵幂块（M: x_k = A^k x0 + M_k U）
  Eigen::MatrixXd M_cache_;      // (2N) × N
  Eigen::MatrixXd A_pow_cache_;  // (2N) × 2
  int cache_N_ = 0;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__AIM_TRAJECTORY_PLANNER_HPP_
