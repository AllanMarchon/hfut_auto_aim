// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gimbal_controller/strategies/predicted_position_strategy.hpp"
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"
#include <angles/angles.h>

#include <algorithm>
#include <cmath>

#include <iostream>

namespace gimbal_controller
{

rm_interfaces::msg::GimbalCmd PredictedPositionStrategy::solve(
  const GimbalControlContext & context)
{
  clearControlTargetDebug();
  // TEMP_LOST still carries a predicted target state. Keep steering from that
  // estimate; the orchestrator independently suppresses fire until a fresh
  // TRACKING observation returns.
  if (!context.is_tracking && !context.is_temp_lost) {
    markDelayAuditInvalid(getName(), false);
    state_ = TRACKING_ARMOR;
    overflow_count_ = 0;
    if (armor_selector_) {
      armor_selector_->resetState();
    }
    // 跟丢目标时重置自适应 delay 状态
    if (adaptive_delay_enabled_) {
      adaptive_ctrl_.reset();
    }
    return createIdleCmd();
  }

  // 检查组件是否已设置
  if (!position_calculator_ || !armor_selector_) {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  const auto robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const auto center_position =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerPosition(robot);
  const auto linear_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(robot);
  const double yaw_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawVelocity(robot);

  // 估计飞行时间 (使用当前位置)
  const Eigen::Vector3d current_center = center_position;

  double flight_time = 0;
  if (local_compensator_) {
    local_compensator_->setBulletSpeed(context.bullet_speed);
    flight_time = local_compensator_->getFlyingTime(current_center);
  } else {
    flight_time = current_center.norm() / context.bullet_speed;
  }

  // 计算总预测时间 = 处理延迟 + 飞行时间 + 额外预测延迟
  delay_management::DelayRawInputs delay_raw;
  delay_raw.current_time = context.current_time;
  delay_raw.observation_stamp = context.target_stamp;
  delay_raw.prediction_extra_s = prediction_delay_;
  delay_raw.max_processing_delay_s = max_processing_delay_s_;

  double processing_delay = delay_manager_.computeProcessingDelay(delay_raw);
  double total_prediction_time = delay_manager_.computePredictionTime(
    delay_raw, flight_time, max_prediction_time_);

  std::cout << "Processing delay: " << processing_delay
            << " s, Flight time: " << flight_time
            << " s, Total prediction time: " << total_prediction_time << " s" << std::endl;

  // 计算预测位置的装甲板坐标
  auto predicted_armor_positions = position_calculator_->calculatePredicted(
    robot, total_prediction_time);

  // 计算当前位置的装甲板坐标 (用于开火判断)
  auto current_armor_positions = position_calculator_->calculate(robot);

  if (predicted_armor_positions.empty() || current_armor_positions.empty()) {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  // 预测中心位置与 yaw (CA 外推：小幅快速换向运动时加速度项自然收敛提前量，
  // 避免匀速外推把短暂小动作放大成大幅度预测)
  Eigen::Vector3d predicted_center =
    fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
    robot,
    total_prediction_time,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_ACCELERATION);
  double predicted_yaw =
    fyt::auto_aim::robot_description::TrackedRobotUsage::predictYaw(
    robot,
    total_prediction_time,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_ACCELERATION);

  // 轨迹规划器接管瞄准：跟随段与射击轨迹重合，换板按云台加速度上限
  // QP 平滑过渡（轨迹视角，sp_vision_25 §4）。
  if (aim_planner_.cfg().enable) {
    return solveWithPlanner(context, robot, linear_velocity,
                            total_prediction_time, processing_delay, flight_time);
  }

  // 选择最佳装甲板 (基于预测位置，路由到配置的选板策略)
  auto predicted_selection = armor_selector_->selectBest(
    predicted_armor_positions,
    predicted_center,
    predicted_yaw,
    robot.num_armors,
    yaw_velocity,
    context.current_yaw,
    context.current_pitch);
  auto applied_selection = predicted_selection;
  auto applied_armor_positions = predicted_armor_positions;

  // 选择最佳装甲板 (基于当前位置，用于开火判断，始终使用最小运动量算法)
  auto current_selection = armor_selector_->selectByMinMovement(
    current_armor_positions,
    context.current_yaw,
    context.current_pitch);

  if (current_selection.selected_index < 0) {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  // 高转速状态机处理
  double abs_v_yaw = std::abs(yaw_velocity);

  switch (state_) {
    case TRACKING_ARMOR:
      if (abs_v_yaw > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }
      if (overflow_count_ > transfer_thresh_) {
        state_ = TRACKING_CENTER;
      }
      break;

    case TRACKING_CENTER:
      if (abs_v_yaw < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }
      if (overflow_count_ > transfer_thresh_) {
        state_ = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      break;
  }

  // 根据状态选择目标位置
  Eigen::Vector3d control_target_position;
  Eigen::Vector3d applied_predicted_center = predicted_center;
  double applied_control_latency = 0.0;
  double applied_prediction_time = total_prediction_time;

  if (state_ == TRACKING_CENTER) {
    // 高转速时跟踪机器人中心 (预测位置)
    control_target_position = predicted_center;
  } else {
    // 正常跟踪预测装甲板位置 (含 facing 过滤后的结果或 center fallback)
    control_target_position = predicted_selection.position;

    // controller_delay 前馈：在 TRACKING_ARMOR 状态且配置了 controller_delay 时，
    // 在 total_prediction_time 基础上再额外预测 controller_delay_ 秒，
    // 强制云台超前运动（与 armor_solver 原版 controller_delay 机制一致）
    // 若启用自适应模式，则使用 adaptive_ctrl_ 的当前 delay 替代静态 controller_delay_
    double effective_ctrl_delay =
      adaptive_delay_enabled_ ? adaptive_ctrl_.getDelay() : controller_delay_;
    if (effective_ctrl_delay > 0.0) {
      double extra_dt = total_prediction_time + effective_ctrl_delay;
      extra_dt = std::min(extra_dt, max_prediction_time_);
      auto extra_positions = position_calculator_->calculatePredicted(robot, extra_dt);
      if (!extra_positions.empty()) {
        applied_control_latency = std::max(effective_ctrl_delay, 0.0);
        applied_prediction_time = extra_dt;
        Eigen::Vector3d extra_center =
          fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
          robot,
          extra_dt,
          fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
        double extra_yaw =
          fyt::auto_aim::robot_description::TrackedRobotUsage::predictYaw(
          robot,
          extra_dt,
          fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
        auto extra_selection = armor_selector_->selectBest(
          extra_positions, extra_center, extra_yaw,
          robot.num_armors, yaw_velocity,
          context.current_yaw, context.current_pitch);
        control_target_position = extra_selection.position;
        applied_selection = extra_selection;
        applied_armor_positions = std::move(extra_positions);
        applied_predicted_center = extra_center;
      }
    }
  }

  const Eigen::Vector3d target_velocity = linear_velocity;

  // 计算云台控制角度 (使用预测位置)
  double control_pitch, control_yaw, control_flight_time;
  if (!computeBallistic(control_target_position, target_velocity, context.bullet_speed,
                        control_pitch, control_yaw, control_flight_time))
  {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  // 应用手动补偿
  double pitch_offset_rad = pitch_offset_ * M_PI / 180.0;
  double yaw_offset_rad = yaw_offset_ * M_PI / 180.0;

  double cmd_pitch = control_pitch + pitch_offset_rad;
  double cmd_yaw = angles::normalize_angle(control_yaw + yaw_offset_rad);

  // 计算偏差
  double yaw_diff = angles::normalize_angle(cmd_yaw - context.current_yaw);
  double pitch_diff = cmd_pitch - context.current_pitch;

  // 策略层仅输出控制参数；开火建议由主循环统一计算。
  const double fire_compensation_s = trigger_to_muzzle_s_;
  bool fire_like = (state_ == TRACKING_CENTER);
  if (!fire_like) {
    constexpr double kFireLikeThreshold = 1.5 * M_PI / 180.0;
    fire_like =
      std::abs(yaw_diff) < kFireLikeThreshold &&
      std::abs(pitch_diff) < kFireLikeThreshold;
  }

  // 自适应 delay 更新（根据本帧 fire_advice 和目标速度）
  if (adaptive_delay_enabled_) {
    double v_linear  = linear_velocity.norm();
    double v_angular = std::abs(yaw_velocity);
    adaptive_ctrl_.update(fire_like, v_linear, v_angular);
  }

  // 构建控制命令
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = std::max(control_target_position.norm(), 0.0);

  DelayAuditSnapshot audit;
  audit.strategy_name = getName();
  audit.tracking = true;
  audit.processing_delay_s = processing_delay;
  audit.prediction_extra_s = std::max(prediction_delay_, 0.0);
  audit.flight_time_s = flight_time;
  audit.total_prediction_time_s = applied_prediction_time;
  audit.control_latency_s = applied_control_latency;
  audit.fire_control_compensation_s = fire_compensation_s;
  audit.control_delay_steps = 0;
  audit.uses_delayed_b = false;
  audit.double_compensation_risk = false;
  markDelayAuditValid(audit);

  last_control_target_debug_.valid = true;
  last_control_target_debug_.tracks_center =
    state_ == TRACKING_CENTER || applied_selection.is_center_fallback;
  last_control_target_debug_.is_virtual_target = applied_selection.is_virtual_target;
  last_control_target_debug_.selected_index = applied_selection.selected_index;
  last_control_target_debug_.real_selected_index = applied_selection.real_selected_index;
  last_control_target_debug_.prediction_time_s = applied_prediction_time;
  last_control_target_debug_.yaw_velocity = yaw_velocity;
  const bool uses_large_armor =
    robot.robot_type == rm_interfaces::msg::TrackedRobot::BALANCE_2 ||
    robot.robot_type == rm_interfaces::msg::TrackedRobot::HERO_4 ||
    robot.robot_type == rm_interfaces::msg::TrackedRobot::OUTPOST_3 ||
    robot.robot_type == rm_interfaces::msg::TrackedRobot::BASE;
  last_control_target_debug_.armor_width_m = uses_large_armor ? 0.230 : 0.135;
  last_control_target_debug_.armor_height_m = 0.055;
  last_control_target_debug_.current_center = current_center;
  last_control_target_debug_.predicted_center = applied_predicted_center;
  last_control_target_debug_.linear_velocity = linear_velocity;
  last_control_target_debug_.control_target_position = control_target_position;
  last_control_target_debug_.current_armor_positions = current_armor_positions;
  const auto current_armor_poses = position_calculator_->calculatePoses(robot);
  last_control_target_debug_.current_armor_normals.reserve(current_armor_poses.size());
  last_control_target_debug_.current_armor_width_axes.reserve(current_armor_poses.size());
  last_control_target_debug_.current_armor_height_axes.reserve(current_armor_poses.size());
  for (const auto & pose : current_armor_poses) {
    last_control_target_debug_.current_armor_normals.push_back(pose.normal);
    last_control_target_debug_.current_armor_width_axes.push_back(pose.width_axis);
    last_control_target_debug_.current_armor_height_axes.push_back(pose.height_axis);
  }
  last_control_target_debug_.predicted_armor_positions = std::move(applied_armor_positions);
  const int physical_index = applied_selection.real_selected_index >= 0 ?
    applied_selection.real_selected_index : applied_selection.selected_index;
  if (physical_index >= 0 &&
    physical_index < static_cast<int>(current_armor_positions.size()))
  {
    last_control_target_debug_.current_selected_armor = current_armor_positions[physical_index];
  }

  return cmd;
}

void PredictedPositionStrategy::setPredictionParameters(
  double prediction_delay,
  double max_prediction_time)
{
  prediction_delay_ = prediction_delay;
  max_prediction_time_ = max_prediction_time;
}

void PredictedPositionStrategy::setMaxProcessingDelay(double max_processing_delay)
{
  max_processing_delay_s_ = max_processing_delay > 0.0 ? max_processing_delay : 0.0;
}

void PredictedPositionStrategy::setManualOffset(double pitch_offset, double yaw_offset)
{
  pitch_offset_ = pitch_offset;
  yaw_offset_ = yaw_offset;
}

void PredictedPositionStrategy::setControllerDelay(double controller_delay)
{
  controller_delay_ = controller_delay;
}

void PredictedPositionStrategy::setTriggerToMuzzleDelay(double trigger_to_muzzle_s)
{
  trigger_to_muzzle_s_ = trigger_to_muzzle_s > 0.0 ? trigger_to_muzzle_s : 0.0;
}

void PredictedPositionStrategy::setAdaptiveDelayParams(
  bool enable,
  double initial_delay,
  double min_delay,
  double max_delay,
  double add_step,
  double mul_factor,
  int    fire_wait_threshold,
  double max_linear_speed,
  double max_angular_speed)
{
  adaptive_delay_enabled_ = enable;
  if (enable) {
    adaptive_ctrl_.init(
      initial_delay, min_delay, max_delay,
      add_step, mul_factor, fire_wait_threshold,
      max_linear_speed, max_angular_speed);
  }
}

void PredictedPositionStrategy::setTrackingCenterParams(double max_tracking_v_yaw, int transfer_thresh)
{
  max_tracking_v_yaw_ = max_tracking_v_yaw;
  transfer_thresh_ = transfer_thresh;
}

void PredictedPositionStrategy::setAimPlannerConfig(
  const AimTrajectoryPlannerConfig & cfg)
{
  aim_planner_.setConfig(cfg);
}

// ═══════════════════════════════════════════════════════════════════
// 自瞄轨迹规划器路径（轨迹视角，参考 sp_vision_25 §4）
//
// 以"射击轨迹"（目标轨迹提前 total_prediction_time ≈ 延迟+飞行时间）
// 为参考，参考中每步取相对当前云台角最近的装甲板——换板在参考中形成
// 突变三角波，规划器以角加速度为决策变量做 QP，在云台能力上限内把
// 突变平滑为可跟随的过渡段。平滑 ≠ 滞后：跟随段与射击轨迹完全重合。
// ═══════════════════════════════════════════════════════════════════
rm_interfaces::msg::GimbalCmd PredictedPositionStrategy::solveWithPlanner(
  const GimbalControlContext & context,
  const rm_interfaces::msg::TrackedRobot & robot,
  const Eigen::Vector3d & linear_velocity,
  double total_prediction_time, double processing_delay, double flight_time)
{
  const auto & pcfg = aim_planner_.cfg();
  const int H = pcfg.half_horizon;
  const int N = 2 * H;

  std::vector<double> ref_yaw(N, 0.0), ref_pitch(N, 0.0);
  double yaw0 = 0.0;
  Eigen::Vector3d follow_position = Eigen::Vector3d::Zero();
  for (int k = 0; k < N; ++k) {
    const double t_pred = total_prediction_time + (k - H) * pcfg.dt;
    auto plates = position_calculator_->calculatePredicted(robot, t_pred);
    if (plates.empty()) {
      markDelayAuditInvalid(getName(), true);
      return createIdleCmd();
    }
    // 该步参考：相对当前云台角最近的板的弹道解算角（射击轨迹）
    double best_ang = 1e9, best_yaw = 0.0, best_pitch = 0.0;
    Eigen::Vector3d best_pos = plates.front();
    for (const auto & p : plates) {
      double pitch = 0.0, yaw = 0.0, ft = 0.0;
      if (!computeBallistic(p, linear_velocity, context.bullet_speed,
                            pitch, yaw, ft)) {
        continue;
      }
      const double ang = std::hypot(
        angles::normalize_angle(yaw - context.current_yaw),
        pitch - context.current_pitch);
      if (ang < best_ang) {
        best_ang = ang;
        best_yaw = yaw;
        best_pitch = pitch;
        best_pos = p;
      }
    }
    ref_yaw[k] = best_yaw;
    ref_pitch[k] = best_pitch;
    if (k == H) {
      yaw0 = best_yaw;
      follow_position = best_pos;
    }
  }
  for (int k = 0; k < N; ++k) {
    ref_yaw[k] = angles::normalize_angle(ref_yaw[k] - yaw0);
  }

  auto plan = aim_planner_.plan(ref_yaw, ref_pitch);
  if (!plan.valid) {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  const double cmd_yaw = angles::normalize_angle(plan.yaw + yaw0);
  const double cmd_pitch = plan.pitch;
  const double yaw_diff = angles::normalize_angle(cmd_yaw - context.current_yaw);
  const double pitch_diff = cmd_pitch - context.current_pitch;

  rm_interfaces::msg::GimbalCmd cmd;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  // 规划轨迹前馈：与射击轨迹同源的平滑速度/加速度，供下位机跟随
  cmd.yaw_v = plan.yaw_rate * 180.0 / M_PI;
  cmd.pitch_v = plan.pitch_rate * 180.0 / M_PI;
  cmd.yaw_a = plan.yaw_acc * 180.0 / M_PI;
  cmd.pitch_a = plan.pitch_acc * 180.0 / M_PI;
  cmd.distance = std::max(follow_position.norm(), 0.0);

  DelayAuditSnapshot audit;
  audit.strategy_name = getName();
  audit.tracking = true;
  audit.processing_delay_s = processing_delay;
  audit.prediction_extra_s = std::max(prediction_delay_, 0.0);
  audit.flight_time_s = flight_time;
  audit.total_prediction_time_s = total_prediction_time;
  audit.control_latency_s = 0.0;
  audit.fire_control_compensation_s = trigger_to_muzzle_s_;
  audit.control_delay_steps = 0;
  audit.uses_delayed_b = false;
  audit.double_compensation_risk = false;
  markDelayAuditValid(audit);

  last_control_target_debug_.valid = true;
  last_control_target_debug_.tracks_center = false;
  last_control_target_debug_.is_virtual_target = false;
  last_control_target_debug_.selected_index = -1;
  last_control_target_debug_.real_selected_index = -1;
  last_control_target_debug_.prediction_time_s = total_prediction_time;
  last_control_target_debug_.control_target_position = follow_position;
  last_control_target_debug_.current_armor_positions =
    position_calculator_->calculate(robot);

  return cmd;
}

}  // namespace gimbal_controller
