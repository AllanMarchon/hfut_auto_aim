#ifndef AUTO_BUFF__MPC_PLANNER_HPP
#define AUTO_BUFF__MPC_PLANNER_HPP

#include <array>
#include <chrono>
#include <optional>
#include <string>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/planner/tinympc/tiny_api.hpp"
#include "buff_aimer.hpp"
#include "buff_target.hpp"

namespace auto_buff
{

// 小符专用控制覆盖项。未填写的字段继续使用 controller.yaml 的值。
// 速度字段在读取 YAML 后统一转换为弧度每秒，便于直接送入控制器。
struct SmallBuffMpcOverrides
{
  std::optional<double> max_yaw_acceleration_rad_s2;
  std::optional<double> max_pitch_acceleration_rad_s2;
  std::optional<double> yaw_error_gain;
  std::optional<double> pitch_error_gain;
  std::optional<double> max_yaw_velocity_rad_s;
  std::optional<double> max_pitch_velocity_rad_s;
};

SmallBuffMpcOverrides loadSmallBuffMpcOverrides(const std::string & buff_config);

// 小符 MPC 的一次规划结果。角度仍使用 Buff Aimer 的世界坐标约定，
// standard.cpp 会在串口边界统一处理 yaw/pitch 符号。
struct BuffMpcPlan
{
  auto_aim::Plan plan{};
  double distance = 0.0;
  bool valid = false;
};

// 用小符 EKF 的预测状态生成角度参考轨迹，再复用普通自瞄的 TinyMPC 二阶控制器。
class MpcPlanner
{
public:
  MpcPlanner(const std::string & controller_config, const std::string & buff_config);

  BuffMpcPlan plan(
    const SmallTarget & target, double bullet_speed,
    std::chrono::steady_clock::time_point timestamp,
    double current_yaw, double current_pitch);

  BuffMpcPlan planStaticPoint(
    const Eigen::Vector3d & point_world, double bullet_speed,
    double current_yaw, double current_pitch);

private:
  struct AimSample
  {
    double yaw = 0.0;
    double pitch = 0.0;
    double distance = 0.0;
    bool valid = false;
  };

  double yaw_offset_ = 0.0;
  double pitch_offset_ = 0.0;
  double predict_time_ = 0.12;

  TinySolver * yaw_solver_ = nullptr;
  TinySolver * pitch_solver_ = nullptr;
  bool warned_solver_status_ = false;

  AimSample aimAt(SmallTarget target, double prediction_time, double bullet_speed) const;
  AimSample aimAtStaticPoint(const Eigen::Vector3d & point_world, double bullet_speed) const;
  BuffMpcPlan solveSamples(
    const std::array<AimSample, auto_aim::HORIZON + 2> & samples,
    double current_yaw, double current_pitch);
  void setupYawSolver(
    const std::string & controller_config,
    const std::optional<double> & max_acceleration_override);
  void setupPitchSolver(
    const std::string & controller_config,
    const std::optional<double> & max_acceleration_override);
};

// 小符控制器的唯一入口：把轨迹规划、换叶片保护和开火节拍收在同一个
// 状态对象里，standard.cpp 不再同时拼接多套小符输出。
class SmallBuffController
{
public:
  SmallBuffController(
    const std::string & controller_config, const std::string & buff_config, Aimer & aimer);

  BuffMpcPlan update(
    const SmallTarget & target, double bullet_speed,
    std::chrono::steady_clock::time_point timestamp,
    double current_yaw, double current_pitch);

  BuffMpcPlan updateStaticPoint(
    const Eigen::Vector3d & point_world, double bullet_speed,
    double current_yaw, double current_pitch);

  void reset();

private:
  MpcPlanner planner_;
  Aimer & aimer_;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__MPC_PLANNER_HPP
