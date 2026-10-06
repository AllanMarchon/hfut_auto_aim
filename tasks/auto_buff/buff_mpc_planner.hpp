#ifndef AUTO_BUFF__MPC_PLANNER_HPP
#define AUTO_BUFF__MPC_PLANNER_HPP

#include <chrono>
#include <string>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/planner/tinympc/tiny_api.hpp"
#include "buff_target.hpp"

namespace auto_buff
{

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
    std::chrono::steady_clock::time_point timestamp, bool fire_advice);

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
  void setupYawSolver(const std::string & controller_config);
  void setupPitchSolver(const std::string & controller_config);
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__MPC_PLANNER_HPP
