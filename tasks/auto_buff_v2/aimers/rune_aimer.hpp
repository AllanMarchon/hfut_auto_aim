#ifndef AUTO_BUFF_V2__RUNE_AIMER_HPP
#define AUTO_BUFF_V2__RUNE_AIMER_HPP

#include <cstdint>
#include <optional>

#include "../models/rune_model.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace auto_buff_v2
{
struct AimSolution
{
  Eigen::Vector2d angles;
  double fly_time;
  Eigen::Vector3d point;
};

struct RuneAim
{
  auto_aim::Plan plan;
  AimSolution solution;  // 保留 double 精度，供 MPC 参考轨迹使用。
  Timestamp prediction_time;
  double bullet_speed;
};

inline Timestamp offset_time(Timestamp time, double seconds)
{
  return time + std::chrono::duration_cast<Timestamp::duration>(
                   std::chrono::duration<double>(seconds));
}

class RuneAimer
{
public:
  virtual ~RuneAimer() = default;
  // 返回拦截解和直接跟踪命令；返回 nullopt 表示当前没有可用解。
  virtual std::optional<RuneAim> aim(
    std::uint64_t generation, const std::optional<RuneEstimate> & target,
    double bullet_speed, const io::GimbalState & gimbal, Timestamp now) = 0;
  // 在固定时刻采样 MPC 参考轨迹，不推进瞄准器内部状态。
  virtual std::optional<AimSolution> aim_at(
    const RuneEstimate & target, Timestamp prediction_time, double bullet_speed) const = 0;
  virtual bool fire_advice(
    const Eigen::Vector3d & rune_center, const Eigen::Vector3d & aimpoint,
    const auto_aim::Plan & plan, const io::GimbalState & gimbal, Timestamp now) = 0;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__RUNE_AIMER_HPP
