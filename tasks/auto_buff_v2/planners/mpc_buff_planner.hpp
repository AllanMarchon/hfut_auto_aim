#ifndef AUTO_BUFF_V2__MPC_BUFF_PLANNER_HPP
#define AUTO_BUFF_V2__MPC_BUFF_PLANNER_HPP

#include <string>

#include "buff_plan_strategy.hpp"
#include "mpc_buff_tracking.hpp"

namespace auto_buff_v2
{
class MpcBuffPlanner final : public BuffPlanStrategy
{
public:
  MpcBuffPlanner(const BuffConfig::Planner & config, const std::string & config_path);
  auto_aim::Plan plan(
    std::uint64_t target_generation, const std::optional<RuneEstimate> & target,
    double bullet_speed, const io::GimbalState & gimbal, Timestamp now) override;

private:
  auto_aim::Planner planner_;
  MpcBuffTracking tracking_;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__MPC_BUFF_PLANNER_HPP
