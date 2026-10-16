#ifndef AUTO_BUFF_V2__NJUST_BUFF_PLANNER_HPP
#define AUTO_BUFF_V2__NJUST_BUFF_PLANNER_HPP

#include <memory>

#include "../aimers/rune_aimer.hpp"
#include "../buff_config.hpp"
#include "buff_plan_strategy.hpp"

namespace auto_buff_v2
{
class NjustBuffPlanner final : public BuffPlanStrategy
{
public:
  explicit NjustBuffPlanner(BuffConfig::Planner config);
  auto_aim::Plan plan(
    std::uint64_t target_generation, const std::optional<RuneEstimate> & target,
    double bullet_speed, const io::GimbalState & gimbal, Timestamp now) override;

private:
  std::unique_ptr<RuneAimer> aimer_;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__NJUST_BUFF_PLANNER_HPP
