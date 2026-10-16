#ifndef AUTO_BUFF_V2__BUFF_PLAN_STRATEGY_HPP
#define AUTO_BUFF_V2__BUFF_PLAN_STRATEGY_HPP

#include <cstdint>
#include <optional>

#include "../models/rune_model.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace auto_buff_v2
{
class BuffPlanStrategy
{
public:
  virtual ~BuffPlanStrategy() = default;
  virtual auto_aim::Plan plan(
    std::uint64_t target_generation, const std::optional<RuneEstimate> & target,
    double bullet_speed, const io::GimbalState & gimbal, Timestamp now) = 0;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__BUFF_PLAN_STRATEGY_HPP
