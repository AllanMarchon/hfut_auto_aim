#ifndef AUTO_BUFF_V2__BUFF_PLANNER_FACTORY_HPP
#define AUTO_BUFF_V2__BUFF_PLANNER_FACTORY_HPP

#include <memory>
#include <stdexcept>
#include <string>

#include "../buff_config.hpp"
#include "buff_plan_strategy.hpp"
#include "mpc_buff_planner.hpp"
#include "njust_buff_planner.hpp"

namespace auto_buff_v2
{
inline std::unique_ptr<BuffPlanStrategy> make_buff_planner(
  const BuffConfig::Planner & config, const std::string & config_path)
{
  if (config.mode == "mpc") return std::make_unique<MpcBuffPlanner>(config, config_path);
  if (config.mode == "njust") return std::make_unique<NjustBuffPlanner>(config);
  throw std::invalid_argument("unknown buff_v2 planner mode: " + config.mode);
}
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__BUFF_PLANNER_FACTORY_HPP
