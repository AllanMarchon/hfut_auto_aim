#ifndef AUTO_BUFF_V2__RUNE_AIMER_FACTORY_HPP
#define AUTO_BUFF_V2__RUNE_AIMER_FACTORY_HPP

#include <memory>
#include <stdexcept>

#include "climber_rune_aimer.hpp"
#include "njust_rune_aimer.hpp"

namespace auto_buff_v2
{
inline std::unique_ptr<RuneAimer> make_rune_aimer(const BuffConfig::Planner & config)
{
  if (config.aimer == "njust")
    return std::make_unique<NjustRuneAimer>(
      config, NjustRuneAimer::Config::load(config.aimer_parameters["njust"]));
  if (config.aimer == "climber")
    return std::make_unique<ClimberRuneAimer>(
      config, ClimberRuneAimer::Config::load(config.aimer_parameters["climber"]));
  throw std::invalid_argument("unknown buff_v2 aimer: " + config.aimer);
}
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__RUNE_AIMER_FACTORY_HPP
