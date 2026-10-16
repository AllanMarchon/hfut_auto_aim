#ifndef AUTO_BUFF_V2__RUNE_DETECTOR_FACTORY_HPP
#define AUTO_BUFF_V2__RUNE_DETECTOR_FACTORY_HPP

#include <memory>
#include <stdexcept>

#include "../buff_config.hpp"
#include "climber_rune_detector.hpp"
#include "njust_rune_detector.hpp"
#include "rune_detector.hpp"

namespace auto_buff_v2
{
inline std::unique_ptr<RuneDetector> make_rune_detector(const BuffConfig::Detector & config)
{
  if (config.type == "climber")
    return std::make_unique<ClimberRuneDetector>(config.parameters["climber"]);
  if (config.type == "njust") {
    auto detector = std::make_unique<NjustRuneDetector>();
    detector->config = NjustRuneDetector::Config::load(config.parameters["njust"]);
    detector->config.fx = config.fx;
    detector->config.fy = config.fy;
    return detector;
  }
  throw std::invalid_argument("unknown buff_v2 detector: " + config.type);
}
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__RUNE_DETECTOR_FACTORY_HPP
