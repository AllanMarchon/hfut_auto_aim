#ifndef AUTO_BUFF_V2__RUNE_MODEL_FACTORY_HPP
#define AUTO_BUFF_V2__RUNE_MODEL_FACTORY_HPP

#include <memory>
#include <stdexcept>

#include "njust_rune_model.hpp"
#include "sp_rune_model.hpp"

namespace auto_buff_v2
{
inline std::unique_ptr<RuneModel> make_rune_model(
  const BuffConfig::Camera & camera, const BuffConfig::Model & config, bool big_rune)
{
  if (config.type == "njust") return std::make_unique<NjustRuneModel>(
    camera, NjustRuneModel::Config::load(config.parameters["njust"]), big_rune);
  if (config.type == "sp") return std::make_unique<SPRuneModel>(
    camera, SPRuneModel::Config::load(config.parameters["sp"]), big_rune);
  throw std::invalid_argument("unknown buff_v2 model: " + config.type);
}
}  // namespace auto_buff_v2

#endif
