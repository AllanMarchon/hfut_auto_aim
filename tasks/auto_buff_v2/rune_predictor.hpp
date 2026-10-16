#ifndef AUTO_BUFF_V2__RUNE_PREDICTOR_HPP
#define AUTO_BUFF_V2__RUNE_PREDICTOR_HPP

#include <optional>

#include "models/rune_model.hpp"

namespace auto_buff_v2
{
class RunePredictor
{
public:
  RuneEstimate predict(const RuneEstimate & estimate, Timestamp prediction_time) const;
  std::optional<Eigen::Vector3d> aimpoint_at(const RuneEstimate & estimate,
                                              Timestamp prediction_time) const;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__RUNE_PREDICTOR_HPP
