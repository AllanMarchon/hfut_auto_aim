#ifndef AUTO_BUFF__RUNE_PREDICTOR_HPP
#define AUTO_BUFF__RUNE_PREDICTOR_HPP

#include <optional>

#include "models/rune_model.hpp"

namespace auto_buff
{
class RunePredictor
{
public:
  RuneEstimate predict(const RuneEstimate & estimate, Timestamp prediction_time) const;
  std::optional<Eigen::Vector3d> aimpoint_at(const RuneEstimate & estimate,
                                              Timestamp prediction_time) const;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_PREDICTOR_HPP
