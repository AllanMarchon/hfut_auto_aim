#ifndef MAX_ENTROPY_TRACKER_UTILS_TRACKING_OUTPUT_POLICY_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_TRACKING_OUTPUT_POLICY_HPP_

#include <cmath>
#include <optional>

namespace fyt::auto_aim {

inline bool temporaryPredictionIsFresh(
    std::optional<double> observation_age_s, double max_age_s) {
  return observation_age_s.has_value() &&
      std::isfinite(*observation_age_s) && *observation_age_s >= 0.0 &&
      std::isfinite(max_age_s) && max_age_s >= 0.0 &&
      *observation_age_s <= max_age_s;
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_TRACKING_OUTPUT_POLICY_HPP_
