// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_UTILS_DUAL_HEIGHT_EVIDENCE_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_DUAL_HEIGHT_EVIDENCE_HPP_

#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

// Same-frame plates share the vehicle center height, so an adjacent-parity
// pair directly observes dza without attributing the alternating plate height
// to center Z motion. The sign follows the tracker's current 90-degree phase;
// shifting that phase by one panel legitimately flips dza and swaps r1/r2.
inline std::optional<double> selectDualHeightEvidence(
    const std::vector<ObservationData>& observations,
    double center_yaw,
    double max_phase_residual = 0.60) {
  if (observations.size() < 2 || !std::isfinite(center_yaw)) {
    return std::nullopt;
  }

  const auto bind_panel = [center_yaw](double armor_yaw) {
    const double phase = normalize_angle(armor_yaw - center_yaw);
    int panel = static_cast<int>(std::llround(phase / (M_PI / 2.0)));
    panel = ((panel % 4) + 4) % 4;
    const double residual = std::abs(normalize_angle(
        armor_yaw - center_yaw - panel * (M_PI / 2.0)));
    return std::pair<int, double>{panel, residual};
  };

  double largest_height_difference = -1.0;
  std::optional<double> selected;
  for (size_t first = 0; first < observations.size(); ++first) {
    for (size_t second = first + 1; second < observations.size(); ++second) {
      const auto [first_panel, first_residual] =
          bind_panel(observations[first].yaw);
      const auto [second_panel, second_residual] =
          bind_panel(observations[second].yaw);
      if (first_residual > max_phase_residual ||
          second_residual > max_phase_residual ||
          (first_panel % 2) == (second_panel % 2)) {
        continue;
      }

      const double height_difference =
          std::abs(observations[first].z - observations[second].z);
      if (height_difference <= largest_height_difference) {
        continue;
      }
      const double first_sign = (first_panel % 2 == 0) ? -1.0 : 1.0;
      const double second_sign = (second_panel % 2 == 0) ? -1.0 : 1.0;
      selected = (observations[first].z - observations[second].z) /
                 (first_sign - second_sign);
      largest_height_difference = height_difference;
    }
  }
  return selected;
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_DUAL_HEIGHT_EVIDENCE_HPP_
