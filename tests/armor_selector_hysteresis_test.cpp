// Temporal hysteresis test for ArmorSelector::selectByMinMovementWithFacing.
//
// Regression cover for the "abnormal armor switching" symptom: when the
// predicted armor set jitters (whole-vehicle observation noise), the min-
// movement argmin used to flip between adjacent plates every frame. The
// selector must now hold the previous plate unless a competitor is better by
// more than switch_movement_margin_.

#include "gimbal_controller/armor_selector.hpp"

#include <cstdio>

namespace {

int fail(const char* message, int code) {
  std::fprintf(stderr, "%s\n", message);
  return code;
}

}  // namespace

int main() {
  gimbal_controller::ArmorSelector selector;
  // Disable the facing/distance side effects: 180 deg thresholds pass every
  // plate; the third "far" plate is always the farthest and gets excluded by
  // the distance filter, leaving plates 0/1 as the two competitors.
  selector.setFacingParameters(180.0, 180.0);
  // The selector now blends facing thresholds with spin rate; pin the
  // low-spin end open as well so the hysteresis logic is what gets tested.
  selector.setLowSpinFacingParameters(180.0, 180.0);
  selector.setSwitchMovementMargin(0.005);

  const Eigen::Vector3d target_center(5.2, 0.0, 0.0);
  const Eigen::Vector3d far_plate(5.2, 1.5, 0.0);

  auto select = [&](double plate0_y, double plate1_y) {
    const std::vector<Eigen::Vector3d> positions = {
        {5.2, plate0_y, 0.0}, {5.2, plate1_y, 0.0}, far_plate};
    return selector.selectByMinMovementWithFacing(
        positions, target_center, 0.0, 4, 0.0, 0.0);
  };

  // Frame 1: plate 1 is genuinely closer to the aim axis -> selected.
  const auto frame1 = select(+0.0347, -0.030);
  if (frame1.selected_index != 1) {
    return fail("frame1: the better plate was not selected", 1);
  }

  // Frame 2: plate 0 becomes marginally better (movement difference ~4e-6
  // rad^2, far below the 0.005 margin) -> hysteresis must hold plate 1.
  const auto frame2 = select(+0.029, -0.031);
  if (frame2.selected_index != 1) {
    return fail("frame2: hysteresis failed to hold the previous plate", 2);
  }

  // Frame 3: plate 0 becomes clearly better (movement difference ~0.006
  // rad^2, above the margin) -> the selector must switch.
  const auto frame3 = select(+0.001, -0.40);
  if (frame3.selected_index != 0) {
    return fail("frame3: hysteresis blocked a genuine plate switch", 3);
  }
  return 0;
}
