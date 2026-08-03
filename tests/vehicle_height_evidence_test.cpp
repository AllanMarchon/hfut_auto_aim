#include <cmath>
#include <cstdio>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/vehicle/backends/vehicle_ukf_backend_v1.hpp"
#include "max_entropy_tracker/utils/dual_height_evidence.hpp"

namespace {

using fyt::auto_aim::ObservationData;
using fyt::auto_aim::UnifiedConfig;
using fyt::auto_aim::selectDualHeightEvidence;
using fyt::auto_aim::vehicle::VehicleUkfBackendV1;

bool near(double left, double right, double tolerance = 1e-9) {
  return std::abs(left - right) <= tolerance;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "vehicle height evidence test failed: %s\n", message);
  return code;
}

ObservationData armor(double z, double yaw) {
  ObservationData observation;
  observation.x = 3.0 + 0.2 * std::cos(yaw);
  observation.y = 0.2 * std::sin(yaw);
  observation.z = z;
  observation.yaw = yaw;
  return observation;
}

}  // namespace

int main() {
  const std::vector<ObservationData> adjacent{
      armor(0.97, 0.0), armor(1.03, M_PI / 2.0)};
  const auto positive = selectDualHeightEvidence(adjacent, 0.0);
  if (!positive || !near(*positive, 0.03)) {
    return fail("adjacent plates did not produce +3cm evidence", 1);
  }

  // A 90-degree phase shift is the equivalent representation with r1/r2
  // swapped and dza negated; the physical layout has not changed.
  const auto phase_shifted = selectDualHeightEvidence(adjacent, M_PI / 2.0);
  if (!phase_shifted || !near(*phase_shifted, -0.03)) {
    return fail("phase-shifted layout did not flip signed dza", 2);
  }

  const std::vector<ObservationData> same_parity{
      armor(0.97, 0.0), armor(0.97, M_PI)};
  if (selectDualHeightEvidence(same_parity, 0.0)) {
    return fail("same-height opposite plates produced false evidence", 3);
  }

  const std::vector<ObservationData> bad_phase{
      armor(0.97, 0.70), armor(1.03, M_PI / 2.0 + 0.70)};
  if (selectDualHeightEvidence(bad_phase, 0.0)) {
    return fail("poor phase binding passed the residual gate", 4);
  }

  UnifiedConfig config;
  auto& ukf = config.vehicle_tracker.ukf_v1;
  ukf.posterior_sanity.min_dza = -0.15;
  ukf.posterior_sanity.max_dza = 0.15;
  ukf.posterior_sanity.max_dza_jump = 0.03;
  ukf.posterior_sanity.max_vertical_center_jump = 0.04;
  ukf.dual_height_evidence_window = 5;
  ukf.dual_height_evidence_min_samples = 3;
  ukf.dual_height_evidence_gain = 1.0;
  ukf.max_vertical_speed = 0.5;
  ukf.max_vertical_acceleration = 4.0;

  VehicleUkfBackendV1 backend(config, 0.05);
  backend.reset(adjacent.front(), 0, 0.20, 0.17, 0.0);
  backend.noteDualHeightEvidence(0.030);
  backend.noteDualHeightEvidence(0.150);  // one-frame outlier
  backend.noteDualHeightEvidence(0.031);
  backend.noteDualHeightEvidence(0.031);
  if (!near(backend.get_dza(), 0.031, 1e-6)) {
    return fail("window median did not reject a height outlier", 5);
  }

  const auto& index = backend.state_idx();
  backend.x()(index.VZ()) = 2.0;
  backend.x()(index.AZ()) = -9.0;
  backend.predict(0.1);
  if (std::abs(backend.x()(index.VZ())) > ukf.max_vertical_speed + 1e-9 ||
      std::abs(backend.x()(index.AZ())) > ukf.max_vertical_acceleration + 1e-9) {
    return fail("vertical state constraints were not applied", 6);
  }

  backend.reset(adjacent.front(), 0, 0.20, 0.17, 0.03);
  backend.P()(index.Z(), index.Z()) = 1.0;
  ObservationData phase_jump = adjacent.front();
  phase_jump.z += 0.06;
  const auto rejected =
      backend.tryUpdateSingle(backend.buildPredictContext(), phase_jump, 0);
  if (rejected.success ||
      rejected.reject_reason.find("posterior_vertical_center_jump") ==
          std::string::npos) {
    return fail("single-plate phase height jump passed posterior sanity", 7);
  }

  ObservationData continuous_height = adjacent.front();
  continuous_height.z += 0.02;
  const auto accepted = backend.tryUpdateSingle(
      backend.buildPredictContext(), continuous_height, 0);
  if (!accepted.success) {
    return fail("continuous center height correction was rejected", 8);
  }
  return 0;
}
