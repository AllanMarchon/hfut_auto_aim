#include <cmath>
#include <cstdio>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/vehicle/backends/vehicle_ukf_backend_v1.hpp"

namespace {

int fail(const char* message, int code) {
  std::fprintf(stderr, "vehicle rotation witness test failed: %s\n", message);
  return code;
}

}  // namespace

int main() {
  fyt::auto_aim::UnifiedConfig config;
  fyt::auto_aim::ObservationData seed;
  seed.x = 3.2;
  seed.y = 0.0;
  seed.z = 0.2;
  seed.yaw = 0.0;

  fyt::auto_aim::vehicle::VehicleUkfBackendV1 backend(config, 0.032);
  backend.reset(seed, 0, 0.20, 0.17, 0.03);
  const int yaw_rate = backend.state_idx().YAW_RATE();
  backend.x()(yaw_rate) = -8.0;

  // Independent radial-yaw evidence says +8 rad/s. A magnitude-only conflict
  // check cannot distinguish this from the incorrect -8 rad/s state.
  double angle = 0.0;
  double time = 1.0;
  for (int sample = 0; sample < 40; ++sample) {
    backend.noteArmorAngle(angle, time);
    angle = std::remainder(angle + 8.0 * 0.032, 2.0 * M_PI);
    time += 0.032;
  }
  if (backend.x()(yaw_rate) <= 2.0) {
    return fail("opposite-sign yaw rate was not corrected", 1);
  }

  // A physical plate transition is close to pi/2 and must reset rather than
  // inject an aliased instantaneous rate.
  const double before_switch = backend.x()(yaw_rate);
  backend.noteArmorAngle(std::remainder(angle + M_PI / 2.0, 2.0 * M_PI), time);
  if (std::abs(backend.x()(yaw_rate) - before_switch) > 1e-9) {
    return fail("plate switch changed the yaw-rate state", 2);
  }

  // A vision plate may only remain associated for six frames at speed. The
  // EMA must initialize from the first derivative instead of spending the
  // entire visible window ramping up from an artificial zero sample.
  fyt::auto_aim::vehicle::VehicleUkfBackendV1 short_window(config, 0.032);
  short_window.reset(seed, 0, 0.20, 0.17, 0.03);
  angle = 0.0;
  time = 1.0;
  for (int sample = 0; sample < 6; ++sample) {
    short_window.noteArmorAngle(angle, time);
    angle = std::remainder(angle + 4.0 * 0.032, 2.0 * M_PI);
    time += 0.032;
  }
  if (short_window.x()(yaw_rate) < 1.5) {
    return fail("short associated plate window did not seed yaw rate", 3);
  }

  fyt::auto_aim::vehicle::VehicleUkfBackendV1 alternating(config, 0.032);
  alternating.reset(seed, 0, 0.20, 0.17, 0.03);
  angle = 0.0;
  time = 1.0;
  for (int sample = 0; sample < 12; ++sample) {
    alternating.noteArmorAngle(angle, time);
    const double rate = sample % 2 == 0 ? 4.0 : -4.0;
    angle = std::remainder(angle + rate * 0.032, 2.0 * M_PI);
    time += 0.032;
  }
  if (std::abs(alternating.x()(yaw_rate)) > 1e-9) {
    return fail("alternating angle jitter passed direction coherence gate", 4);
  }
  return 0;
}
