#include <cmath>
#include <cstdio>

#include "gimbal_controller/gimbal_cmd_filter.hpp"

namespace {

bool near(double left, double right, double tolerance = 1e-9) {
  return std::abs(left - right) <= tolerance;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "gimbal command filter test failed: %s\n", message);
  return code;
}

rm_interfaces::msg::GimbalCmd command(double yaw, double velocity, double acceleration) {
  rm_interfaces::msg::GimbalCmd result;
  result.yaw = yaw;
  result.yaw_diff = yaw;
  result.yaw_v = velocity;
  result.yaw_a = acceleration;
  return result;
}

}  // namespace

int main() {
  gimbal_controller::GimbalCmdFilterConfig config;
  config.enable_outlier_rejection = false;
  config.enable_rate_limiter = false;

  gimbal_controller::GimbalCmdFilter mpc_filter(config);
  auto mpc = command(1.0, 2.5, -12.0);
  mpc_filter.filter(mpc, true);
  if (!near(mpc.yaw_v, 2.5) || !near(mpc.yaw_a, -12.0)) {
    return fail("MPC state derivatives were overwritten", 1);
  }

  gimbal_controller::GimbalCmdFilter position_filter(config);
  auto position = command(1.0, 2.5, -12.0);
  position_filter.filter(position, false);
  if (!near(position.yaw_v, 0.0) || !near(position.yaw_a, 0.0)) {
    return fail("position-mode first frame did not clear synthetic derivatives", 2);
  }

  config.enable_outlier_rejection = true;
  config.outlier_threshold_yaw = 1.0;
  config.max_outlier_count = 3;
  gimbal_controller::GimbalCmdFilter guarded_filter(config);
  auto accepted = command(0.0, 1.0, 4.0);
  guarded_filter.filter(accepted, true);
  auto rejected = command(10.0, 5.0, 20.0);
  guarded_filter.filter(rejected, true);
  if (!near(rejected.yaw, accepted.yaw) ||
      !near(rejected.yaw_v, accepted.yaw_v) || !near(rejected.yaw_a, 0.0)) {
    return fail("outlier hold did not suppress MPC acceleration", 3);
  }
  return 0;
}
