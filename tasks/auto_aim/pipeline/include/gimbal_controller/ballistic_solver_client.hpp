// ROS-free stub for the ballistic solver service client.
//
// The original called a ROS2 `SolveBallistic` service. In the ros-free sim
// port we run with `controller.ballistic_mode: "local"`, so the controller
// always uses its in-process LocalTrajectoryCompensator. This stub reports the
// service as unavailable; every call site is guarded by isServiceAvailable(),
// so solve() is never reached. Keeping the type (and BallisticResult) lets the
// gimbal_controller sources compile unmodified.
#ifndef GIMBAL_CONTROLLER__BALLISTIC_SOLVER_CLIENT_HPP_
#define GIMBAL_CONTROLLER__BALLISTIC_SOLVER_CLIENT_HPP_

#include <chrono>
#include <string>

#include <Eigen/Dense>

namespace gimbal_controller {

struct BallisticResult {
  double pitch = 0.0;
  double yaw = 0.0;
  double flight_time = 0.0;
  bool success = false;
  std::string message;
};

class BallisticSolverClient {
 public:
  BallisticSolverClient() = default;
  ~BallisticSolverClient() = default;

  BallisticResult solve(
      const Eigen::Vector3d& /*target_position*/,
      const Eigen::Vector3d& /*target_velocity*/,
      double /*bullet_speed*/,
      std::chrono::milliseconds /*timeout*/ = std::chrono::milliseconds(50)) {
    return BallisticResult{};  // success=false; never called (guarded)
  }

  bool isServiceAvailable() const { return false; }
  void setServiceName(const std::string& service_name) { service_name_ = service_name; }

 private:
  std::string service_name_{"/ballistic_solver/solve"};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__BALLISTIC_SOLVER_CLIENT_HPP_
