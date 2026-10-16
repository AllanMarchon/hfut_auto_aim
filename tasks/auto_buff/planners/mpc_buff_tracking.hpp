#ifndef AUTO_BUFF__MPC_BUFF_TRACKING_HPP
#define AUTO_BUFF__MPC_BUFF_TRACKING_HPP

#include <cstdint>
#include <memory>
#include <optional>

#include "../aimers/rune_aimer.hpp"
#include "../buff_config.hpp"
#include "../models/rune_model.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace auto_buff
{
struct MpcBuffTrackingRequest
{
  auto_aim::Trajectory trajectory;
  double yaw0 = 0;
  double distance = -1;
  double fly_time = 0;
  Eigen::Vector3d rune_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d aimpoint = Eigen::Vector3d::Zero();
};

class MpcBuffTracking
{
public:
  using Config = BuffConfig::Planner;

  explicit MpcBuffTracking(Config config);
  std::optional<MpcBuffTrackingRequest> prepare(
    std::uint64_t target_generation, const std::optional<RuneEstimate> & target,
    double bullet_speed, Timestamp now, const io::GimbalState & gimbal = {});
  bool fire_advice(
    const MpcBuffTrackingRequest & request, const auto_aim::Plan & plan,
    const io::GimbalState & gimbal, Timestamp now);

private:
  std::unique_ptr<RuneAimer> aimer_;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__MPC_BUFF_TRACKING_HPP
