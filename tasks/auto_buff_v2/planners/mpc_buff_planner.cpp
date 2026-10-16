#include "mpc_buff_planner.hpp"

namespace auto_buff_v2
{
MpcBuffPlanner::MpcBuffPlanner(const BuffConfig::Planner & config, const std::string & path)
: planner_(path), tracking_(config)
{
}

auto_aim::Plan MpcBuffPlanner::plan(
  std::uint64_t generation, const std::optional<RuneEstimate> & target, double speed,
  const io::GimbalState & gimbal, Timestamp now)
{
  const auto request = tracking_.prepare(generation, target, speed, now, gimbal);
  if (!request) return {};
  auto result = planner_.plan(request->trajectory, request->yaw0, request->distance);
  result.fire = tracking_.fire_advice(*request, result, gimbal, now);
  return result;
}
}  // namespace auto_buff_v2
