#ifndef AUTO_BUFF_V2__NJUST_RUNE_AIMER_HPP
#define AUTO_BUFF_V2__NJUST_RUNE_AIMER_HPP

#include <memory>

#include "rune_aimer.hpp"

namespace auto_buff_v2
{
class NjustRuneAimer final : public RuneAimer
{
public:
  struct Config
  {
    double shoot_delay = 0.04;
    double rune_idle_duration = 0.4;
    double rune_shoot_duration = 0.2;
    double yaw_tolerance = 0.07;
    double pitch_tolerance = 0.04;

    static Config load(const YAML::Node & node);
  };

  NjustRuneAimer(BuffConfig::Planner config, Config njust_config);
  std::optional<RuneAim> aim(
    std::uint64_t generation, const std::optional<RuneEstimate> & target,
    double bullet_speed, const io::GimbalState & gimbal, Timestamp now) override;
  std::optional<AimSolution> aim_at(
    const RuneEstimate & target, Timestamp prediction_time, double bullet_speed) const override;
  bool fire_advice(
    const Eigen::Vector3d & rune_center, const Eigen::Vector3d & aimpoint,
    const auto_aim::Plan & plan, const io::GimbalState & gimbal, Timestamp now) override;

private:
  BuffConfig::Planner config_;
  Config njust_config_;
  std::unique_ptr<tools::BallisticSolver> ballistic_solver_;
  std::optional<Timestamp> attack_start_;
  std::optional<std::uint64_t> attack_generation_;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__NJUST_RUNE_AIMER_HPP
