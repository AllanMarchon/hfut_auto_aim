#ifndef AUTO_BUFF_V2__CLIMBER_RUNE_AIMER_HPP
#define AUTO_BUFF_V2__CLIMBER_RUNE_AIMER_HPP

#include "rune_aimer.hpp"

namespace auto_buff_v2
{
class ClimberRuneAimer final : public RuneAimer
{
public:
  struct Config
  {
    double fire_gap_time = 0.7;
    double predict_time = 0.12;
    static Config load(const YAML::Node & node);
  };

  ClimberRuneAimer(BuffConfig::Planner config, Config climber_config);
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
  Config climber_config_;
  std::unique_ptr<tools::BallisticSolver> ballistic_solver_;
  std::optional<std::uint64_t> generation_;
  Timestamp last_fire_{};
  Eigen::Vector2d last_angles_ = Eigen::Vector2d::Zero();
  int mistake_count_ = 0;
  bool switch_fanblade_ = false;
  bool first_in_aimer_ = true;
  bool predicted_ = false;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__CLIMBER_RUNE_AIMER_HPP
