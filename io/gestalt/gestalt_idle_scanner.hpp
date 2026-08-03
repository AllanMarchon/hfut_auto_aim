#ifndef HFUT_AUTO_AIM_IO_GESTALT_IDLE_SCANNER_HPP_
#define HFUT_AUTO_AIM_IO_GESTALT_IDLE_SCANNER_HPP_

#include <chrono>
#include <optional>

#include "hfut_auto_aim/gimbal_command.hpp"

namespace hfut::io {

struct GestaltIdleScanConfig {
  bool enabled = true;
  double yaw_rate_deg_s = 20.0;
  double pitch_rate_deg_s = 6.0;
  double pitch_limit_deg = 30.0;
  double activation_delay_s = 3.0;
  double max_step_s = 0.20;
};

class GestaltIdleScanner {
 public:
  using Clock = std::chrono::steady_clock;

  explicit GestaltIdleScanner(GestaltIdleScanConfig config = {});

  // Returns true when the scanner replaces the pipeline command. A raw visual
  // contact freezes at feedback pose until the tracker can take control.
  bool update(bool auto_aim_active, bool visual_contact,
              double feedback_yaw_rad, double feedback_pitch_rad,
              Clock::time_point now, GimbalCommand& command);

  void reset();
  bool scanning() const { return scanning_; }
  const GestaltIdleScanConfig& config() const { return config_; }

 private:
  void writeCommand(double feedback_yaw_rad, double feedback_pitch_rad,
                    double yaw_velocity_rad_s, double pitch_velocity_rad_s,
                    GimbalCommand& command) const;

  GestaltIdleScanConfig config_;
  bool scanning_ = false;
  int pitch_direction_ = -1;
  double target_yaw_rad_ = 0.0;
  double target_pitch_rad_ = 0.0;
  std::optional<Clock::time_point> inactive_since_;
  Clock::time_point last_step_{};
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_IO_GESTALT_IDLE_SCANNER_HPP_
