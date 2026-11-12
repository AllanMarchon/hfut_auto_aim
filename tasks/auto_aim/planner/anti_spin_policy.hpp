#ifndef AUTO_AIM__ANTI_SPIN_POLICY_HPP
#define AUTO_AIM__ANTI_SPIN_POLICY_HPP

#include <algorithm>
#include <cmath>

namespace auto_aim
{
class AntiSpinModeSelector
{
public:
  void configure(bool enabled, double enter_speed_rad_s, int enter_frames,
                 int exit_frames, double exit_hysteresis_rad_s)
  {
    enabled_ = enabled;
    enter_speed_rad_s_ = std::max(enter_speed_rad_s, 0.0);
    enter_frames_ = std::max(enter_frames, 1);
    exit_frames_ = std::max(exit_frames, 1);
    exit_hysteresis_rad_s_ = std::max(exit_hysteresis_rad_s, 0.0);
    reset();
  }

  bool update(double signed_speed_rad_s, bool supported_target)
  {
    if (!enabled_ || !supported_target || !std::isfinite(signed_speed_rad_s)) {
      reset();
      return false;
    }

    const double speed = std::abs(signed_speed_rad_s);
    if (!active_) {
      exit_count_ = 0;
      enter_count_ = speed >= enter_speed_rad_s_ ? enter_count_ + 1 : 0;
      if (enter_count_ >= enter_frames_) {
        active_ = true;
        enter_count_ = 0;
      }
    } else {
      enter_count_ = 0;
      const double exit_speed = std::max(0.0, enter_speed_rad_s_ - exit_hysteresis_rad_s_);
      exit_count_ = speed <= exit_speed ? exit_count_ + 1 : 0;
      if (exit_count_ >= exit_frames_) {
        active_ = false;
        exit_count_ = 0;
      }
    }
    return active_;
  }

  bool active() const { return active_; }

  void reset()
  {
    active_ = false;
    enter_count_ = 0;
    exit_count_ = 0;
  }

private:
  bool enabled_{true};
  bool active_{false};
  double enter_speed_rad_s_{7.0};
  double exit_hysteresis_rad_s_{1.0};
  int enter_frames_{3};
  int exit_frames_{5};
  int enter_count_{0};
  int exit_count_{0};
};

inline double antiSpinAngularError(double lhs_rad, double rhs_rad)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::abs(std::remainder(lhs_rad - rhs_rad, kTwoPi));
}

inline bool antiSpinShotCandidate(
  double armor_facing_yaw_rad, double target_center_yaw_rad,
  double armor_ray_yaw_rad, double guard_yaw_rad,
  double armor_pitch_rad, double guard_pitch_rad,
  double front_face_tolerance_rad, double yaw_hit_tolerance_rad,
  double pitch_hit_tolerance_rad)
{
  return antiSpinAngularError(armor_facing_yaw_rad, target_center_yaw_rad) <=
           front_face_tolerance_rad &&
         antiSpinAngularError(armor_ray_yaw_rad, guard_yaw_rad) <= yaw_hit_tolerance_rad &&
         std::abs(armor_pitch_rad - guard_pitch_rad) <= pitch_hit_tolerance_rad;
}
}  // namespace auto_aim

#endif  // AUTO_AIM__ANTI_SPIN_POLICY_HPP
