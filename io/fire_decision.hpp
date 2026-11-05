#ifndef HFUT_AUTO_AIM_IO_FIRE_DECISION_HPP_
#define HFUT_AUTO_AIM_IO_FIRE_DECISION_HPP_

#include <algorithm>
#include <cmath>
#include <string>

namespace hfut::io {

struct FireDecisionConfig {
  bool enable_alignment_gate{true};
  bool require_tracking{true};
  double yaw_tolerance_rad{0.0};
  double pitch_tolerance_rad{0.0};
  int min_aligned_frames{1};
};

struct FireDecisionInput {
  bool mpc_candidate{false};
  bool fire_enabled{false};
  bool control_valid{false};
  bool target_valid{false};
  bool tracking{false};
  bool feedback_fresh{false};
  bool fire_delay_valid{false};
  double aim_yaw_rad{0.0};
  double aim_pitch_rad{0.0};
  double feedback_yaw_rad{0.0};
  double feedback_pitch_rad{0.0};
};

struct FireDecision {
  bool allowed{false};
  bool blocked{false};
  double yaw_error_rad{0.0};
  double pitch_error_rad{0.0};
  int aligned_frames{0};
  std::string reason{"no_candidate"};
};

inline FireDecision evaluateFireDecision(const FireDecisionConfig& config,
                                         const FireDecisionInput& input,
                                         int& aligned_frames) {
  FireDecision result;
  const bool angles_finite = std::isfinite(input.aim_yaw_rad) &&
                             std::isfinite(input.aim_pitch_rad) &&
                             std::isfinite(input.feedback_yaw_rad) &&
                             std::isfinite(input.feedback_pitch_rad);
  if (angles_finite) {
    constexpr double kTwoPi = 6.28318530717958647692;
    result.yaw_error_rad = std::abs(std::remainder(
        input.aim_yaw_rad - input.feedback_yaw_rad, kTwoPi));
    result.pitch_error_rad = std::abs(input.aim_pitch_rad - input.feedback_pitch_rad);
  }

  const bool aligned = angles_finite &&
      result.yaw_error_rad <= config.yaw_tolerance_rad &&
      result.pitch_error_rad <= config.pitch_tolerance_rad;
  const bool fire_preconditions_met = input.mpc_candidate && input.fire_enabled &&
      input.control_valid && input.target_valid &&
      (!config.require_tracking || input.tracking) && input.feedback_fresh &&
      input.fire_delay_valid && angles_finite;
  if (fire_preconditions_met && (!config.enable_alignment_gate || aligned)) {
    aligned_frames = std::min(aligned_frames + 1, std::max(config.min_aligned_frames, 1));
  } else {
    aligned_frames = 0;
  }
  result.aligned_frames = aligned_frames;

  if (!input.mpc_candidate) {
    result.reason = "mpc_no_candidate";
  } else if (!input.fire_enabled) {
    result.reason = "fire_disabled";
  } else if (!input.control_valid) {
    result.reason = "no_control";
  } else if (!input.target_valid) {
    result.reason = "no_target";
  } else if (config.require_tracking && !input.tracking) {
    result.reason = "target_not_tracking";
  } else if (!input.feedback_fresh) {
    result.reason = "feedback_stale";
  } else if (!input.fire_delay_valid) {
    result.reason = "fire_delay_unavailable";
  } else if (!angles_finite) {
    result.reason = "invalid_angle";
  } else if (config.enable_alignment_gate && !aligned) {
    result.reason = "not_aligned";
  } else if (config.enable_alignment_gate &&
             aligned_frames < std::max(config.min_aligned_frames, 1)) {
    result.reason = "aligning";
  } else {
    result.allowed = true;
    result.reason = "allowed";
  }

  result.blocked = input.mpc_candidate && !result.allowed;
  return result;
}

}  // 命名空间 hfut::io

#endif  // 头文件保护结束
