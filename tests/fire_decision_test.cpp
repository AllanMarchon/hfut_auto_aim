#include "io/fire_decision.hpp"

#include <cmath>
#include <cstdio>

namespace {
bool check(bool condition, const char* message) {
  if (condition) return true;
  std::fprintf(stderr, "测试失败：%s\n", message);
  return false;
}
}  // 命名空间

int main() {
  hfut::io::FireDecisionConfig config;
  config.yaw_tolerance_rad = 0.25 * 3.14159265358979323846 / 180.0;
  config.pitch_tolerance_rad = 0.25 * 3.14159265358979323846 / 180.0;

  hfut::io::FireDecisionInput input;
  input.mpc_candidate = true;
  input.fire_enabled = true;
  input.control_valid = true;
  input.target_valid = true;
  input.tracking = true;
  input.feedback_fresh = true;
  input.fire_delay_valid = true;

  int aligned_frames = 0;
  input.aim_yaw_rad = 0.2 * 3.14159265358979323846 / 180.0;
  input.aim_pitch_rad = -0.2 * 3.14159265358979323846 / 180.0;
  auto result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(result.allowed, "门限内且候选有效时应允许开火") ||
      !check(!result.blocked, "允许开火时不应标记拦截")) return 1;

  input.aim_yaw_rad = 0.3 * 3.14159265358979323846 / 180.0;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(!result.allowed, "超出 yaw 门限时必须拦截") ||
      !check(result.blocked, "MPC 候选被拦截时应标记拦截") ||
      !check(result.reason == "not_aligned", "超门限时应报告 not_aligned") ||
      !check(result.aligned_frames == 0, "超门限后连续对准帧数应清零")) return 1;

  input.aim_yaw_rad = -3.14159265358979323846 + 0.001;
  input.feedback_yaw_rad = 3.14159265358979323846 - 0.001;
  input.aim_pitch_rad = input.feedback_pitch_rad = 0.0;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(result.allowed, "yaw 跨越正负 pi 边界时应按最短角度差处理")) return 1;

  config.min_aligned_frames = 2;
  aligned_frames = 0;
  input.aim_yaw_rad = input.feedback_yaw_rad = 0.0;
  input.mpc_candidate = false;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(!result.allowed, "无 MPC 候选时不应开火") ||
      !check(result.aligned_frames == 0, "无 MPC 候选时不能预先累计对准帧")) return 1;
  input.mpc_candidate = true;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(!result.allowed, "累计帧数不足时应继续拦截") ||
      !check(result.reason == "aligning", "累计帧数不足时应报告 aligning")) return 1;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(result.allowed, "连续达到最小对准帧数后应允许开火")) return 1;

  input.tracking = false;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(!result.allowed, "非 tracking 状态时必须拦截") ||
      !check(result.reason == "target_not_tracking", "非 tracking 时应报告拦截原因") ||
      !check(result.aligned_frames == 0, "失去 tracking 后连续对准帧数应清零")) return 1;

  input.tracking = true;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(!result.allowed, "重新 tracking 后需重新累计对准帧数")) return 1;
  input.fire_delay_valid = false;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(!result.allowed, "无有效发弹延迟时必须拦截") ||
      !check(result.reason == "fire_delay_unavailable", "无有效发弹延迟应报告原因") ||
      !check(result.aligned_frames == 0, "发弹延迟无效时不能累计对准帧")) return 1;

  input.fire_delay_valid = true;
  input.feedback_fresh = false;
  result = hfut::io::evaluateFireDecision(config, input, aligned_frames);
  if (!check(!result.allowed, "反馈过期时必须拦截") ||
      !check(result.reason == "feedback_stale", "反馈过期时应报告原因") ||
      !check(result.aligned_frames == 0, "反馈过期时连续对准帧数应清零")) return 1;

  config.enable_alignment_gate = true;
  config.min_aligned_frames = 1;
  config.min_aligned_duration_s = 0.03;
  input.feedback_fresh = true;
  input.mpc_candidate = false;
  int duration_frames = 0;
  double aligned_duration_s = 0.0;
  result = hfut::io::evaluateFireDecision(
      config, input, duration_frames, 0.02, &aligned_duration_s);
  if (!check(!result.allowed, "无 MPC 候选时即使已对齐也不能开火") ||
      !check(result.aligned_frames == 1, "候选中断时仍可记录持续对齐") ||
      !check(result.reason == "mpc_no_candidate", "候选中断时应报告无候选")) return 1;
  result = hfut::io::evaluateFireDecision(
      config, input, duration_frames, 0.02, &aligned_duration_s);
  if (!check(!result.allowed, "持续对齐期间没有候选仍不得开火")) return 1;
  input.mpc_candidate = true;
  result = hfut::io::evaluateFireDecision(
      config, input, duration_frames, 0.01, &aligned_duration_s);
  if (!check(result.allowed, "持续对齐达标且当前帧有候选时应允许开火")) return 1;
  result = hfut::io::evaluateFireDecision(
      config, input, duration_frames, 0.2, &aligned_duration_s);
  if (!check(!result.allowed && result.aligned_frames == 0,
             "观测间隔过长时不能把间隔直接当作连续对齐时间")) return 1;
  input.aim_yaw_rad = 1.0;
  result = hfut::io::evaluateFireDecision(
      config, input, duration_frames, 0.01, &aligned_duration_s);
  if (!check(!result.allowed && result.aligned_frames == 0,
             "角度失配后持续对齐计时必须清零")) return 1;

  std::puts("开火门控测试通过");
  return 0;
}
