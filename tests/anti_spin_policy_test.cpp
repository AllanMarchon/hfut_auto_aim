#include "tasks/auto_aim/planner/anti_spin_policy.hpp"

#include <cstdio>

namespace {
bool check(bool condition, const char* message) {
  if (condition) return true;
  std::fprintf(stderr, "测试失败：%s\n", message);
  return false;
}
}  // 命名空间

int main() {
  auto_aim::AntiSpinModeSelector selector;
  selector.configure(true, 7.0, 3, 5, 1.0);
  if (!check(!selector.update(7.5, true), "高速连续确认不足时不应切换模式") ||
      !check(!selector.update(-8.0, true), "反向高速也应等待进入确认帧数") ||
      !check(selector.update(9.0, true), "达到进入确认帧数后应切换高速模式") ||
      !check(selector.update(6.5, true), "退出滞回区内应保持高速模式") ||
      !check(selector.update(5.5, true), "退出确认未满时应保持高速模式")) return 1;
  for (int i = 0; i < 3; ++i) {
    if (!check(selector.update(5.5, true), "退出确认未满时应继续保持高速模式")) return 1;
  }
  if (!check(!selector.update(5.5, true), "满足退出确认帧数后应退出高速模式") ||
      !check(!selector.update(15.0, false), "不支持该模式的目标应强制退出")) return 1;

  constexpr double pi = 3.14159265358979323846;
  const bool wrapped_front_face = auto_aim::antiSpinShotCandidate(
    -pi + 0.01, pi - 0.01, -pi + 0.02, pi - 0.02,
    0.10, 0.11, 0.05, 0.05, 0.02);
  if (!check(wrapped_front_face, "角度跨越正负 pi 边界时应使用最短角差")) return 1;

  const bool edge_miss = auto_aim::antiSpinShotCandidate(
    0.0, 0.0, 0.20, 0.0, 0.0, 0.0, 0.05, 0.10, 0.02);
  if (!check(!edge_miss, "装甲板中心超出弹道角宽时不应提出开火候选")) return 1;

  std::puts("高速反小陀螺策略测试通过");
  return 0;
}
