#include "tasks/auto_aim/spin_speed_profile.hpp"

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
  const auto yaml = YAML::Load(R"(
- speed_rad_s: 0.0
  extra_prediction_s: 0.00
  aligned_hold_ms: 0.0
- speed_rad_s: 3.0
  extra_prediction_s: 0.03
  aligned_hold_ms: 30.0
- speed_rad_s: 6.0
  extra_prediction_s: 0.06
  aligned_hold_ms: 60.0
)");
  const auto profile = auto_aim::SpinSpeedProfile::fromYaml(yaml);
  const auto at_three = profile.sample(3.0);
  const auto between = profile.sample(4.5);
  const auto below = profile.sample(-1.0);
  const auto above = profile.sample(9.0);
  if (!check(std::abs(at_three.extra_prediction_s - 0.03) < 1e-12,
             "标定点应返回对应额外预测时间") ||
      !check(std::abs(between.extra_prediction_s - 0.045) < 1e-12,
             "标定点之间应线性插值预测时间") ||
      !check(std::abs(between.aligned_hold_s - 0.045) < 1e-12,
             "标定点之间应线性插值开火保持时间") ||
      !check(std::abs(below.aligned_hold_s) < 1e-12,
             "低于首档时应使用首档值") ||
      !check(std::abs(above.extra_prediction_s - 0.06) < 1e-12,
             "高于末档时应使用末档值")) return 1;

  std::puts("小陀螺转速配置表测试通过");
  return 0;
}
