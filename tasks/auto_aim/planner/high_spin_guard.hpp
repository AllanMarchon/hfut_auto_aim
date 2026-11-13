#ifndef AUTO_AIM__HIGH_SPIN_GUARD_HPP
#define AUTO_AIM__HIGH_SPIN_GUARD_HPP

#include <Eigen/Dense>

#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"

namespace auto_aim
{

struct HighSpinGuardConfig
{
  bool enable{false};

  // 未处于守点模式时，转速大于该值才进入。
  double enter_speed_rad_s{10.0};

  // 已处于守点模式时，转速小于等于该值才退出。
  double exit_speed_rad_s{8.0};

  // 预计命中时刻，装甲板法向与守点方向的最大角度误差。
  double phase_tolerance_rad{0.12};

  // 云台需要提前保持对齐的时间。
  double aligned_hold_s{0.02};
};

class HighSpinGuard
{
public:
  void configure(const HighSpinGuardConfig & config);

  // 根据目标 EKF 角速度更新模式状态。
  // 返回 true 表示本帧应使用高速守点策略。
  bool update(const Target & target);

  void reset();

  bool active() const;

  // 返回当前目标旋转中心前方的固定守点。
  // 格式为 x、y、z、装甲板法向角。
  Eigen::Vector4d aimPoint(const Target & target) const;

  // 判断预计命中时刻是否有合适的装甲板进入守点窗口。
  bool phaseReady(const Target & impact_target) const;

  // 返回预计命中时刻装甲板与守点的最小相位误差。
  double phaseError(const Target & impact_target) const;

private:
  bool initializeGuard(const Target & target);

  static double centerBearing(const Target & target);

  HighSpinGuardConfig config_;

  bool active_{false};
  ArmorName target_name_{not_armor};

  // 进入守点模式时锁定当前最近装甲板对应的半径和高度层。
  double guard_radius_m_{0.0};
  double guard_height_offset_m_{0.0};

  // 四块装甲板可能有长短半径和高低差，只允许同一组装甲板触发开火。
  std::vector<int> eligible_armor_indices_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__HIGH_SPIN_GUARD_HPP