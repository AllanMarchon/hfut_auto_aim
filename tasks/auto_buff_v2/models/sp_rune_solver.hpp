#ifndef AUTO_BUFF_V2__SP_RUNE_SOLVER_HPP
#define AUTO_BUFF_V2__SP_RUNE_SOLVER_HPP

#include <optional>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "../buff_config.hpp"
#include "../rune.hpp"

namespace auto_buff_v2
{
// 移植后的球坐标 EKF 使用的 PnP 观测量。
struct SPRuneObservation
{
  Eigen::Vector3d ypd_in_world;
  Eigen::Vector3d ypr_in_world;
  Eigen::Vector3d blade_ypd_in_world;
};

class SPRuneSolver
{
public:
  explicit SPRuneSolver(BuffConfig::Camera camera);
  void set_q_gimbal2world(const Eigen::Quaterniond & q) { q_gimbal2world_ = q; }
  std::optional<SPRuneObservation> solve(const RuneElements & elements) const;
  std::optional<cv::Point2f> project(const Eigen::Vector3d & world) const;

private:
  BuffConfig::Camera camera_;
  Eigen::Quaterniond q_gimbal2world_ = Eigen::Quaterniond::Identity();
};
}  // namespace auto_buff_v2

#endif
