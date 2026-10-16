#ifndef AUTO_BUFF_V2__SP_RUNE_TRACKER_HPP
#define AUTO_BUFF_V2__SP_RUNE_TRACKER_HPP

#include "rune_model.hpp"
#include "sp_rune_config.hpp"
#include "sp_rune_solver.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_buff_v2
{
// 移植 SmallTarget 的 7 状态和 BigTarget 的 10 状态球坐标 EKF。
// 状态顺序：[符心 yaw、yaw 速度、符心 pitch、距离、符面 yaw、转角、速度、加速度、角频率、相位]。
class SPTracker
{
public:
  SPTracker(bool big_rune, SPRuneConfig config) : big_rune_(big_rune), config_(config) {}
  bool update(const SPRuneObservation & observation, Timestamp timestamp);
  bool update(const std::optional<SPRuneObservation> & observation, Timestamp timestamp);
  void reset();
  const Eigen::VectorXd & state() const { return ekf_.x; }
  int direction() const { return direction_votes_ > 0 ? 1 : -1; }
  Timestamp start_timestamp() const { return start_timestamp_.value(); }
  double time() const { return last_time_; }

private:
  void initialize(const SPRuneObservation & observation, double elapsed);
  void correct(const SPRuneObservation & observation, double elapsed);
  void predict(double dt);
  Eigen::MatrixXd blade_jacobian() const;

  bool big_rune_;
  SPRuneConfig config_;
  bool initialized_ = false;
  int direction_votes_ = 0;
  int lost_count_ = 0;
  std::optional<Timestamp> start_timestamp_;
  double last_time_ = 0;
  tools::ExtendedKalmanFilter ekf_;
};
}  // namespace auto_buff_v2

#endif
