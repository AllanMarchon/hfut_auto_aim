#ifndef AUTO_BUFF__TARGET_HPP
#define AUTO_BUFF__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "buff_detector.hpp"
#include "buff_type.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/ransac_sine_fitter.hpp"

namespace auto_buff
{
class Voter
{
public:
  Voter();
  void reset();
  void vote(const double angle_last, const double angle_now);
  int clockwise();

private:
  int clockwise_;
};

/// Target 基类

class Target
{
public:
  Target();
  virtual void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) = 0;  // 纯虚函数

  virtual void predict(double dt) = 0;  // 纯虚函数

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & point_in_buff) const;

  bool is_unsolve() const;

  // 只有经过连续有效观测确认后，目标才允许进入稳定跟踪。
  bool is_tracking_ready() const;

  Eigen::VectorXd ekf_x() const;

  double spd = 0;  //调试用

protected:
  // 使用指定的滤波状态计算符叶在世界坐标系中的位置，供 EKF 观测模型使用。
  Eigen::Vector3d point_buff2world(
    const Eigen::VectorXd & state, const Eigen::Vector3d & point_in_buff) const;

  virtual void init(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  virtual void update(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  Eigen::VectorXd x0_;
  Eigen::MatrixXd P0_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  tools::ExtendedKalmanFilter ekf_;
  double lasttime_ = 0;
  Voter voter;  // 逆时针-1 顺时针1
  bool first_in_;
  bool unsolvable_;
  int lost_count_ = 0;
  bool have_start_timestamp_ = false;
  std::chrono::steady_clock::time_point start_timestamp_{};
  int valid_count_ = 0;
  bool tracking_ready_ = false;

  bool state_is_finite() const;
};

/// SmallTarget子类

class SmallTarget : public Target
{
public:
  SmallTarget();

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;

private:
  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  Eigen::MatrixXd h_jacobian() const;

  const double SMALL_W = CV_PI / 3;
  // const double SMALL_W = 0;
};

/// BigTarget子类

class BigTarget : public Target
{
public:
  BigTarget();

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;

private:
  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  Eigen::MatrixXd h_jacobian() const;

  tools::RansacSineFitter spd_fitter_;

  double fit_spd_;
};

}  // namespace auto_buff
#endif
