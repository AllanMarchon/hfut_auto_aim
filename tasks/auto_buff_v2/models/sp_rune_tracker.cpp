#include "sp_rune_tracker.hpp"

#include <cmath>

#include "tools/math_tools.hpp"

namespace auto_buff_v2
{
namespace
{
Eigen::Vector3d blade_position(const Eigen::VectorXd & x)
{
  return tools::ypd2xyz(Eigen::Vector3d(x[0], x[2], x[3])) +
    tools::rotation_matrix(Eigen::Vector3d(x[4], 0, x[5])) * Eigen::Vector3d(0, 0, 0.7);
}
}  // namespace

void SPTracker::initialize(const SPRuneObservation & observation, double elapsed)
{
  last_time_ = elapsed;
  const int size = big_rune_ ? 10 : 7;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(size);
  x.head<7>() << observation.ypd_in_world[0], 0, observation.ypd_in_world[1],
    observation.ypd_in_world[2], observation.ypr_in_world[0], observation.ypr_in_world[2],
    big_rune_ ? config_.initial_big_speed : direction() * config_.initial_small_speed;
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(size, size) * config_.initial_covariance;
  if (big_rune_) {
    x.tail<3>() << config_.initial_amplitude, config_.initial_frequency, config_.initial_phase;
    P(6, 6) = config_.big_speed_covariance;
    P(9, 9) = config_.big_phase_covariance;
  } else {
    P(6, 6) = config_.small_speed_covariance;
  }
  const auto add = [big = big_rune_](const Eigen::VectorXd & a,
                                   const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd result = a + b;
    for (const int index : {0, 2, 4, 5}) result[index] = tools::limit_rad(result[index]);
    if (big) result[9] = tools::limit_rad(result[9]);
    return result;
  };
  ekf_ = tools::ExtendedKalmanFilter(x, P, add);
  initialized_ = true;
}

void SPTracker::predict(double dt)
{
  const int size = ekf_.x.size();
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(size, size);
  F(0, 1) = dt;
  F(5, 6) = big_rune_ ? direction() * dt : dt;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(size, size);
  const double variance =
    big_rune_ ? config_.big_yaw_acceleration_noise : config_.small_yaw_acceleration_noise;
  Q(0, 0) = std::pow(dt, 4) / 4 * variance;
  Q(0, 1) = Q(1, 0) = std::pow(dt, 3) / 2 * variance;
  Q(1, 1) = dt * dt * variance;
  const double time = last_time_ + dt;
  if (big_rune_) {
    const double a = ekf_.x[7], w = ekf_.x[8], phi = ekf_.x[9];
    F(6, 6) = 0;
    F(6, 7) = std::sin(w * time + phi) - 1;
    F(6, 8) = time * a * std::cos(w * time + phi);
    F(6, 9) = a * std::cos(w * time + phi);
    Q(5, 5) = config_.big_rotation_angle_noise;
    Q(6, 6) = config_.big_rotation_speed_noise;
    Q(9, 9) = config_.big_phase_noise;
  }
  const auto transition = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd result = x;
    result[0] = tools::limit_rad(x[0] + dt * x[1]);
    if (big_rune_) {
      const double a = x[7], w = x[8], phi = x[9];
      result[5] = x[5] + direction() *
        (-a / w * std::cos(w * time + phi) + a / w * std::cos(w * last_time_ + phi) +
         (2.09 - a) * dt);
      result[6] = a * std::sin(w * time + phi) + 2.09 - a;
    } else {
      result[5] = x[5] + dt * x[6];
    }
    for (const int index : {2, 4, 5}) result[index] = tools::limit_rad(result[index]);
    return result;
  };
  ekf_.predict(F, Q, transition);
}

void SPTracker::reset()
{
  // 兼容旧模型的重新初始化会保留时钟和方向投票。
  initialized_ = false;
  lost_count_ = 0;
}

bool SPTracker::update(const SPRuneObservation & observation, Timestamp timestamp)
{
  return update(std::optional<SPRuneObservation>(observation), timestamp);
}

bool SPTracker::update(const std::optional<SPRuneObservation> & observation, Timestamp timestamp)
{
  if (!observation) {
    ++lost_count_;
    return false;
  }
  if (!start_timestamp_) start_timestamp_ = timestamp;
  const double elapsed = std::chrono::duration<double>(timestamp - *start_timestamp_).count();
  if (!initialized_) initialize(*observation, elapsed);
  if (lost_count_ > config_.max_lost_frames) {
    reset();
    return false;
  }
  correct(*observation, elapsed);
  if (!big_rune_ &&
      (std::abs(ekf_.x[6]) > config_.initial_small_speed + config_.small_speed_tolerance ||
       std::abs(ekf_.x[6]) < config_.initial_small_speed - config_.small_speed_tolerance)) {
    initialized_ = false;
    return false;
  }
  if (big_rune_ &&
      (ekf_.x[7] > 1.045 * config_.divergence_factor ||
       ekf_.x[7] < 0.78 / config_.divergence_factor ||
       ekf_.x[8] > 2.0 * config_.divergence_factor ||
       ekf_.x[8] < 1.884 / config_.divergence_factor)) {
  // BigTarget 会先输出这一帧，再在下一次观测时完成初始化。
    initialized_ = false;
  }
  return true;
}

void SPTracker::correct(const SPRuneObservation & observation, double elapsed)
{
  const double roll = observation.ypr_in_world[2];
  if (std::abs(roll - ekf_.x[5]) > CV_PI / 12) {
    for (int i = -5; i <= 5; ++i) {
      if (std::abs(ekf_.x[5] + i * 2 * CV_PI / 5 - roll) < CV_PI / 5) {
        ekf_.x[5] += i * 2 * CV_PI / 5;
        break;
      }
    }
  }
  if (std::abs(direction_votes_) <= 50) direction_votes_ += ekf_.x[5] > roll ? -1 : 1;
  if (!big_rune_ && direction() * ekf_.x[6] < 0) ekf_.x[6] *= -1;
  predict(elapsed - last_time_);

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, ekf_.x.size());
  H(0, 0) = H(1, 2) = H(2, 3) = H(3, 5) = 1;
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R.diagonal() << config_.observation_yaw, config_.observation_pitch,
    config_.observation_distance, config_.observation_rotation_angle;
  Eigen::Vector4d z;
  z << observation.ypd_in_world, observation.ypr_in_world[2];
  const auto subtract_pose = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b)
    -> Eigen::VectorXd {
    Eigen::VectorXd result = a - b;
    for (const int index : {0, 1, 3}) result[index] = tools::limit_rad(result[index]);
    return result;
  };
  ekf_.update(z, H, R, subtract_pose);

  const Eigen::MatrixXd Hb = blade_jacobian();
  Eigen::Matrix3d Rb = Eigen::Matrix3d::Zero();
  Rb.diagonal() << config_.observation_yaw, config_.observation_pitch,
    config_.observation_distance;
  const auto observe_blade = [](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    return tools::xyz2ypd(blade_position(x));
  };
  const auto subtract_blade = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b)
    -> Eigen::VectorXd {
    Eigen::VectorXd result = a - b;
    result[0] = tools::limit_rad(result[0]);
    result[1] = tools::limit_rad(result[1]);
    return result;
  };
  ekf_.update(observation.blade_ypd_in_world, Hb, Rb, observe_blade, subtract_blade);
  last_time_ = elapsed;
}

Eigen::MatrixXd SPTracker::blade_jacobian() const
{
  const auto & x = ekf_.x;
  Eigen::MatrixXd H0 = Eigen::MatrixXd::Zero(5, x.size());
  H0(0, 0) = H0(1, 2) = H0(2, 3) = H0(3, 4) = H0(4, 5) = 1;
  Eigen::Matrix<double, 5, 5> H1 = Eigen::Matrix<double, 5, 5>::Identity();
  H1.topLeftCorner<3, 3>() = tools::ypd2xyz_jacobian(Eigen::Vector3d(x[0], x[2], x[3]));
  const double cy = std::cos(x[4]), sy = std::sin(x[4]);
  const double cr = std::cos(x[5]), sr = std::sin(x[5]);
  Eigen::Matrix<double, 3, 5> H2;
  // 关闭 clang-format 自动格式化。
  H2 << 1, 0, 0, 0.7 * cy * sr,  0.7 * sy * cr,
        0, 1, 0, 0.7 * sy * sr, -0.7 * cy * cr,
        0, 0, 1,              0,        -0.7 * sr;
  // 恢复 clang-format 自动格式化。
  return tools::xyz2ypd_jacobian(blade_position(x)) * H2 * H1 * H0;
}
}  // namespace auto_buff_v2
