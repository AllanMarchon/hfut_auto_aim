#include "sp_rune_model.hpp"

#include <cmath>

#include "tools/math_tools.hpp"

namespace auto_buff
{
SPRuneConfig SPRuneConfig::load(const YAML::Node & node)
{
  SPRuneConfig result;
  if (!node) return result;
  result.max_lost_frames = node["max_lost_frames"].as<int>(result.max_lost_frames);
  result.small_speed_tolerance =
    node["small_speed_tolerance"].as<double>(result.small_speed_tolerance);
  result.small_yaw_acceleration_noise =
    node["small_yaw_acceleration_noise"].as<double>(result.small_yaw_acceleration_noise);
  result.big_yaw_acceleration_noise =
    node["big_yaw_acceleration_noise"].as<double>(result.big_yaw_acceleration_noise);
  result.big_rotation_angle_noise =
    node["big_rotation_angle_noise"].as<double>(result.big_rotation_angle_noise);
  result.big_rotation_speed_noise =
    node["big_rotation_speed_noise"].as<double>(result.big_rotation_speed_noise);
  result.big_phase_noise = node["big_phase_noise"].as<double>(result.big_phase_noise);
  result.observation_yaw = node["observation_yaw"].as<double>(result.observation_yaw);
  result.observation_pitch = node["observation_pitch"].as<double>(result.observation_pitch);
  result.observation_distance =
    node["observation_distance"].as<double>(result.observation_distance);
  result.observation_rotation_angle =
    node["observation_rotation_angle"].as<double>(result.observation_rotation_angle);
  result.initial_small_speed = node["initial_small_speed"].as<double>(result.initial_small_speed);
  result.initial_big_speed = node["initial_big_speed"].as<double>(result.initial_big_speed);
  result.initial_amplitude = node["initial_amplitude"].as<double>(result.initial_amplitude);
  result.initial_frequency = node["initial_frequency"].as<double>(result.initial_frequency);
  result.initial_phase = node["initial_phase"].as<double>(result.initial_phase);
  result.initial_covariance = node["initial_covariance"].as<double>(result.initial_covariance);
  result.small_speed_covariance =
    node["small_speed_covariance"].as<double>(result.small_speed_covariance);
  result.big_speed_covariance =
    node["big_speed_covariance"].as<double>(result.big_speed_covariance);
  result.big_phase_covariance =
    node["big_phase_covariance"].as<double>(result.big_phase_covariance);
  result.divergence_factor = node["divergence_factor"].as<double>(result.divergence_factor);
  return result;
}

SPRuneModel::SPRuneModel(BuffConfig::Camera camera, Config config, bool big_rune)
: solver_(std::move(camera)), config_(std::move(config)), big_rune_(big_rune)
{
  reset();
}

void SPRuneModel::set_q_gimbal2world(const Eigen::Quaterniond & q)
{
  solver_.set_q_gimbal2world(q);
}

void SPRuneModel::reset()
{
  if (tracker_) {
    tracker_->reset();
  } else {
    tracker_ = std::make_unique<SPTracker>(big_rune_, config_);
  }
  state_.reset();
  diagnostic_.clear();
}

bool SPRuneModel::update(const RuneElements & elements, Timestamp timestamp)
{
  if (state_ && timestamp < state_->timestamp) {
    reset();
    diagnostic_ = "timestamp reversed";
    return false;
  }
  const auto observation = solver_.solve(elements);
  if (!tracker_->update(observation, timestamp)) {
    state_.reset();
    diagnostic_ = observation ? "SP reinitializing" : "missing inactive bullseye or valid PnP";
    return false;
  }
  const auto & x = tracker_->state();
  if (!x.allFinite()) {
    reset();
    diagnostic_ = "SP model diverged";
    return false;
  }
  RuneEstimate estimate;
  estimate.start_timestamp = tracker_->start_timestamp();
  estimate.requires_warmup = false;
  estimate.update_count = state_ ? state_->update_count + 1 : 1;
  estimate.timestamp = timestamp;
  estimate.center = tools::ypd2xyz(Eigen::Vector3d(x[0], x[2], x[3]));
  estimate.face_yaw = x[4];
  estimate.center_yaw_speed = x[1];
  estimate.rotation_angle = x[5];
    estimate.inactive[0] = true;  // SP 将选中的未激活符叶作为第 0 片符叶跟踪。
  estimate.prediction_method = RunePredictionMethod::ekf;
  estimate.rotation_speed = x[6];
  if (big_rune_) {
    // 旧 BigTarget 使用 EKF 正弦参数；这里导出带符号的同一运动规律，
    // 使共享预测器能够处理正反两个旋转方向。
    const double elapsed = tracker_->time();
    const int direction = tracker_->direction();
    estimate.prediction_method = RunePredictionMethod::weighted_least_squares;
    estimate.sine_valid = true;
    estimate.sine_v = direction * (2.09 - x[7]);
    estimate.sine_a = direction * x[7];
    estimate.sine_omega = x[8];
    estimate.sine_phase = x[8] * elapsed + x[9];
    estimate.sine_t = elapsed;
    estimate.rotation_speed = direction * x[6];
  }
  state_ = estimate;
  diagnostic_ = "tracking";
  return true;
}

std::vector<RuneReprojectedFeature> SPRuneModel::reprojected_features() const
{
  if (!state_) return {};
  std::vector<RuneReprojectedFeature> result;
  if (const auto icon = solver_.project(state_->center)) result.push_back({0, *icon});
  for (int blade = 0; blade < 5; ++blade) {
    const double angle = state_->rotation_angle + blade * 2 * CV_PI / 5;
    const Eigen::Vector3d world = state_->center +
      Eigen::AngleAxisd(state_->face_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::Vector3d(0, -0.7 * std::sin(angle), 0.7 * std::cos(angle));
    if (const auto pixel = solver_.project(world)) result.push_back({blade + 1, *pixel});
  }
  return result;
}

std::optional<cv::Point2f> SPRuneModel::reprojected_center() const
{
  return state_ ? solver_.project(state_->center) : std::nullopt;
}
}  // namespace auto_buff
