#include "njust_rune_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <fmt/format.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "../hungarian.hpp"
#include "../rune_predictor.hpp"

namespace auto_buff
{
NjustRuneModel::Config NjustRuneModel::Config::load(const YAML::Node & node)
{
  Config result;
  if (!node) return result;
  const auto prediction_method =
    node["prediction_method"].as<std::string>("weighted_least_squares");
  if (prediction_method == "ekf") {
    result.prediction_method = RunePredictionMethod::ekf;
  } else if (prediction_method != "weighted_least_squares") {
    throw std::invalid_argument(
      "buff.model.njust.prediction_method must be weighted_least_squares or ekf");
  }
  const auto motion_fit = node["motion_fit"];
  if (motion_fit) {
    const auto load_fit = [](const YAML::Node & node, BuffConfig::MotionFit & fit) {
      if (!node) return;
      fit.window_seconds = node["window_seconds"].as<double>(fit.window_seconds);
      fit.min_fit_seconds = node["min_fit_seconds"].as<double>(fit.min_fit_seconds);
      fit.weight_half_life_seconds =
        node["weight_half_life_seconds"].as<double>(fit.weight_half_life_seconds);
    };
    load_fit(motion_fit["small"], result.small_motion_fit);
    load_fit(motion_fit["big"], result.big_motion_fit);
  }
  result.timeout_seconds = node["timeout_seconds"].as<double>(result.timeout_seconds);
  result.noise_x = node["noise_x"].as<double>(result.noise_x);
  result.noise_y = node["noise_y"].as<double>(result.noise_y);
  result.noise_z = node["noise_z"].as<double>(result.noise_z);
  result.noise_rotation_speed =
    node["noise_rotation_speed"].as<double>(result.noise_rotation_speed);
  result.noise_rotation_angle =
    node["noise_rotation_angle"].as<double>(result.noise_rotation_angle);
  result.noise_face_yaw =
    node["noise_face_yaw"].as<double>(result.noise_face_yaw);
  result.noise_observation =
    node["noise_observation"].as<double>(result.noise_observation);
  result.gate_threshold = node["gate_threshold"].as<double>(result.gate_threshold);
  result.init_seed_mean_error =
    node["init_seed_mean_error"].as<double>(result.init_seed_mean_error);
  result.init_seed_max_error =
    node["init_seed_max_error"].as<double>(result.init_seed_max_error);
  result.init_center_gate =
    node["init_center_gate"].as<double>(result.init_center_gate);
  result.init_pitch_bound =
    node["init_pitch_bound"].as<double>(result.init_pitch_bound);
  result.diverge_face_angle =
    node["diverge_face_angle"].as<double>(result.diverge_face_angle);
  return result;
}

NjustRuneModel::NjustRuneModel(BuffConfig::Camera camera, Config config, bool big_rune)
: camera_matrix_(std::move(camera.camera_matrix)),
  distort_coeffs_(std::move(camera.distort_coeffs)),
  R_camera2gimbal_(camera.R_camera2gimbal),
  t_camera2gimbal_(camera.t_camera2gimbal),
  big_rune_(big_rune),
  config_(std::move(config)),
  fitter_(big_rune ? config_.big_motion_fit : config_.small_motion_fit)
{
}

void NjustRuneModel::set_q_gimbal2world(const Eigen::Quaterniond & q_gimbal2world)
{
  q_gimbal2world_ = q_gimbal2world;
}

void NjustRuneModel::reset()
{
  diagnostic_.clear();
  state_.reset();
  fitter_.reset();
  inactive_timeout_.fill(Timestamp{});
  last_inactive_corrected_ = Timestamp{};
  force_sine_until_ = Timestamp{};
}

std::optional<RuneEstimate> NjustRuneModel::state() const { return state_; }

namespace
{
constexpr double kPi = 3.14159265358979323846;
using Vector = RuneEkfState::Vector;
using Matrix = Eigen::Matrix<double, 6, 6>;

double normalize_angle(double angle)
{
  return std::remainder(angle, 2 * kPi);
}

double squared_pixel_error(const cv::Point2f & a, const cv::Point2f & b)
{
  const double dx = static_cast<double>(a.x) - b.x;
  const double dy = static_cast<double>(a.y) - b.y;
  return dx * dx + dy * dy;
}

RuneEkfState ekf_state_from(const RuneEstimate & state)
{
  Vector result;
  result << state.center.x(), state.center.y(), state.center.z(), state.rotation_speed,
    state.rotation_angle, state.face_yaw;
  return RuneEkfState(result);
}

void ekf_state_into(RuneEstimate & state, const RuneEkfState & value)
{
  state.center = {value.center_x(), value.center_y(), value.center_z()};
  state.rotation_speed = value.rotation_speed();
  state.rotation_angle = value.rotation_angle();
  state.face_yaw = normalize_angle(value.face_yaw());
}

std::optional<cv::Point2f> project_feature(
  const RuneEkfState & value, int feature, const Eigen::Matrix3d & R_camera2world,
  const Eigen::Vector3d & t_camera2world, const cv::Mat & camera_matrix,
  const cv::Mat & distortion)
{
  Eigen::Vector3d local;
  if (feature == 0) {
    local = {-kRuneIconProminentDistance, 0, 0};
  } else {
    const double angle = value.rotation_angle() + (feature - 1) * 2 * kPi / 5;
    local = {0, -kRuneGlobalRadius * std::sin(angle), kRuneGlobalRadius * std::cos(angle)};
  }
  const Eigen::Vector3d world(value.center_x(), value.center_y(), value.center_z());
  const Eigen::Vector3d rotated_world =
    world + Eigen::AngleAxisd(value.face_yaw(), Eigen::Vector3d::UnitZ()) * local;
  const Eigen::Vector3d camera = R_camera2world.transpose() * (rotated_world - t_camera2world);
  if (camera.z() <= 0.1) return std::nullopt;
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(std::vector<cv::Point3f>{{static_cast<float>(camera.x()),
                                             static_cast<float>(camera.y()),
                                             static_cast<float>(camera.z())}},
                    cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, distortion, pixels);
  return pixels.front();
}

Eigen::Matrix<double, 2, 6> observation_jacobian(
  const RuneEkfState & x, int feature, const Eigen::Matrix3d & R_camera2world,
  const Eigen::Vector3d & t_camera2world, const cv::Mat & camera_matrix,
  const cv::Mat & distortion)
{
  Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
  const auto predicted = project_feature(x, feature, R_camera2world, t_camera2world,
                                          camera_matrix, distortion);
  if (!predicted) return H;
  for (int j = 0; j < 6; ++j) {
    RuneEkfState perturb = x;
    perturb.add_component(static_cast<RuneEkfStateComponent>(j), 1e-4);
    const auto p = project_feature(perturb, feature, R_camera2world, t_camera2world,
                                   camera_matrix, distortion);
    if (!p) continue;
    H(0, j) = (p->x - predicted->x) / 1e-4;
    H(1, j) = (p->y - predicted->y) / 1e-4;
  }
  return H;
}

bool solve_seed(const RuneIcon & icon, const RuneBullseye & bull, const cv::Mat & camera_matrix,
                const cv::Mat & distortion, const Eigen::Matrix3d & R_camera2world,
                const Eigen::Vector3d & t_camera2world, const NjustRuneModel::Config & config,
                RuneEstimate & state, double & seed_sse, double & seed_max_error,
                std::string & rejection)
{
  rejection = "degenerate corners";
  std::size_t bottom = 0, top = 0;
  double nearest = std::numeric_limits<double>::max(), farthest = 0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double distance = cv::norm(bull.corners[i] - icon.center);
    if (distance < nearest) { nearest = distance; bottom = i; }
    if (distance > farthest) { farthest = distance; top = i; }
  }
  if (bottom == top) return false;
  std::array<std::size_t, 2> sides{};
  int count = 0;
  for (std::size_t i = 0; i < 4; ++i)
    if (i != bottom && i != top) sides[count++] = i;
  if (count != 2) return false;
  const auto & b = bull.corners[bottom];
  const auto & t = bull.corners[top];
  const cv::Point2f up = t - b;
  const double up_length = cv::norm(up);
  if (up_length <= 1e-6) return false;
  const cv::Point2f right(up.y / up_length, -up.x / up_length);
  auto l = bull.corners[sides[0]], r = bull.corners[sides[1]];
  const double first_projection = (l - bull.center).dot(right);
  const double second_projection = (r - bull.center).dot(right);
  if (std::abs(first_projection - second_projection) <= 1e-6) return false;
  if (first_projection < second_projection) std::swap(l, r);

  const std::vector<cv::Point3f> object = {
    {-0.1f, 0, 0}, {0, 0, 0.85f}, {0, 0.15f, 0.7f},
    {0, 0, 0.55f}, {0, -0.15f, 0.7f}};
  const std::vector<cv::Point2f> image = {icon.center, t, l, b, r};
  cv::Vec3d rvec, tvec;
  rejection = "PnP failed or behind camera";
  try {
    if (!cv::solvePnP(object, image, camera_matrix, distortion, rvec, tvec, false,
                      cv::SOLVEPNP_EPNP) || tvec[2] <= 0) return false;
    if (!cv::solvePnP(object, image, camera_matrix, distortion, rvec, tvec, true,
                      cv::SOLVEPNP_ITERATIVE) || tvec[2] <= 0) return false;
  } catch (const cv::Exception &) { return false; }
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  Eigen::Matrix3d R_page2camera;
  cv::cv2eigen(rotation, R_page2camera);
  const Eigen::Matrix3d R_page2world = R_camera2world * R_page2camera;
  const Eigen::Vector3d face = R_page2world.col(0);
  rejection = "degenerate face normal";
  if (face.head<2>().squaredNorm() <= 1e-6) return false;
  const double face_pitch = std::asin(std::abs(face.z())) * 180 / kPi;
  rejection = fmt::format("face pitch {:.1f} > {:.1f} deg", face_pitch, config.init_pitch_bound);
  if (std::asin(std::abs(face.z())) > config.init_pitch_bound * kPi / 180) return false;
  state.center = R_camera2world * Eigen::Vector3d(tvec[0], tvec[1], tvec[2]) + t_camera2world;
  state.face_yaw = std::atan2(face.y(), face.x());
  const Eigen::Vector3d horizontal_face(std::cos(state.face_yaw),
                                         std::sin(state.face_yaw), 0);
  rejection = "face normal reversed";
  if ((R_camera2world.transpose() * horizontal_face).z() <= 0) return false;
  const Eigen::Matrix3d local_rotation =
    Eigen::AngleAxisd(-state.face_yaw, Eigen::Vector3d::UnitZ()) * R_page2world;
  state.rotation_angle = std::atan2(-local_rotation(1, 2), local_rotation(2, 2));
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object, rvec, tvec, camera_matrix, distortion, projected);
  seed_sse = 0;
  seed_max_error = 0;
  for (std::size_t i = 0; i < image.size(); ++i) {
    const double error2 = squared_pixel_error(projected[i], image[i]);
    const double error = std::sqrt(error2);
    seed_sse += error2;
    seed_max_error = std::max(seed_max_error, error);
  }
  rejection = fmt::format("reprojection rms {:.1f}/{:.1f}, max {:.1f}/{:.1f} px",
                          std::sqrt(seed_sse / image.size()), config.init_seed_mean_error,
                          seed_max_error, config.init_seed_max_error);
  return std::sqrt(seed_sse / image.size()) <= config.init_seed_mean_error &&
         seed_max_error <= config.init_seed_max_error &&
         state.center.allFinite();
}

void correct_initial_observation(RuneEkfState & x, Matrix & covariance, int feature,
                                  const cv::Point2f & observed,
                                  const Eigen::Matrix3d & R_camera2world,
                                  const Eigen::Vector3d & t_camera2world,
                                  const cv::Mat & camera_matrix, const cv::Mat & distortion,
                                  double observation_noise)
{
  const auto predicted = project_feature(x, feature, R_camera2world, t_camera2world,
                                          camera_matrix, distortion);
  if (!predicted) return;
  const Eigen::Vector2d residual(observed.x - predicted->x, observed.y - predicted->y);
  const auto H = observation_jacobian(x, feature, R_camera2world, t_camera2world,
                                       camera_matrix, distortion);
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * observation_noise;
  const Eigen::Matrix2d S = H * covariance * H.transpose() + R;
  const Eigen::Matrix<double, 6, 2> K = covariance * H.transpose() * S.inverse();
  RuneEkfState corrected(x.vector() + K * residual);
  corrected.set_face_yaw(normalize_angle(corrected.face_yaw()));
  if (!corrected.all_finite()) return;
  const Matrix I = Matrix::Identity() - K * H;
  Matrix posterior = I * covariance * I.transpose() + K * R * K.transpose();
  posterior = 0.5 * (posterior + posterior.transpose());
  if (!posterior.allFinite()) return;
  x = corrected;
  covariance = posterior;
}
}  // namespace

std::vector<RuneReprojectedFeature> NjustRuneModel::reprojected_features() const
{
  if (!state_) return {};
  const Eigen::Matrix3d R_camera2world = q_gimbal2world_.toRotationMatrix() * R_camera2gimbal_;
  const Eigen::Vector3d t_camera2world = q_gimbal2world_ * t_camera2gimbal_;
  const auto state = ekf_state_from(*state_);
  std::vector<RuneReprojectedFeature> result;
  result.reserve(6);
  for (int id = 0; id <= 5; ++id) {
    const auto point = project_feature(
      state, id, R_camera2world, t_camera2world, camera_matrix_, distort_coeffs_);
    if (point) result.push_back({id, *point});
  }
  return result;
}

std::optional<cv::Point2f> NjustRuneModel::reprojected_center() const
{
  if (!state_) return std::nullopt;
  const Eigen::Matrix3d R_camera2world = q_gimbal2world_.toRotationMatrix() * R_camera2gimbal_;
  const Eigen::Vector3d t_camera2world = q_gimbal2world_ * t_camera2gimbal_;
  const Eigen::Vector3d camera =
    R_camera2world.transpose() * (state_->center - t_camera2world);
  if (camera.z() <= 0.1) return std::nullopt;
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(
    std::vector<cv::Point3f>{{static_cast<float>(camera.x()), static_cast<float>(camera.y()),
                              static_cast<float>(camera.z())}},
    cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix_, distort_coeffs_, pixels);
  return pixels.front();
}

bool NjustRuneModel::update(const RuneElements & elements, Timestamp timestamp)
{
  const Eigen::Matrix3d R_camera2world = q_gimbal2world_.toRotationMatrix() * R_camera2gimbal_;
  const Eigen::Vector3d t_camera2world = q_gimbal2world_ * t_camera2gimbal_;
  if (!state_) return initialize(elements, timestamp, R_camera2world, t_camera2world);

  auto & state = *state_;
  if (std::chrono::duration<double>(timestamp - last_inactive_corrected_).count() >
      config_.timeout_seconds) {
    reset();
    const bool initialized = initialize(elements, timestamp, R_camera2world, t_camera2world);
    diagnostic_ = "inactive timeout; " + diagnostic_;
    return initialized;
  }
  const double dt = std::chrono::duration<double>(timestamp - state.timestamp).count();
  if (dt < 0) { reset(); diagnostic_ = "timestamp reversed"; return false; }
  state = RunePredictor{}.predict(state, timestamp);
  RuneEkfState x = ekf_state_.predict(dt);
  Matrix F = Matrix::Identity();
  F(4, 3) = dt;
  Matrix Q = Matrix::Zero();
  Q.diagonal() << config_.noise_x, config_.noise_y, config_.noise_z,
    config_.noise_rotation_speed, config_.noise_rotation_angle, config_.noise_face_yaw;
  covariance_ = F * covariance_ * F.transpose() + Q;
  for (std::size_t i = 0; i < 5; ++i)
    if (timestamp >= inactive_timeout_[i]) state.inactive[i] = false;

  struct Observation { cv::Point2f pixel; bool icon; bool inactive; };
  std::vector<Observation> observations;
  for (const auto & icon : elements.icons) observations.push_back({icon.center, true, false});
  for (const auto & bull : elements.bullseyes)
    observations.push_back({bull.center, false, !bull.active});
  const double kGate = config_.gate_threshold;
  const double huge = std::numeric_limits<double>::max() / 4;
  Eigen::MatrixXd cost(observations.size(), 6);
  cost.setConstant(huge);
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const auto & observation = observations[i];
    for (int feature = observation.icon ? 0 : 1; feature <= (observation.icon ? 0 : 5); ++feature) {
      const auto predicted = project_feature(x, feature, R_camera2world, t_camera2world,
                                              camera_matrix_, distort_coeffs_);
      if (!predicted) continue;
      const Eigen::Vector2d residual(observation.pixel.x - predicted->x,
                                     observation.pixel.y - predicted->y);
      const auto H = observation_jacobian(x, feature, R_camera2world, t_camera2world,
                                           camera_matrix_, distort_coeffs_);
      const Eigen::Matrix2d S =
        H * covariance_ * H.transpose() +
        Eigen::Matrix2d::Identity() * config_.noise_observation;
      cost(i, feature) = residual.transpose() * S.inverse() * residual;
    }
  }
  const auto assignments = hungarian_assign(cost, kGate);
  int corrected_blades = 0;
  int corrected_inactive = 0;
  std::array<bool, 5> observed_inactive{};
  for (std::size_t i = 0; i < observations.size(); ++i) {
    if (!assignments[i]) continue;
    const int best = *assignments[i];
    const auto & observation = observations[i];
    const auto predicted = *project_feature(x, best, R_camera2world, t_camera2world,
                                            camera_matrix_, distort_coeffs_);
    Eigen::Vector2d residual(observation.pixel.x - predicted.x,
                             observation.pixel.y - predicted.y);
    const auto H = observation_jacobian(x, best, R_camera2world, t_camera2world,
                                        camera_matrix_, distort_coeffs_);
    Eigen::Matrix2d S = H * covariance_ * H.transpose() +
                        Eigen::Matrix2d::Identity() * config_.noise_observation;
    if (residual.transpose() * S.inverse() * residual > kGate) continue;
    const Eigen::Matrix<double, 6, 2> K = covariance_ * H.transpose() * S.inverse();
    x = RuneEkfState(x.vector() + K * residual);
    x.set_face_yaw(normalize_angle(x.face_yaw()));
    const Matrix I = Matrix::Identity() - K * H;
    covariance_ = I * covariance_ * I.transpose() + K *
                   (Eigen::Matrix2d::Identity() * config_.noise_observation) * K.transpose();
    if (best > 0) {
      ++corrected_blades;
      const int blade = best - 1;
      if (observation.inactive) {
        ++corrected_inactive;
        observed_inactive[blade] = true;
        inactive_timeout_[blade] = timestamp + std::chrono::milliseconds(100);
      } else {
        state.inactive[blade] = false;
      }
    }
  }
  // 只有当前帧没有观测到可击打符叶时才保留检测宽限期；
  // 否则低编号的旧符叶可能在瞄准时覆盖当前观测。
  if (corrected_inactive > 0) state.inactive = observed_inactive;
  ekf_state_ = x;
  ekf_state_into(state, x);
  if (state.sine_valid)
    state.rotation_speed = state.sine_v + state.sine_a * std::sin(state.sine_phase);
  else if (state.has_fitted_motion)
    state.rotation_speed = state.fitted_rotation_speed;
  if (diverged()) { reset(); diagnostic_ = "model diverged"; return false; }
  state.predicted = corrected_inactive == 0;
  if (corrected_inactive > 0) last_inactive_corrected_ = timestamp;
  if (corrected_inactive > 1) force_sine_until_ = timestamp + std::chrono::seconds(3);
  if (corrected_blades == 0) return false;
  ++state.update_count;
  const double elapsed_seconds =
    std::chrono::duration<double>(timestamp - state.start_timestamp).count();
  if (state.prediction_method == RunePredictionMethod::weighted_least_squares &&
      elapsed_seconds >= 1) update_motion_fit(state, elapsed_seconds);
  return true;
}

bool NjustRuneModel::initialize(const RuneElements & elements, Timestamp timestamp,
                           const Eigen::Matrix3d & R_camera2world,
                           const Eigen::Vector3d & t_camera2world)
{
  const auto inactive_count = std::count_if(elements.bullseyes.begin(),
                                            elements.bullseyes.end(),
                                            [](const auto & bull) { return !bull.active; });
  diagnostic_ = fmt::format("init: R={}, inactive={}; ", elements.icons.size(), inactive_count);
  if (elements.icons.empty()) {
    diagnostic_ += "missing R";
    return false;
  }
  if (inactive_count == 0 || inactive_count > 2) {
    diagnostic_ += "need 1 or 2 inactive bullseyes";
    return false;
  }
  std::string rejection;
  struct Candidate
  {
    RuneEstimate state;
    cv::Point2f icon_pixel;
    cv::Point2f seed_pixel;
    int inactive_inliers = 0;
    double seed_center_error = 0;
    double icon_error = 0;
    double seed_max_error = 0;
    double seed_sse = 0;
    double inactive_center_sse = 0;
  };
  const auto rank = [](const Candidate & item) {
    return std::make_tuple(-item.inactive_inliers, item.seed_center_error,
                           item.icon_error, item.seed_max_error, item.seed_sse,
                           item.inactive_center_sse);
  };
  std::optional<Candidate> best;
  for (const auto & icon : elements.icons) {
    for (const auto & bull : elements.bullseyes) {
      if (bull.active) continue;
      RuneEstimate seed;
      double seed_sse = 0, seed_max_error = 0;
      if (!solve_seed(icon, bull, camera_matrix_, distort_coeffs_, R_camera2world,
                      t_camera2world, config_, seed, seed_sse, seed_max_error, rejection)) continue;
      const RuneEkfState x = ekf_state_from(seed);
      const auto projected_icon = project_feature(x, 0, R_camera2world, t_camera2world,
                                                   camera_matrix_, distort_coeffs_);
      const auto projected_seed = project_feature(x, 1, R_camera2world, t_camera2world,
                                                   camera_matrix_, distort_coeffs_);
      if (!projected_icon || !projected_seed) {
        rejection = "model features behind camera";
        continue;
      }
      Candidate candidate;
      candidate.state = seed;
      candidate.icon_pixel = icon.center;
      candidate.seed_pixel = bull.center;
      candidate.icon_error = squared_pixel_error(*projected_icon, icon.center);
      candidate.seed_center_error = squared_pixel_error(*projected_seed, bull.center);
      candidate.seed_sse = seed_sse;
      candidate.seed_max_error = seed_max_error;
      const double center_gate2 = config_.init_center_gate * config_.init_center_gate;
      if (candidate.seed_center_error > center_gate2) {
        rejection = fmt::format("center error {:.1f} > {:.1f} px",
                                std::sqrt(candidate.seed_center_error), config_.init_center_gate);
        continue;
      }
      if (inactive_count > 1)
        candidate.inactive_center_sse = std::numeric_limits<double>::max();
      for (const auto & other : elements.bullseyes) {
        if (&other == &bull || other.active) continue;
        double nearest2 = std::numeric_limits<double>::max();
        for (int feature = 2; feature <= 5; ++feature) {
          const auto predicted = project_feature(x, feature, R_camera2world,
                                                  t_camera2world, camera_matrix_,
                                                  distort_coeffs_);
          if (predicted) {
            nearest2 = std::min(nearest2,
                                squared_pixel_error(*predicted, other.center));
          }
        }
        if (nearest2 <= center_gate2) {
          ++candidate.inactive_inliers;
          if (candidate.inactive_inliers == 1) candidate.inactive_center_sse = 0;
          candidate.inactive_center_sse += nearest2;
        }
      }
      const bool better = !best || rank(candidate) < rank(*best);
      if (better) best = candidate;
    }
  }
  if (!best) {
    diagnostic_ += "last seed rejected: " + rejection;
    return false;
  }
  RuneEstimate seed = best->state;
  seed.prediction_method = big_rune_ ? RunePredictionMethod::weighted_least_squares
                                     : config_.prediction_method;
  seed.start_timestamp = timestamp;
  seed.timestamp = timestamp;
  inactive_timeout_.fill(Timestamp{});
  state_ = seed;
  ekf_state_ = ekf_state_from(seed);
  last_inactive_corrected_ = timestamp;
  covariance_.diagonal() << 64, 64, 64, 100, 25, 10;
  correct_initial_observation(ekf_state_, covariance_, 0, best->icon_pixel,
                               R_camera2world, t_camera2world, camera_matrix_,
                               distort_coeffs_, config_.noise_observation);
  correct_initial_observation(ekf_state_, covariance_, 1, best->seed_pixel,
                               R_camera2world, t_camera2world, camera_matrix_,
                               distort_coeffs_, config_.noise_observation);
  ekf_state_into(*state_, ekf_state_);
  fitter_.reset();
  diagnostic_ = "initialized";
  return true;
}

void NjustRuneModel::update_motion_fit(RuneEstimate & state, double elapsed_seconds)
{
  fitter_.push(elapsed_seconds, state.rotation_angle);
  const auto linear = fitter_.fit_linear();
  const auto sine = big_rune_ ? fitter_.fit_sine() : std::nullopt;
  if (sine && (!linear || state.timestamp < force_sine_until_ ||
               (sine->cost < linear->cost && sine->a >= 0.6))) {
    state.sine_v = sine->v;
    state.sine_a = sine->a;
    state.sine_omega = sine->omega;
    state.sine_phase = sine->omega * elapsed_seconds + sine->phi;
    state.sine_t = elapsed_seconds;
    state.sine_valid = true;
    state.has_fitted_motion = false;
    state.motion_fit_cost = sine->cost;
    state.rotation_speed = sine->v + sine->a * std::sin(state.sine_phase);
  } else if (linear) {
    state.fitted_rotation_speed = linear->speed;
    state.rotation_speed = linear->speed;
    state.has_fitted_motion = true;
    state.sine_valid = false;
    state.motion_fit_cost = linear->cost;
  }
}

bool NjustRuneModel::diverged() const
{
  if (!ekf_state_.all_finite() || !covariance_.allFinite()) return true;
  if (covariance_(0, 0) > 150 || covariance_(1, 1) > 150 ||
      std::abs(ekf_state_.center_x()) > 15 || std::abs(ekf_state_.center_y()) > 15 ||
      std::abs(ekf_state_.center_z()) > 5 ||
      std::abs(ekf_state_.rotation_speed()) > 10 * kPi)
    return true;
  const double radius = std::hypot(ekf_state_.center_x(), ekf_state_.center_y());
  if (radius > 0.5) {
    const double to_center = std::atan2(ekf_state_.center_y(), ekf_state_.center_x());
    if (std::abs(normalize_angle(ekf_state_.face_yaw() - to_center)) >
        config_.diverge_face_angle * kPi / 180) return true;
  }
  return false;
}
}  // namespace auto_buff
