#include "rune_energy_fitter.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>

namespace auto_buff
{

RuneEnergyFitter::RuneEnergyFitter(BuffConfig::MotionFit config) : config_(config)
{
  if (!std::isfinite(config_.window_seconds) || config_.window_seconds <= 0 ||
      !std::isfinite(config_.min_fit_seconds) || config_.min_fit_seconds <= 0 ||
      !std::isfinite(config_.weight_half_life_seconds) || config_.weight_half_life_seconds <= 0 ||
      config_.min_fit_seconds > config_.window_seconds)
    throw std::invalid_argument(
      "buff.motion_fit: durations must be finite and positive, min_fit_seconds <= window_seconds");
}

double RuneEnergyFitter::sample_weight(double t, double reference_t) const
{
  return std::exp2((t - reference_t) / config_.weight_half_life_seconds);
}

void RuneEnergyFitter::push(double t, double theta)
{
  buffer_.push_back({t, theta});

  while (!buffer_.empty() && buffer_.back().t - buffer_.front().t > config_.window_seconds)
    buffer_.pop_front();
}

void RuneEnergyFitter::reset() { buffer_.clear(); }

template <typename Pred>
double RuneEnergyFitter::compute_weighted_cost(
  const std::deque<Point> & buffer, Pred && pred_fn) const
{

  const auto reference_t = buffer.back().t;
  double weighted_sum    = 0.0;
  double weight_sum      = 0.0;
  for (const auto & pt : buffer) {
    const auto weight = sample_weight(pt.t, reference_t);
    const auto diff   = pt.theta - pred_fn(pt.t);
    weighted_sum += weight * diff * diff;
    weight_sum += weight;
  }
  return weighted_sum / weight_sum;
}

std::optional<RuneEnergyFitter::LinearFitResult> RuneEnergyFitter::fit_linear() const
{
  if (buffer_.size() < 2) return std::nullopt;
  if (buffer_.back().t - buffer_.front().t < config_.min_fit_seconds) return std::nullopt;

  const auto reference_t = buffer_.back().t;

  double sum_weight = 0.0, sum_t = 0.0, sum_theta = 0.0, sum_tt = 0.0, sum_t_theta = 0.0;
  for (const auto & pt : buffer_) {
    const auto weight = sample_weight(pt.t, reference_t);
    sum_weight += weight;
    sum_t += weight * pt.t;
    sum_theta += weight * pt.theta;
    sum_tt += weight * pt.t * pt.t;
    sum_t_theta += weight * pt.t * pt.theta;
  }

  double denom = sum_weight * sum_tt - sum_t * sum_t;
  if (std::abs(denom) < 1e-12) return std::nullopt;

  double speed = (sum_weight * sum_t_theta - sum_t * sum_theta) / denom;
  double C     = (sum_theta - speed * sum_t) / sum_weight;

  auto pred   = [&](double dt) { return C + speed * dt; };
  double cost = compute_weighted_cost(buffer_, pred);

  return LinearFitResult{C, speed, cost};
}

std::optional<RuneEnergyFitter::SineFitResult> RuneEnergyFitter::fit_sine() const
{
  if (buffer_.size() < 2) return std::nullopt;
  if (buffer_.back().t - buffer_.front().t < config_.min_fit_seconds) return std::nullopt;

  const auto N = static_cast<Eigen::Index>(buffer_.size());
  Eigen::MatrixXd X(N, 4);
  Eigen::VectorXd y(N);
  Eigen::VectorXd sqrt_weights(N);
  const auto reference_t = buffer_.back().t;
  double weight_sum      = 0.0;
  for (Eigen::Index i = 0; i < N; ++i) {
    const auto t      = buffer_[i].t;
    const auto weight = sample_weight(t, reference_t);
    y(i)              = buffer_[i].theta;
    X(i, 0)           = 1.0;
    X(i, 1)           = t;
    sqrt_weights(i)   = std::sqrt(weight);
    weight_sum += weight;
  }
  const auto weighted_y = y.cwiseProduct(sqrt_weights);

  static constexpr int kSweepSteps  = 41;
  static constexpr double kOmegaMin = 1.80;
  static constexpr double kOmegaMax = 2.20;

  double best_mse = std::numeric_limits<double>::max();
  double best_C = 0, best_v = 0, best_A = 0, best_B = 0, best_omega = 0;

  for (int s = 0; s <= kSweepSteps; ++s) {
    double omega = kOmegaMin + s * (kOmegaMax - kOmegaMin) / kSweepSteps;

    for (Eigen::Index i = 0; i < N; ++i) {
      double t = buffer_[i].t;
      X(i, 2)  = std::cos(omega * t);
      X(i, 3)  = std::sin(omega * t);
    }

    auto weighted_X = Eigen::MatrixXd{X};
    for (Eigen::Index i = 0; i < N; ++i)
      weighted_X.row(i) *= sqrt_weights(i);

    Eigen::Vector4d coeff = weighted_X.colPivHouseholderQr().solve(weighted_y);
    double mse            = (weighted_y - weighted_X * coeff).squaredNorm() / weight_sum;
    if (mse < best_mse) {
      best_mse   = mse;
      best_C     = coeff(0);
      best_v     = coeff(1);
      best_A     = coeff(2);
      best_B     = coeff(3);
      best_omega = omega;
    }
  }

  double a   = best_omega * std::sqrt(best_A * best_A + best_B * best_B);
  double phi = std::atan2(best_B, -best_A);

  auto pred = [&](double t) {
    return best_C + best_v * t - a / best_omega * std::cos(best_omega * t + phi);
  };
  double cost = compute_weighted_cost(buffer_, pred);

  return SineFitResult{best_C, best_v, a, best_omega, phi, cost};
}

}  // namespace auto_buff
