#include "extended_kalman_filter.hpp"

#include <cmath>
#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  // 预测矩阵异常时保留上一帧，避免把非法协方差传给后续更新。
  if (x.size() == 0 || !x.allFinite() || !P.allFinite() || !F.allFinite() || !Q.allFinite() ||
      F.rows() != x.size() || F.cols() != x.size() || Q.rows() != x.size() ||
      Q.cols() != x.size()) {
    return x;
  }
  const Eigen::MatrixXd P_predicted = F * P * F.transpose() + Q;
  const Eigen::VectorXd x_predicted = f(x);
  if (!P_predicted.allFinite() || !x_predicted.allFinite()) return x;
  P = P_predicted;
  x = x_predicted;
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  // 输入或预测状态异常时保留上一帧，避免坏观测污染整个滤波器。
  if (x.size() == 0 || !x.allFinite() || !P.allFinite() || !z.allFinite() || !H.allFinite() ||
      !R.allFinite()) {
    return x;
  }

  Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd S = H * P * H.transpose() + R;
  if (!S.allFinite() || S.rows() == 0 || S.rows() != S.cols()) return x;
  Eigen::LDLT<Eigen::MatrixXd> S_factor(S);
  if (S_factor.info() != Eigen::Success) return x;
  Eigen::MatrixXd K = P * H.transpose() * S_factor.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
  if (!K.allFinite()) return x;

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  const Eigen::MatrixXd P_updated =
    (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
  if (!P_updated.allFinite()) return x;

  const Eigen::VectorXd residual_prior = z_subtract(z, h(x));
  if (!residual_prior.allFinite()) return x;
  const Eigen::VectorXd x_updated = x_add(x, K * residual_prior);
  if (!x_updated.allFinite()) return x;
  P = P_updated;
  x = x_updated;

  /// 卡方检验
  Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验
  const Eigen::MatrixXd S_updated = H * P * H.transpose() + R;
  Eigen::LDLT<Eigen::MatrixXd> S_updated_factor(S_updated);
  double nis = 0.0;
  if (S_updated.allFinite() && S_updated_factor.info() == Eigen::Success) {
    const Eigen::VectorXd nis_solution = S_updated_factor.solve(residual);
    if (nis_solution.allFinite()) nis = residual.transpose() * nis_solution;
  }
  double nees = 0.0;
  Eigen::LDLT<Eigen::MatrixXd> P_factor(P);
  if (P_factor.info() == Eigen::Success) {
    const Eigen::VectorXd nees_solution = P_factor.solve(x - x_prior);
    if (nees_solution.allFinite()) nees = (x - x_prior).transpose() * nees_solution;
  }

  // 卡方检验阈值（自由度=4，取置信水平95%）
  constexpr double nis_threshold = 0.711;
  constexpr double nees_threshold = 0.711;

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual.size() > 0 ? residual[0] : 0.0;
  data["residual_pitch"] = residual.size() > 1 ? residual[1] : 0.0;
  data["residual_distance"] = residual.size() > 2 ? residual[2] : 0.0;
  data["residual_angle"] = residual.size() > 3 ? residual[3] : 0.0;
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
