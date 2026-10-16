#ifndef AUTO_BUFF_V2__HUNGARIAN_HPP
#define AUTO_BUFF_V2__HUNGARIAN_HPP

#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

namespace auto_buff_v2
{
template <typename Scalar>
std::vector<int> hungarian(
  const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & cost)
{
  if (cost.rows() != cost.cols()) throw std::invalid_argument("hungarian requires a square matrix");
  const int n = cost.rows();
  if (n == 0) return {};
  std::vector<Scalar> u(n + 1, 0), v(n + 1, 0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<Scalar> minimum(n + 1, std::numeric_limits<Scalar>::max());
    std::vector<char> used(n + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      Scalar delta = std::numeric_limits<Scalar>::max();
      int j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (used[j]) continue;
        const Scalar current = cost(i0 - 1, j - 1) - u[i0] - v[j];
        if (current < minimum[j]) {
          minimum[j] = current;
          way[j] = j0;
        }
        if (minimum[j] < delta) {
          delta = minimum[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minimum[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  std::vector<int> result(n, -1);
  for (int j = 1; j <= n; ++j)
    if (p[j] != 0) result[p[j] - 1] = j - 1;
  return result;
}

template <typename Scalar>
std::vector<std::optional<int>> hungarian_assign(
  const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & cost,
  Scalar gate_threshold)
{
  const int N = cost.rows();
  const int M = cost.cols();
  if (N == 0) return {};
  if (M == 0) return std::vector<std::optional<int>>(N, std::nullopt);
  const int K = N + M;
  const Scalar huge = std::numeric_limits<Scalar>::max() / 4;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> augmented(K, K);
  augmented.setConstant(huge);
  augmented.topLeftCorner(N, M) = cost;
  for (int i = 0; i < N; ++i)
    for (int j = M; j < K; ++j)
      augmented(i, j) = (j - M == i) ? gate_threshold : huge;
  for (int i = N; i < K; ++i)
    for (int j = 0; j < M; ++j)
      augmented(i, j) = (i - N == j) ? 0 : huge;
  augmented.bottomRightCorner(M, N).setZero();
  const auto assignment = hungarian(augmented);
  std::vector<std::optional<int>> result(N, std::nullopt);
  for (int i = 0; i < N; ++i)
    if (assignment[i] >= 0 && assignment[i] < M) result[i] = assignment[i];
  return result;
}
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__HUNGARIAN_HPP
