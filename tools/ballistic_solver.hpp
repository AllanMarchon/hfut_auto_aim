#ifndef TOOLS__BALLISTIC_SOLVER_HPP
#define TOOLS__BALLISTIC_SOLVER_HPP

#include <memory>
#include <optional>
#include <string>

namespace tools
{
struct BallisticSolution
{
  double pitch = 0;
  double fly_time = 0;
};

struct BallisticImpact
{
  double height = 0;
  double fly_time = 0;
};

struct BallisticSolverConfig
{
  double njust_air_resistance = 0.003;
  double climber_air_resistance_k = 0;
};

class BallisticSolver
{
public:
  virtual ~BallisticSolver() = default;
  virtual std::optional<BallisticSolution> solve(double speed, double distance,
                                                  double height) const = 0;
  // 计算指定水平距离处的弹道高度；pitch 向上为正。
  virtual std::optional<BallisticImpact> impact(
    double speed, double distance, double pitch) const = 0;
};

std::unique_ptr<BallisticSolver> make_ballistic_solver(
  const std::string & model, const BallisticSolverConfig & config = {});
}  // namespace tools

#endif  // TOOLS__BALLISTIC_SOLVER_HPP
