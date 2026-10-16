#include "ballistic_solver.hpp"

#include <cmath>
#include <stdexcept>

#include "trajectory.hpp"

namespace tools
{
namespace
{
bool valid_shot(double speed, double distance, double pitch)
{
  return std::isfinite(speed) && std::isfinite(distance) && std::isfinite(pitch) &&
         speed > 0 && distance > 0 && std::abs(pitch) < std::acos(-1) / 2;
}

class NjustBallisticSolver final : public BallisticSolver
{
public:
  explicit NjustBallisticSolver(double air_resistance) : air_resistance_(air_resistance) {}

  std::optional<BallisticSolution> solve(double speed, double distance,
                                          double height) const override
  {
    if (speed <= 0 || distance <= 0) return std::nullopt;
    double pitch = std::atan2(height, distance);
    for (int iteration = 0; iteration < 10; ++iteration) {
      const auto shot = impact(speed, distance, pitch);
      if (!shot) return std::nullopt;
      const double error = height - shot->height;
      if (std::abs(error) < 0.001) return BallisticSolution{pitch, shot->fly_time};
      pitch += std::atan2(error, distance);
      if (std::abs(pitch) > 80.0 / 57.3) break;
    }
    return std::nullopt;
  }

  std::optional<BallisticImpact> impact(
    double speed, double distance, double pitch) const override
  {
    if (!valid_shot(speed, distance, pitch)) return std::nullopt;
    constexpr double kStep = 0.005;
    constexpr double kGravity = 9.81;
    double x = 0, y = 0, t = 0;
    double vx = speed * std::cos(pitch), vy = speed * std::sin(pitch);
    double previous_x = 0, previous_y = 0, previous_t = 0;
    while (x < distance) {
      previous_x = x;
      previous_y = y;
      previous_t = t;
      const double velocity = std::hypot(vx, vy);
      vx -= air_resistance_ * velocity * vx * kStep;
      vy -= (kGravity + air_resistance_ * velocity * vy) * kStep;
      x += vx * kStep;
      y += vy * kStep;
      t += kStep;
      if (t > 4 || vx <= 0.1) break;
    }
    if (x < distance || x <= previous_x) return std::nullopt;
    const double ratio = (distance - previous_x) / (x - previous_x);
    return BallisticImpact{
      previous_y + (y - previous_y) * ratio, previous_t + (t - previous_t) * ratio};
  }
private:
  double air_resistance_;
};

class ClimberBallisticSolver final : public BallisticSolver
{
public:
  explicit ClimberBallisticSolver(double air_resistance_k) : air_resistance_k_(air_resistance_k) {}

  // Climber 线性阻力模型：dv/dt = -k*v - g，选择 +/-60 度以内的低弹道。
  std::optional<BallisticSolution> solve(
    double speed, double distance, double height) const override
  {
    constexpr double kPi = 3.14159265358979323846;
    const double drag = air_resistance_k_;
    if (distance < 0.1 || !std::isfinite(speed) || !std::isfinite(distance) ||
        !std::isfinite(height)) return std::nullopt;
    const tools::Trajectory vacuum(speed, distance, height);
    if (vacuum.unsolvable || std::abs(vacuum.pitch) > kPi / 3) return std::nullopt;
    if (drag < 1e-6) return tools::BallisticSolution{vacuum.pitch, vacuum.fly_time};
    constexpr double gravity = 9.7833;
    const auto flight_time = [&](double pitch) {
      const double ratio = drag * distance / (speed * std::cos(pitch));
      return ratio < 0.99 ? -std::log1p(-ratio) / drag : NAN;
    };
    const auto residual = [&](double pitch) {
      const double time = flight_time(pitch);
      return (speed * std::sin(pitch) + gravity / drag) *
               distance / (speed * std::cos(pitch)) - gravity * time / drag - height;
    };
    // 先定位第一个根以选择低弹道，再对高度残差进行二分求根。
    double lower = -kPi / 3;
    for (int i = 1; i <= 128; ++i) {
      double upper = -kPi / 3 + i * (2 * kPi / 3) / 128;
      if (residual(lower) <= 0 && residual(upper) >= 0) {
        for (int j = 0; j < 60; ++j) {
          const double middle = (lower + upper) / 2;
          if (residual(middle) < 0) lower = middle;
          else upper = middle;
        }
        const double pitch = (lower + upper) / 2;
        return tools::BallisticSolution{pitch, flight_time(pitch)};
      }
      lower = upper;
    }
    return std::nullopt;
  }

  std::optional<BallisticImpact> impact(
    double speed, double distance, double pitch) const override
  {
    if (!valid_shot(speed, distance, pitch)) return std::nullopt;
    constexpr double gravity = 9.7833;
    const double vx = speed * std::cos(pitch);
    const double vy = speed * std::sin(pitch);
    const double drag = air_resistance_k_;
    if (drag < 1e-6) {
      const double time = distance / vx;
      return BallisticImpact{vy * time - gravity * time * time / 2, time};
    }
    const double ratio = drag * distance / vx;
    if (ratio >= 0.99) return std::nullopt;
    const double time = -std::log1p(-ratio) / drag;
    return BallisticImpact{
      (vy + gravity / drag) * distance / vx - gravity * time / drag, time};
  }

private:
  double air_resistance_k_;
};

class VacuumBallisticSolver final : public BallisticSolver
{
public:
  std::optional<BallisticSolution> solve(double speed, double distance,
                                          double height) const override
  {
    if (speed <= 0 || distance <= 0) return std::nullopt;
    const Trajectory trajectory(speed, distance, height);
    if (trajectory.unsolvable) return std::nullopt;
    return BallisticSolution{trajectory.pitch, trajectory.fly_time};
  }

  std::optional<BallisticImpact> impact(
    double speed, double distance, double pitch) const override
  {
    if (!valid_shot(speed, distance, pitch)) return std::nullopt;
    const double time = distance / (speed * std::cos(pitch));
    return BallisticImpact{speed * std::sin(pitch) * time - 9.7833 * time * time / 2, time};
  }
};
}  // namespace

std::unique_ptr<BallisticSolver> make_ballistic_solver(const std::string & model,
                                                        const BallisticSolverConfig & config)
{
  if (model == "njust") {
    if (!std::isfinite(config.njust_air_resistance) || config.njust_air_resistance <= 0) {
      throw std::invalid_argument("njust_air_resistance 必须为有限正数");
    }
    return std::make_unique<NjustBallisticSolver>(config.njust_air_resistance);
  }
  if (model == "climber") {
    if (!std::isfinite(config.climber_air_resistance_k) || config.climber_air_resistance_k < 0)
      throw std::invalid_argument("climber_air_resistance_k must be finite and >= 0");
    return std::make_unique<ClimberBallisticSolver>(config.climber_air_resistance_k);
  }
  if (model == "vacuum") return std::make_unique<VacuumBallisticSolver>();
  throw std::invalid_argument("未知弹道模型: " + model);
}
}  // namespace tools
