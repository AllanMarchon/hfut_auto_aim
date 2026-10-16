#ifndef AUTO_BUFF__RUNE_ENERGY_FITTER_HPP
#define AUTO_BUFF__RUNE_ENERGY_FITTER_HPP

#include <deque>
#include <limits>
#include <optional>

#include "buff_config.hpp"

namespace auto_buff
{

class RuneEnergyFitter
{
public:
  RuneEnergyFitter() = default;
  explicit RuneEnergyFitter(BuffConfig::MotionFit config);

  struct SineFitResult
  {
    double C = 0;
    double v = 0;
    double a = 0;
    double omega = 0;
    double phi = 0;
    double cost = std::numeric_limits<double>::max();
  };

  struct LinearFitResult
  {
    double C = 0;
    double speed = 0;
    double cost = std::numeric_limits<double>::max();
  };

  void push(double t, double theta);
  void reset();

  std::optional<LinearFitResult> fit_linear() const;
  std::optional<SineFitResult> fit_sine() const;

private:
  BuffConfig::MotionFit config_;
  struct Point
  {
    double t;
    double theta;
  };
  std::deque<Point> buffer_;

  template <typename Pred>
  double compute_weighted_cost(const std::deque<Point> & buffer, Pred && pred_fn) const;
  double sample_weight(double t, double reference_t) const;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_ENERGY_FITTER_HPP
