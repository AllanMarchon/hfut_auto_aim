#include "rune_predictor.hpp"
#include "rune_ekf_state.hpp"

#include <cmath>

namespace auto_buff_v2
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

RuneEstimate RunePredictor::predict(const RuneEstimate & estimate,
                                    Timestamp prediction_time) const
{
  RuneEstimate predicted = estimate;
  if (prediction_time <= estimate.timestamp) return predicted;
  const double seconds =
    std::chrono::duration<double>(prediction_time - estimate.timestamp).count();
  predicted.center = Eigen::AngleAxisd(estimate.center_yaw_speed * seconds,
    Eigen::Vector3d::UnitZ()) * estimate.center;
  if (predicted.prediction_method == RunePredictionMethod::ekf) {
    RuneEkfState::Vector values;
    values << estimate.center.x(), estimate.center.y(), estimate.center.z(),
      estimate.rotation_speed, estimate.rotation_angle, estimate.face_yaw;
    const auto future = RuneEkfState(values).predict(seconds);
    predicted.rotation_angle = future.rotation_angle();
  } else if (predicted.sine_valid) {
    const auto old_phase = predicted.sine_phase;
    predicted.sine_t += seconds;
    predicted.sine_phase += predicted.sine_omega * seconds;
    if (std::abs(predicted.sine_omega) > 1e-12) {
      predicted.rotation_angle +=
        predicted.sine_v * seconds + predicted.sine_a / predicted.sine_omega *
                                      (std::cos(old_phase) - std::cos(predicted.sine_phase));
    } else {
      predicted.rotation_angle += predicted.rotation_speed * seconds;
    }
    predicted.rotation_speed = predicted.sine_v + predicted.sine_a * std::sin(predicted.sine_phase);
  } else {
    predicted.rotation_angle += predicted.rotation_speed * seconds;
  }
  predicted.timestamp = prediction_time;
  return predicted;
}

std::optional<Eigen::Vector3d> RunePredictor::aimpoint_at(
  const RuneEstimate & estimate, Timestamp prediction_time) const
{
  const auto delay = std::chrono::seconds(estimate.sine_valid ? 6 : 3);
  if (prediction_time < estimate.timestamp ||
      (estimate.requires_warmup && prediction_time - estimate.start_timestamp < delay))
    return std::nullopt;
  const RuneEstimate predicted = predict(estimate, prediction_time);
  for (std::size_t i = 0; i < predicted.inactive.size(); ++i) {
    if (!predicted.inactive[i]) continue;
    const double angle = predicted.rotation_angle + i * 2 * kPi / 5;
    const Eigen::Vector3d local(0, -kRuneGlobalRadius * std::sin(angle),
                                kRuneGlobalRadius * std::cos(angle));
    return predicted.center +
           Eigen::AngleAxisd(predicted.face_yaw, Eigen::Vector3d::UnitZ()) * local;
  }
  return std::nullopt;
}
}  // namespace auto_buff_v2
