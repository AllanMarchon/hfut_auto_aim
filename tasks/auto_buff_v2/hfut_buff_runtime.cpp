#include "hfut_buff_runtime.hpp"

#include "detectors/rune_detector_factory.hpp"
#include "models/rune_model_factory.hpp"

namespace hfut::buff
{
Runtime::Runtime(const std::string & config_path, bool big_rune)
: config_(auto_buff_v2::BuffConfig::load(config_path)), big_rune_(big_rune),
  model_(auto_buff_v2::make_rune_model(config_.camera, config_.model, big_rune)),
  planner_(auto_buff_v2::make_buff_planner(config_.planner, config_path)),
  detector_(auto_buff_v2::make_rune_detector(config_.detector))
{
}

FrameResult Runtime::process(
  const cv::Mat & image, auto_buff_v2::Timestamp timestamp,
  const Eigen::Quaterniond & q_gimbal2world, bool enemy_red)
{
  FrameResult result;
  detector_->set_enemy_red(enemy_red);
  model_->set_q_gimbal2world(q_gimbal2world);
  elements_ = detector_->detect(image);
  result.candidate_count = elements_.candidates.size();
  result.bullseye_count = elements_.bullseyes.size();
  result.icon_count = elements_.icons.size();
  model_->update(elements_, timestamp);
  result.target = model_->state();
  result.reprojected_features = model_->reprojected_features();
  result.reprojected_center = model_->reprojected_center();
  result.diagnostic = model_->diagnostic();
  const bool has_target = result.target.has_value();
  if (has_target && !had_target_) ++target_generation_;
  had_target_ = has_target;
  return result;
}

auto_aim::Plan Runtime::plan(
  double bullet_speed, const ::io::GimbalState & gimbal, auto_buff_v2::Timestamp now)
{
  return planner_->plan(target_generation_, model_->state(), bullet_speed, gimbal, now);
}
}  // namespace hfut::buff
