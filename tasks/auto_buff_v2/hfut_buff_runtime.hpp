#ifndef HFUT_AUTO_BUFF_V2__HFUT_BUFF_RUNTIME_HPP
#define HFUT_AUTO_BUFF_V2__HFUT_BUFF_RUNTIME_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "buff_config.hpp"
#include "detectors/rune_detector.hpp"
#include "models/rune_model.hpp"
#include "planners/buff_planner_factory.hpp"
#include "io/gimbal/gimbal.hpp"

namespace hfut::buff
{
struct FrameResult
{
  std::optional<auto_buff_v2::RuneEstimate> target;
  std::vector<auto_buff_v2::RuneReprojectedFeature> reprojected_features;
  std::optional<cv::Point2f> reprojected_center;
  std::string diagnostic;
  std::size_t candidate_count = 0;
  std::size_t bullseye_count = 0;
  std::size_t icon_count = 0;
};

class Runtime
{
public:
  Runtime(const std::string & config_path, bool big_rune);

  FrameResult process(const cv::Mat & image, auto_buff_v2::Timestamp timestamp,
                      const Eigen::Quaterniond & q_gimbal2world, bool enemy_red);

  auto_aim::Plan plan(double bullet_speed, const ::io::GimbalState & gimbal,
                      auto_buff_v2::Timestamp now);

  std::uint64_t target_generation() const { return target_generation_; }
  const auto_buff_v2::RuneElements & elements() const { return elements_; }

private:
  auto_buff_v2::BuffConfig config_;
  std::unique_ptr<auto_buff_v2::RuneModel> model_;
  std::unique_ptr<auto_buff_v2::BuffPlanStrategy> planner_;
  std::unique_ptr<auto_buff_v2::RuneDetector> detector_;
  bool big_rune_ = false;
  bool had_target_ = false;
  std::uint64_t target_generation_ = 0;
  auto_buff_v2::RuneElements elements_;
};
}  // namespace hfut::buff

#endif  // HFUT_AUTO_BUFF_V2__HFUT_BUFF_RUNTIME_HPP
