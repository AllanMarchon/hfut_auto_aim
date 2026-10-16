#ifndef AUTO_BUFF__SP_RUNE_MODEL_HPP
#define AUTO_BUFF__SP_RUNE_MODEL_HPP

#include <memory>

#include "rune_model.hpp"
#include "sp_rune_config.hpp"
#include "sp_rune_solver.hpp"
#include "sp_rune_tracker.hpp"

namespace auto_buff
{
// 将独立运动模型估计适配到 buff 规划接口。
class SPRuneModel final : public RuneModel
{
public:
  using Config = SPRuneConfig;
  SPRuneModel(BuffConfig::Camera camera, Config config, bool big_rune);
  void set_q_gimbal2world(const Eigen::Quaterniond & q) override;
  bool update(const RuneElements & elements, Timestamp timestamp) override;
  void reset() override;
  std::optional<RuneEstimate> state() const override { return state_; }
  const std::string & diagnostic() const override { return diagnostic_; }
  std::vector<RuneReprojectedFeature> reprojected_features() const override;
  std::optional<cv::Point2f> reprojected_center() const override;

private:
  SPRuneSolver solver_;
  Config config_;
  bool big_rune_;
  std::unique_ptr<SPTracker> tracker_;
  std::optional<RuneEstimate> state_;
  std::string diagnostic_;
};
}  // namespace auto_buff

#endif
