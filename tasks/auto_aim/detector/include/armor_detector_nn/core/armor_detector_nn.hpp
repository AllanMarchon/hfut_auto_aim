#ifndef ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_HPP_
#define ARMOR_DETECTOR_NN_ARMOR_DETECTOR_NN_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/corner_refine/icorner_refiner.hpp"
#include "armor_detector_nn/core/preprocessor.hpp"
#include "armor_detector_nn/backend/inference_backend.hpp"
#include "armor_detector_nn/postprocess/decode_strategy.hpp"
#include "armor_detector_nn/core/label_map.hpp"
#include "armor_detector_nn/core/tracker/itracker_strategy.hpp"
#include "armor_detector_nn/debug/profiler.hpp"

namespace fyt::auto_aim {

class ArmorDetectorNN {
public:
  explicit ArmorDetectorNN(const DetectorConfig& config);
  ~ArmorDetectorNN();

  bool initialize();

  std::vector<FrameDetections> detectBatch(
    const std::vector<cv::Mat>& images,
    const std::vector<std_msgs::msg::Header>& headers);

  bool isInitialized() const { return initialized_; }

  void setTargetColor(fyt::EnemyColor color) { target_color_ = color; }

  const DetectorConfig& config() const { return config_; }
  BackendInfo backendInfo() const;
  const Preprocessor* preprocessor() const { return preprocessor_.get(); }
  const ProfilerEntry& lastProfiler() const { return last_profile_; }

private:
  DetectorConfig config_;
  std::unique_ptr<Preprocessor> preprocessor_;
  std::unique_ptr<IInferenceBackend> backend_;
  std::unique_ptr<IDecodeStrategy> decode_strategy_;
  std::unique_ptr<LabelMap> label_map_;
  std::unique_ptr<ITrackerStrategy> tracker_;
  std::unique_ptr<ICornerRefiner> corner_refiner_;
  fyt::EnemyColor target_color_{fyt::EnemyColor::RED};
  ProfilerEntry last_profile_;
  bool initialized_{false};
  // Corner-refine acceptance counters for acceptance-rate auditing.
  int corner_refine_ok_{0};
  int corner_refine_fail_{0};
  // Per-2D-track EMA of (NN quad extent / refined quad extent). The classical
  // refit measures the light bars, whose extent differs from the plate
  // outline the NN was trained to mark (inset bars shrink the quad and bias
  // PnP depth). Scaling refined corners by this EMA keeps the refit's stable
  // shape while matching the NN's physically-trained extent, without any
  // per-environment constant.
  std::unordered_map<int, double> extent_ratio_ema_;
};

}  // namespace fyt::auto_aim

#endif
