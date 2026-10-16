#ifndef AUTO_BUFF_V2__CLIMBER_RUNE_DETECTOR_HPP
#define AUTO_BUFF_V2__CLIMBER_RUNE_DETECTOR_HPP

#include <openvino/openvino.hpp>
#include <yaml-cpp/yaml.h>

#include "rune_detector.hpp"

namespace auto_buff_v2
{
// Climber_Vision_26 的 YOLO11-pose 检测与 R 标轮廓细化。
class ClimberRuneDetector : public RuneDetector
{
public:
  explicit ClimberRuneDetector(const YAML::Node & config);
  void set_enemy_red(bool enemy_red) override { enemy_red_ = enemy_red; }
  RuneElements detect(const cv::Mat & image) override;

private:
  using Candidate = RuneCandidate;

  std::vector<Candidate> infer(const cv::Mat & image);
  cv::Point2f refine_center(
    const cv::Mat & image, const std::vector<Candidate> & candidates,
    std::vector<std::vector<cv::Point>> & contours);

  ov::Core core_;
  ov::CompiledModel model_;
  ov::InferRequest request_;
  ov::Tensor input_;
  cv::Size input_size_;
  bool enemy_red_ = false;
  float confidence_threshold_ = 0.2;
  float iou_threshold_ = 0.2;
  int binary_threshold_ = 100;
  int dilate_kernel_size_ = 5;
  int dilate_iterations_ = 1;
  double r_center_mask_ratio_ = 0.8;
  double r_center_contour_div_ = 3.0;
  double r_center_refine_max_shift_ = 5.0;
  double dedup_distance_ = 18.0;
  int max_bullseyes_ = 2;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__CLIMBER_RUNE_DETECTOR_HPP
