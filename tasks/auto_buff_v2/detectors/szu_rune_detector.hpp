#ifndef AUTO_BUFF_V2__SZU_RUNE_DETECTOR_HPP
#define AUTO_BUFF_V2__SZU_RUNE_DETECTOR_HPP

#include <openvino/openvino.hpp>
#include <yaml-cpp/yaml.h>

#include "rune_detector.hpp"

namespace auto_buff_v2
{
// 本检测模块由 深圳大学 https://github.com/SZURPVision/RuneDetectionModel.git
// 和 https://github.com/SZURPVision/RP-26Rune.git 修改而来
class SzuRuneDetector : public RuneDetector
{
public:
  explicit SzuRuneDetector(const YAML::Node & config);
  void set_enemy_red(bool enemy_red) override { enemy_red_ = enemy_red; }
  RuneElements detect(const cv::Mat & image) override;

private:
  struct Candidate
  {
    std::array<cv::Point2f, 5> points;  // 上、左、R 标、右、下。
    cv::Point2f center;
    float confidence = 0;
    float quality = 0;
    bool active = false;
  };

  std::vector<Candidate> infer(const cv::Mat & image);
  // 依据网络关键点精修靶心与 R 标，验证失败时返回 false
  bool refine(const cv::Mat & image, const Candidate & candidate,
              RuneBullseye & bull, RuneIcon & icon) const;

  ov::Core core_;
  ov::CompiledModel model_;
  ov::InferRequest request_;
  ov::Tensor input_;
  cv::Size input_size_;
  bool output_nca_ = true;
  bool enemy_red_ = false;
  float confidence_threshold_ = 0.65;
  float keypoint_confidence_threshold_ = 0.5;
  float nms_distance_ = 30;
  int min_valid_keypoints_ = 5;
  int letterbox_value_ = 114;
  double roi_scale_ = 1.4;
  int gaussian_kernel_size_ = 5;
  double gaussian_sigma_ = 0;
  double red_threshold_ = 50;
  double blue_threshold_ = 62;
  double armor_area_error_ = 0.35;
  double armor_solidity_ = 0.8;
  int armor_border_margin_ = 2;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__SZU_RUNE_DETECTOR_HPP
