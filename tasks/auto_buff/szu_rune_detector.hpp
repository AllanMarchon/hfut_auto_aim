#ifndef AUTO_BUFF__SZU_RUNE_DETECTOR_HPP
#define AUTO_BUFF__SZU_RUNE_DETECTOR_HPP

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <yaml-cpp/yaml.h>

namespace auto_buff
{

class SzuRuneDetector
{
public:
  struct Detection
  {
    int class_id = -1;
    float confidence = 0.0f;
    float quality = 0.0f;
    cv::Point2f center{0.0f, 0.0f};
    cv::Point2f r_center{0.0f, 0.0f};
    std::vector<cv::Point2f> corners;
    std::vector<float> keypoint_confidences;
  };

  explicit SzuRuneDetector(const std::string & config_path);

  std::vector<Detection> detect(const cv::Mat & image);

private:
  void preprocess_letterbox(
    const cv::Mat & src, cv::Mat & dst, float & scale, int & pad_w, int & pad_h) const;
  std::vector<Detection> postprocess(float scale, int pad_w, int pad_h, int orig_w, int orig_h);
  std::vector<Detection> nms(std::vector<Detection> & detections) const;

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;

  std::string device_{"CPU"};
  int input_width_{640};
  int input_height_{480};
  int output_channels_{0};
  int num_anchors_{0};
  bool output_layout_nca_{true};

  int num_classes_{3};
  int num_keypoints_{5};
  int keypoint_dim_{3};
  std::vector<int> corner_indices_{0, 1, 3, 4};
  int r_center_index_{2};
  float confidence_threshold_{0.8f};
  float keypoint_confidence_threshold_{0.8f};
  float nms_distance_threshold_{30.0f};
  int min_valid_keypoints_{5};
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__SZU_RUNE_DETECTOR_HPP
