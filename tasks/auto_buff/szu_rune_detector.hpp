#ifndef AUTO_BUFF__SZU_RUNE_DETECTOR_HPP
#define AUTO_BUFF__SZU_RUNE_DETECTOR_HPP

#include <array>
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
    cv::Point2f network_r_center{0.0f, 0.0f};
    cv::Point2f geometric_r_center{0.0f, 0.0f};
    cv::Point2f visual_r_center{0.0f, 0.0f};
    float network_r_confidence = 0.0f;
    bool network_r_valid = false;
    bool geometric_r_valid = false;
    bool visual_r_valid = false;
    std::vector<cv::Point2f> corners;
    std::vector<int> corner_indices;
    std::vector<float> keypoint_confidences;
    // 记录传统阶段是否找到了局部边缘和几何中心，便于区分网络结果与精修结果。
    int traditional_valid_corners = 0;
    bool traditional_geometry_valid = false;
    bool traditional_r_refined = false;
  };

  struct DebugStats
  {
    int anchors = 0;
    int confidence_pass = 0;
    int keypoint_pass = 0;
    int required_keypoint_pass = 0;
    int nms_output = 0;
    int traditional_attempted = 0;
    int traditional_edge_refined = 0;
    int traditional_corner_refined = 0;
    int traditional_geometry_pass = 0;
    int traditional_geometry_rejected = 0;
    int traditional_r_refined = 0;
    int traditional_r_geometry = 0;
    int traditional_corner_fallback = 0;
    std::array<int, 3> class_counts{0, 0, 0};
    float max_confidence = 0.0f;
    float max_keypoint_confidence = 0.0f;
  };

  explicit SzuRuneDetector(const std::string & config_path);

  std::vector<Detection> detect(const cv::Mat & image);

  const DebugStats & debug_stats() const { return debug_stats_; }

private:
  void preprocess(
    const cv::Mat & src, cv::Mat & dst, float & scale, int & pad_w, int & pad_h,
    int & crop_x, int & crop_y) const;
  std::vector<Detection> postprocess(
    float scale, int pad_w, int pad_h, int crop_x, int crop_y, int orig_w, int orig_h);
  std::vector<Detection> nms(std::vector<Detection> & detections) const;
  void refine_detections(const cv::Mat & image, std::vector<Detection> & detections);
  bool refine_radial_edge(
    const cv::Mat & gray, const cv::Mat & gradient_x, const cv::Mat & gradient_y,
    const cv::Point2f & r_center, const cv::Point2f & seed, cv::Point2f & refined) const;
  bool refine_corner_subpix(
    const cv::Mat & gray, const cv::Mat & gradient_x, const cv::Mat & gradient_y,
    const cv::Point2f & seed, cv::Point2f & refined) const;
  bool estimate_geometric_r_center(
    const std::vector<cv::Point2f> & corners, cv::Point2f & r_center) const;
  bool refine_visual_r_center(
    const cv::Mat & gray, const cv::Point2f & seed, const std::vector<cv::Point2f> & corners,
    cv::Point2f & refined) const;
  double corner_strength(
    const cv::Mat & gradient_x, const cv::Mat & gradient_y, const cv::Point2f & point) const;

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;

  std::string device_{"CPU"};
  std::string preprocess_mode_{"letterbox"};
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
  std::vector<int> required_keypoint_indices_{0, 1, 3, 4};
  float confidence_threshold_{0.8f};
  float keypoint_confidence_threshold_{0.8f};
  float nms_distance_threshold_{30.0f};
  int min_valid_keypoints_{4};
  int traditional_min_valid_keypoints_{3};
  bool traditional_refine_enabled_{true};
  bool traditional_r_refine_enabled_{true};
  int traditional_corner_window_{5};
  float traditional_max_shift_px_{8.0f};
  float traditional_r_max_shift_px_{24.0f};
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  DebugStats debug_stats_;
};

}  // namespace auto_buff

#endif  // AUTO_BUFF__SZU_RUNE_DETECTOR_HPP
