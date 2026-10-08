#include "szu_rune_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace auto_buff
{
namespace
{
float yaml_float(const YAML::Node & yaml, const char * key, float fallback)
{
  return yaml[key] ? yaml[key].as<float>() : fallback;
}

int yaml_int(const YAML::Node & yaml, const char * key, int fallback)
{
  return yaml[key] ? yaml[key].as<int>() : fallback;
}

std::vector<int> yaml_int_vector(
  const YAML::Node & yaml, const char * key, const std::vector<int> & fallback)
{
  return yaml[key] ? yaml[key].as<std::vector<int>>() : fallback;
}

void fill_nchw_rgb_float_tensor(const cv::Mat & bgr_image, ov::Tensor & input_tensor)
{
  const auto shape = input_tensor.get_shape();
  if (shape.size() != 4 || shape[1] != 3) {
    throw std::runtime_error("SZU 打符模型输入 tensor 不是 NCHW 三通道");
  }
  const int height = static_cast<int>(shape[2]);
  const int width = static_cast<int>(shape[3]);
  if (bgr_image.rows != height || bgr_image.cols != width || bgr_image.type() != CV_8UC3) {
    throw std::runtime_error("SZU 打符模型预处理尺寸或格式错误");
  }

  float * data = input_tensor.data<float>();
  const size_t plane_size = static_cast<size_t>(height) * static_cast<size_t>(width);
  for (int y = 0; y < height; ++y) {
    const auto * row = bgr_image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < width; ++x) {
      const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      data[offset] = static_cast<float>(row[x][2]) / 255.0f;
      data[plane_size + offset] = static_cast<float>(row[x][1]) / 255.0f;
      data[2 * plane_size + offset] = static_cast<float>(row[x][0]) / 255.0f;
    }
  }
}
}  // namespace

SzuRuneDetector::SzuRuneDetector(const std::string & config_path)
{
  const auto yaml = YAML::LoadFile(config_path);
  const std::string model_path = yaml["szu_model"] ? yaml["szu_model"].as<std::string>()
                                                    : yaml["model"].as<std::string>();
  device_ = yaml["szu_device"] ? yaml["szu_device"].as<std::string>()
                                : (yaml["device"] ? yaml["device"].as<std::string>() : device_);
  preprocess_mode_ = yaml["szu_preprocess_mode"]
                       ? yaml["szu_preprocess_mode"].as<std::string>()
                       : preprocess_mode_;
  confidence_threshold_ = yaml_float(yaml, "szu_confidence", confidence_threshold_);
  keypoint_confidence_threshold_ =
    yaml_float(yaml, "szu_keypoint_confidence", keypoint_confidence_threshold_);
  nms_distance_threshold_ = yaml_float(yaml, "szu_nms_distance", nms_distance_threshold_);
  min_valid_keypoints_ = yaml_int(yaml, "szu_min_valid_keypoints", min_valid_keypoints_);
  traditional_min_valid_keypoints_ = yaml_int(
    yaml, "szu_traditional_min_valid_keypoints", traditional_min_valid_keypoints_);
  traditional_refine_enabled_ = yaml["szu_traditional_refine"]
                                  ? yaml["szu_traditional_refine"].as<bool>()
                                  : traditional_refine_enabled_;
  traditional_r_refine_enabled_ = yaml["szu_traditional_r_refine"]
                                    ? yaml["szu_traditional_r_refine"].as<bool>()
                                    : traditional_r_refine_enabled_;
  rp26_refine_enabled_ = yaml["szu_rp26_refine"]
                           ? yaml["szu_rp26_refine"].as<bool>() : rp26_refine_enabled_;
  rp26_strict_ = yaml["szu_rp26_strict"]
                   ? yaml["szu_rp26_strict"].as<bool>() : rp26_strict_;
  rp26_roi_scale_ = yaml_float(yaml, "szu_rp26_roi_scale", rp26_roi_scale_);
  rp26_armor_area_relative_error_ = yaml_float(
    yaml, "szu_rp26_armor_area_relative_error", rp26_armor_area_relative_error_);
  rp26_armor_solidity_threshold_ = yaml_float(
    yaml, "szu_rp26_armor_solidity_threshold", rp26_armor_solidity_threshold_);
  rp26_light_solidity_threshold_ = yaml_float(
    yaml, "szu_rp26_light_solidity_threshold", rp26_light_solidity_threshold_);
  rp26_red_threshold_ = yaml_float(yaml, "szu_rp26_red_minus_blue_threshold", rp26_red_threshold_);
  rp26_blue_threshold_ = yaml_float(yaml, "szu_rp26_blue_minus_red_threshold", rp26_blue_threshold_);
  rp26_line_samples_ = yaml_int(yaml, "szu_rp26_line_samples", rp26_line_samples_);
  traditional_corner_window_ = yaml_int(
    yaml, "szu_traditional_corner_window", traditional_corner_window_);
  traditional_max_shift_px_ = yaml_float(
    yaml, "szu_traditional_max_shift_px", traditional_max_shift_px_);
  traditional_r_max_shift_px_ = yaml_float(
    yaml, "szu_traditional_r_max_shift_px", traditional_r_max_shift_px_);
  const std::string enemy_color = yaml["enemy_color"]
    ? yaml["enemy_color"].as<std::string>() : "red";
  if (enemy_color != "red" && enemy_color != "blue") {
    throw std::invalid_argument("enemy_color 必须是 red 或 blue");
  }
  enemy_red_ = enemy_color == "red";
  r_color_roi_scale_ = yaml_float(yaml, "szu_r_color_roi_scale", r_color_roi_scale_);
  r_color_red_threshold_ = yaml_float(
    yaml, "szu_r_red_minus_blue_threshold", r_color_red_threshold_);
  r_color_blue_threshold_ = yaml_float(
    yaml, "szu_r_blue_minus_red_threshold", r_color_blue_threshold_);
  r_color_kernel_size_ = yaml_int(yaml, "szu_r_color_kernel_size", r_color_kernel_size_);
  const auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  const auto distortion_data = yaml["distort_coeffs"].as<std::vector<double>>();
  if (camera_matrix_data.size() != 9 || distortion_data.size() < 4) {
    throw std::invalid_argument("SZU 几何 R 计算需要有效的相机内参与畸变参数");
  }
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (size_t i = 0; i < camera_matrix_data.size(); ++i) {
    camera_matrix_.at<double>(static_cast<int>(i / 3), static_cast<int>(i % 3)) =
      camera_matrix_data[i];
  }
  distort_coeffs_ = cv::Mat(1, static_cast<int>(distortion_data.size()), CV_64F);
  for (size_t i = 0; i < distortion_data.size(); ++i) {
    distort_coeffs_.at<double>(0, static_cast<int>(i)) = distortion_data[i];
  }
  corner_indices_ = yaml_int_vector(yaml, "szu_corner_indices", corner_indices_);
  r_center_index_ = yaml_int(yaml, "szu_r_center_index", r_center_index_);
  required_keypoint_indices_ =
    yaml_int_vector(yaml, "szu_required_keypoint_indices", required_keypoint_indices_);

  model_ = core_.read_model(model_path);
  const auto model_input_shape = model_->input().get_shape();
  if (model_input_shape.size() != 4) {
    throw std::runtime_error("SZU 打符模型输入不是 4 维 NCHW");
  }
  input_height_ = static_cast<int>(model_input_shape[2]);
  input_width_ = static_cast<int>(model_input_shape[3]);

  compiled_model_ = core_.compile_model(
    model_, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  infer_request_ = compiled_model_.create_infer_request();

  const auto output_shape = compiled_model_.output().get_shape();
  if (output_shape.size() != 3) {
    throw std::runtime_error("SZU 打符模型输出不是 3 维 [N,C,A] 或 [N,A,C]");
  }
  if (output_shape[1] <= 256 && output_shape[2] >= 100) {
    output_layout_nca_ = true;
    output_channels_ = static_cast<int>(output_shape[1]);
    num_anchors_ = static_cast<int>(output_shape[2]);
  } else {
    output_layout_nca_ = false;
    num_anchors_ = static_cast<int>(output_shape[1]);
    output_channels_ = static_cast<int>(output_shape[2]);
  }

  if (output_channels_ != num_classes_ + num_keypoints_ * keypoint_dim_) {
    throw std::runtime_error("SZU 打符模型输出通道不是 3 类 + 5 点 * 3");
  }
  if (min_valid_keypoints_ < 0 || min_valid_keypoints_ > num_keypoints_) {
    throw std::runtime_error("szu_min_valid_keypoints 必须在 0..5 范围内");
  }
  if (traditional_min_valid_keypoints_ < 0 || traditional_min_valid_keypoints_ > num_keypoints_) {
    throw std::runtime_error("szu_traditional_min_valid_keypoints 必须在 0..5 范围内");
  }
  if (traditional_corner_window_ < 1 || traditional_corner_window_ > 15 ||
      !std::isfinite(traditional_max_shift_px_) || traditional_max_shift_px_ <= 0.0f ||
      !std::isfinite(traditional_r_max_shift_px_) || traditional_r_max_shift_px_ <= 0.0f ||
      !std::isfinite(r_color_roi_scale_) || r_color_roi_scale_ < 1.0f ||
      !std::isfinite(r_color_red_threshold_) || r_color_red_threshold_ < 0.0f ||
      r_color_red_threshold_ > 255.0f ||
      !std::isfinite(r_color_blue_threshold_) || r_color_blue_threshold_ < 0.0f ||
      r_color_blue_threshold_ > 255.0f || r_color_kernel_size_ < 1 ||
      r_color_kernel_size_ % 2 == 0 || !std::isfinite(rp26_roi_scale_) ||
      rp26_roi_scale_ < 1.0f || !std::isfinite(rp26_armor_area_relative_error_) ||
      rp26_armor_area_relative_error_ <= 0.0f ||
      !std::isfinite(rp26_armor_solidity_threshold_) ||
      rp26_armor_solidity_threshold_ < 0.0f || rp26_armor_solidity_threshold_ > 1.0f ||
      !std::isfinite(rp26_light_solidity_threshold_) ||
      rp26_light_solidity_threshold_ < 0.0f || rp26_light_solidity_threshold_ > 1.0f ||
      !std::isfinite(rp26_red_threshold_) || rp26_red_threshold_ < 0.0f ||
      rp26_red_threshold_ > 255.0f || !std::isfinite(rp26_blue_threshold_) ||
      rp26_blue_threshold_ < 0.0f || rp26_blue_threshold_ > 255.0f ||
      rp26_line_samples_ < 4) {
    throw std::runtime_error("SZU 传统角点精修参数无效");
  }
  if (preprocess_mode_ != "letterbox" && preprocess_mode_ != "center_crop") {
    throw std::runtime_error("szu_preprocess_mode 必须是 letterbox 或 center_crop");
  }
  if (corner_indices_.size() != 4) {
    throw std::runtime_error("szu_corner_indices 必须配置 4 个关键点索引");
  }
  for (int index : corner_indices_) {
    if (index < 0 || index >= num_keypoints_) {
      throw std::runtime_error("szu_corner_indices 超出 SZU 关键点范围");
    }
  }
  for (int index : required_keypoint_indices_) {
    if (index < 0 || index >= num_keypoints_) {
      throw std::runtime_error("szu_required_keypoint_indices 超出 SZU 关键点范围");
    }
  }
  if (r_center_index_ < 0 || r_center_index_ >= num_keypoints_) {
    throw std::runtime_error("szu_r_center_index 超出 SZU 关键点范围");
  }
}

std::vector<SzuRuneDetector::Detection> SzuRuneDetector::detect(
  const cv::Mat & image, bool is_big_rune)
{
  debug_stats_ = {};
  if (image.empty()) return {};

  float scale = 1.0f;
  int pad_w = 0;
  int pad_h = 0;
  int crop_x = 0;
  int crop_y = 0;
  cv::Mat input;
  preprocess(image, input, scale, pad_w, pad_h, crop_x, crop_y);

  ov::Tensor input_tensor(
    ov::element::f32, {1, 3, static_cast<size_t>(input_height_), static_cast<size_t>(input_width_)});
  fill_nchw_rgb_float_tensor(input, input_tensor);
  infer_request_.set_input_tensor(input_tensor);
  infer_request_.infer();

  auto detections = postprocess(scale, pad_w, pad_h, crop_x, crop_y, image.cols, image.rows);
  refine_detections(image, detections);
  refine_rp26_detections(image, detections, is_big_rune);
  if (!is_big_rune && rp26_refine_enabled_ && rp26_strict_) {
    detections.erase(
      std::remove_if(
        detections.begin(), detections.end(), [](const Detection & detection) {
          return !detection.rp26_semantic_valid;
        }),
      detections.end());
  }
  // 传统算法是网络结果的精修和补充，失败时保留有限的网络点，不能把检测直接清空。
  detections.erase(
    std::remove_if(
      detections.begin(), detections.end(), [&](const Detection & detection) {
        if (detection.corners.size() != 4) return true;
        return std::any_of(detection.corners.begin(), detection.corners.end(), [&](const auto & point) {
          return !std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0F ||
                 point.y < 0.0F || point.x >= static_cast<float>(image.cols) ||
                 point.y >= static_cast<float>(image.rows);
        });
      }),
    detections.end());
  auto result = nms(detections);
  debug_stats_.nms_output = static_cast<int>(result.size());
  return result;
}

void SzuRuneDetector::preprocess(
  const cv::Mat & src, cv::Mat & dst, float & scale, int & pad_w, int & pad_h,
  int & crop_x, int & crop_y) const
{
  pad_w = 0;
  pad_h = 0;
  crop_x = 0;
  crop_y = 0;

  if (preprocess_mode_ == "center_crop") {
    // 只在打符模型内部裁成模型训练使用的宽高比，不改变外部相机图像和自瞄链路。
    const double target_aspect = static_cast<double>(input_width_) /
      static_cast<double>(input_height_);
    const double source_aspect = static_cast<double>(src.cols) / static_cast<double>(src.rows);
    int crop_width = src.cols;
    int crop_height = src.rows;
    if (source_aspect > target_aspect) {
      crop_width = std::min(src.cols, std::max(1, cvRound(src.rows * target_aspect)));
      crop_x = (src.cols - crop_width) / 2;
    } else if (source_aspect < target_aspect) {
      crop_height = std::min(src.rows, std::max(1, cvRound(src.cols / target_aspect)));
      crop_y = (src.rows - crop_height) / 2;
    }

    const cv::Rect crop_rect(crop_x, crop_y, crop_width, crop_height);
    const cv::Mat cropped = src(crop_rect);
    scale = static_cast<float>(input_width_) / static_cast<float>(crop_width);
    cv::resize(cropped, dst, cv::Size(input_width_, input_height_));
    return;
  }

  scale = std::min(static_cast<float>(input_width_) / static_cast<float>(src.cols),
                   static_cast<float>(input_height_) / static_cast<float>(src.rows));
  const int new_w = static_cast<int>(std::round(src.cols * scale));
  const int new_h = static_cast<int>(std::round(src.rows * scale));
  pad_w = (input_width_ - new_w) / 2;
  pad_h = (input_height_ - new_h) / 2;
  const int right = input_width_ - new_w - pad_w;
  const int bottom = input_height_ - new_h - pad_h;

  cv::Mat resized;
  if (new_w != src.cols || new_h != src.rows) {
    cv::resize(src, resized, cv::Size(new_w, new_h));
  } else {
    resized = src;
  }
  if (pad_w > 0 || pad_h > 0 || right > 0 || bottom > 0) {
    cv::copyMakeBorder(
      resized, dst, pad_h, bottom, pad_w, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  } else {
    dst = resized;
  }
}

std::vector<SzuRuneDetector::Detection> SzuRuneDetector::postprocess(
  float scale, int pad_w, int pad_h, int crop_x, int crop_y, int orig_w, int orig_h)
{
  std::vector<Detection> detections;
  const float * output_data = infer_request_.get_output_tensor().data<float>();
  debug_stats_.anchors = num_anchors_;

  auto get_val = [&](int c, int a) -> float {
    return output_layout_nca_ ? output_data[c * num_anchors_ + a]
                              : output_data[a * output_channels_ + c];
  };

  for (int anchor = 0; anchor < num_anchors_; ++anchor) {
    int best_class = -1;
    float best_confidence = 0.0f;
    for (int c = 0; c < num_classes_; ++c) {
      const float score = get_val(c, anchor);
      if (score > best_confidence) {
        best_confidence = score;
        best_class = c;
      }
    }
    debug_stats_.max_confidence = std::max(debug_stats_.max_confidence, best_confidence);
    if (best_class < 0 || best_confidence < confidence_threshold_) continue;
    ++debug_stats_.confidence_pass;
    if (best_class >= 0 && best_class < static_cast<int>(debug_stats_.class_counts.size())) {
      ++debug_stats_.class_counts[best_class];
    }

    Detection detection;
    detection.class_id = best_class;
    detection.confidence = best_confidence;
    detection.keypoint_confidences.reserve(num_keypoints_);
    std::vector<cv::Point2f> keypoints;
    keypoints.reserve(num_keypoints_);

    int valid_keypoints = 0;
    int valid_required_keypoints = 0;
    float keypoint_confidence_sum = 0.0f;
    cv::Point2f corner_sum(0.0f, 0.0f);
    bool invalid = false;
    std::vector<bool> valid_keypoint_flags(num_keypoints_, false);
    for (int k = 0; k < num_keypoints_; ++k) {
      const int base = num_classes_ + k * keypoint_dim_;
      // 先还原到裁剪图，再加回裁剪偏移，保证输出坐标仍属于原始相机图像。
      float x = (get_val(base, anchor) - static_cast<float>(pad_w)) / scale +
        static_cast<float>(crop_x);
      float y = (get_val(base + 1, anchor) - static_cast<float>(pad_h)) / scale +
        static_cast<float>(crop_y);
      const float keypoint_confidence = get_val(base + 2, anchor);
      debug_stats_.max_keypoint_confidence =
        std::max(debug_stats_.max_keypoint_confidence, keypoint_confidence);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        invalid = true;
        break;
      }
      if (k == r_center_index_) {
        detection.network_r_center = cv::Point2f(x, y);
        detection.network_r_confidence = keypoint_confidence;
        detection.network_r_valid = true;
      }
      const bool coordinate_in_image =
        x >= 0.0f && y >= 0.0f && x <= static_cast<float>(orig_w - 1) &&
        y <= static_cast<float>(orig_h - 1);
      const bool required_keypoint = std::find(
          required_keypoint_indices_.begin(), required_keypoint_indices_.end(), k) !=
        required_keypoint_indices_.end();
      // 高置信度角点越界通常代表错误候选；低置信度越界点仍交给传统阶段尝试恢复。
      if (!coordinate_in_image && required_keypoint &&
          keypoint_confidence >= keypoint_confidence_threshold_) {
        invalid = true;
        break;
      }
      // 低置信度角点越界时保留原始坐标，后续可明确识别为不可恢复的点；不能把它伪装成图像边界。
      if (!coordinate_in_image && required_keypoint) {
        detection.keypoint_confidences.push_back(keypoint_confidence);
        keypoints.emplace_back(x, y);
        continue;
      }
      x = std::clamp(x, 0.0f, static_cast<float>(orig_w - 1));
      y = std::clamp(y, 0.0f, static_cast<float>(orig_h - 1));

      const cv::Point2f point(x, y);
      detection.keypoint_confidences.push_back(keypoint_confidence);
      if (keypoint_confidence >= keypoint_confidence_threshold_) {
        ++valid_keypoints;
        keypoint_confidence_sum += keypoint_confidence;
        valid_keypoint_flags[k] = true;
        if (required_keypoint) ++valid_required_keypoints;
      }
      keypoints.push_back(point);
    }
    const int candidate_min_valid_keypoints = traditional_refine_enabled_
      ? traditional_min_valid_keypoints_ : min_valid_keypoints_;
    const int candidate_min_required_keypoints = traditional_refine_enabled_
      ? std::min(candidate_min_valid_keypoints, static_cast<int>(required_keypoint_indices_.size()))
      : static_cast<int>(required_keypoint_indices_.size());
    if (invalid || valid_keypoints < std::min(candidate_min_valid_keypoints, num_keypoints_) ||
        valid_required_keypoints < candidate_min_required_keypoints) {
      continue;
    }
    ++debug_stats_.keypoint_pass;

    bool required_keypoints_valid = true;
    for (const int index : required_keypoint_indices_) {
      if (!valid_keypoint_flags[index]) {
        required_keypoints_valid = false;
        break;
      }
    }
    if (required_keypoints_valid) {
      ++debug_stats_.required_keypoint_pass;
    } else if (!traditional_refine_enabled_) {
      continue;
    }

    detection.corners.reserve(corner_indices_.size());
    detection.corner_indices.reserve(corner_indices_.size());
    for (int index : corner_indices_) {
      detection.corners.push_back(keypoints[index]);
      detection.corner_indices.push_back(index);
      corner_sum += keypoints[index];
    }
    detection.r_center = keypoints[r_center_index_];

    detection.center = corner_sum * 0.25f;
    const float mean_keypoint_confidence =
      valid_keypoints > 0 ? keypoint_confidence_sum / static_cast<float>(valid_keypoints) : 0.0f;
    detection.quality = detection.confidence * mean_keypoint_confidence;
    detections.emplace_back(std::move(detection));
  }

  return detections;
}

std::vector<SzuRuneDetector::Detection> SzuRuneDetector::nms(
  std::vector<Detection> & detections) const
{
  std::sort(detections.begin(), detections.end(), [](const Detection & a, const Detection & b) {
    return a.quality > b.quality;
  });

  std::vector<Detection> result;
  std::vector<bool> suppressed(detections.size(), false);
  const float threshold_sq = nms_distance_threshold_ * nms_distance_threshold_;
  for (size_t i = 0; i < detections.size(); ++i) {
    if (suppressed[i]) continue;
    result.push_back(detections[i]);
    for (size_t j = i + 1; j < detections.size(); ++j) {
      if (suppressed[j]) continue;
      const float dx = detections[i].center.x - detections[j].center.x;
      const float dy = detections[i].center.y - detections[j].center.y;
      if (dx * dx + dy * dy < threshold_sq) suppressed[j] = true;
    }
  }
  return result;
}

void SzuRuneDetector::refine_detections(
  const cv::Mat & image, std::vector<Detection> & detections)
{
  if (!traditional_refine_enabled_ || detections.empty() || image.empty()) return;

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else if (image.channels() == 3) {
    // 彩色符光在普通灰度中可能被压暗，传统分支使用三通道最大值保留高亮边缘。
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    cv::Mat max_channel;
    cv::max(channels[0], channels[1], max_channel);
    cv::max(max_channel, channels[2], gray);
  } else {
    return;
  }

  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.8, 0.8, cv::BORDER_REPLICATE);
  cv::Mat gradient_x;
  cv::Mat gradient_y;
  cv::Sobel(blurred, gradient_x, CV_32F, 1, 0, 3, 1.0, 0.0, cv::BORDER_REPLICATE);
  cv::Sobel(blurred, gradient_y, CV_32F, 0, 1, 3, 1.0, 0.0, cv::BORDER_REPLICATE);

  for (auto & detection : detections) {
    if (detection.corners.size() != 4 || detection.corner_indices.size() != 4) continue;
    ++debug_stats_.traditional_attempted;

    int refined_count = 0;
    int evidence_corner_count = 0;
    int finite_corner_count = 0;
    std::array<bool, 4> radial_refined_flags{false, false, false, false};
    for (size_t corner_index = 0; corner_index < detection.corners.size(); ++corner_index) {
      auto & corner = detection.corners[corner_index];
      cv::Point2f refined = corner;
      const int original_index = detection.corner_indices[corner_index];
      const bool confidence_valid =
        original_index >= 0 &&
        original_index < static_cast<int>(detection.keypoint_confidences.size()) &&
        detection.keypoint_confidences[original_index] >= keypoint_confidence_threshold_;
      // 高置信网络角点已经是 PnP 的主要观测，不能再被附近背景边缘无条件改写。
      // 传统分支只尝试补回低置信角点，避免局部梯度把目标尺寸和姿态一起拉偏。
      const bool allow_traditional_refine = !confidence_valid;
      const bool radial_refined = allow_traditional_refine && refine_radial_edge(
        gray, gradient_x, gradient_y, detection.r_center, corner, refined);
      bool subpix_refined = false;
      if (allow_traditional_refine) {
        if (radial_refined) {
          corner = refined;
          radial_refined_flags[corner_index] = true;
          ++refined_count;
          ++debug_stats_.traditional_edge_refined;
        } else if (refine_corner_subpix(gray, gradient_x, gradient_y, corner, refined)) {
          corner = refined;
          ++refined_count;
          subpix_refined = true;
        }
      }

      const bool coordinate_valid = std::isfinite(corner.x) && std::isfinite(corner.y) &&
        corner.x >= 0.0F && corner.y >= 0.0F && corner.x < image.cols && corner.y < image.rows;
      if (coordinate_valid) {
        ++finite_corner_count;
        // 低置信度角点只有找到真实边缘才算被传统视觉补回。
        if (confidence_valid || radial_refined_flags[corner_index]) ++evidence_corner_count;
      }
      if (allow_traditional_refine && !radial_refined && !subpix_refined) {
        ++debug_stats_.traditional_corner_fallback;
      }
    }

    detection.traditional_valid_corners = evidence_corner_count;
    // 几何中心必须建立在四个有证据的角点上，不能只因为坐标有限就把异常外推当成有效。
    detection.traditional_geometry_valid = finite_corner_count == 4 && evidence_corner_count == 4;
    debug_stats_.traditional_corner_refined += refined_count;
    detection.center = cv::Point2f(0.0F, 0.0F);
    for (const auto & corner : detection.corners) detection.center += corner;
    detection.center *= 0.25F;

    const cv::Point2f network_r_center = detection.r_center;
    const bool network_r_valid = std::isfinite(network_r_center.x) &&
      std::isfinite(network_r_center.y) && network_r_center.x >= 0.0F &&
      network_r_center.y >= 0.0F && network_r_center.x < image.cols &&
      network_r_center.y < image.rows;
    // R 的网络点是轮廓搜索的种子，不一定落在发光 R 图标的真实几何中心。
    // 用受限颜色轮廓细化 R，只改善 R 观测和 pnp-r 诊断，不改变四角 PnP。
    detection.network_r_valid = network_r_valid;
    detection.geometric_r_center = cv::Point2f(0.0F, 0.0F);
    detection.geometric_r_valid = false;
    detection.visual_r_center = cv::Point2f(0.0F, 0.0F);
    detection.visual_r_valid = false;
    detection.traditional_r_refined = false;
    detection.r_center = network_r_center;
    if (traditional_r_refine_enabled_ && network_r_valid) {
      cv::Point2f visual_r_center;
      if (refine_visual_r_center(image, network_r_center, detection.corners, visual_r_center)) {
        const double shift = cv::norm(visual_r_center - network_r_center);
        // 颜色分割只在网络点附近找到轮廓时生效，避免背景亮斑替换网络 R。
        if (std::isfinite(shift) && shift <= traditional_r_max_shift_px_) {
          detection.visual_r_center = visual_r_center;
          detection.visual_r_valid = true;
          detection.r_center = visual_r_center;
          detection.traditional_r_refined = true;
          ++debug_stats_.traditional_r_refined;
          ++debug_stats_.traditional_r_geometry;
        }
      }
    }
  }
}

void SzuRuneDetector::refine_rp26_detections(
  const cv::Mat & image, std::vector<Detection> & detections, bool is_big_rune)
{
  if (!rp26_refine_enabled_ || image.empty() || is_big_rune) return;
  for (auto & detection : detections) {
    if (detection.class_id != 0) continue;
    ++debug_stats_.rp26_attempted;
    if (refine_rp26_detection(image, detection)) ++debug_stats_.rp26_valid;
  }
}

bool SzuRuneDetector::contour_contains(
  const std::vector<cv::Point> & contour, const cv::Point2f & point)
{
  return contour.size() >= 5 && cv::pointPolygonTest(contour, point, false) >= 0.0;
}

double SzuRuneDetector::contour_solidity(const std::vector<cv::Point> & contour)
{
  if (contour.size() < 3) return 0.0;
  const double area = std::abs(cv::contourArea(contour));
  std::vector<cv::Point> hull;
  cv::convexHull(contour, hull);
  const double hull_area = std::abs(cv::contourArea(hull));
  return hull_area > 1e-6 ? area / hull_area : 0.0;
}

cv::Point2f SzuRuneDetector::contour_center(const std::vector<cv::Point> & contour)
{
  const cv::Moments moments = cv::moments(contour);
  if (std::abs(moments.m00) > 1e-6) {
    return cv::Point2f(
      static_cast<float>(moments.m10 / moments.m00),
      static_cast<float>(moments.m01 / moments.m00));
  }
  cv::Point2f center;
  for (const auto & point : contour) {
    center += cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y));
  }
  if (!contour.empty()) center *= 1.0F / static_cast<float>(contour.size());
  return center;
}

bool SzuRuneDetector::line_passes_contour(
  const cv::Point2f & a, const cv::Point2f & b,
  const std::vector<cv::Point> & contour, int samples)
{
  if (contour.size() < 5 || samples < 1) return false;
  for (int i = 0; i <= samples; ++i) {
    const float ratio = static_cast<float>(i) / static_cast<float>(samples);
    const cv::Point2f point = a + ratio * (b - a);
    if (cv::pointPolygonTest(contour, point, false) > 0.0) return true;
  }
  return false;
}

bool SzuRuneDetector::build_rp26_anchor_points(
  const std::vector<cv::Point> & armor_contour,
  const std::vector<cv::Point> & light_arm_contour,
  const cv::Point2f & r_center,
  std::vector<cv::Point2f> & anchor_points)
{
  if (armor_contour.size() < 5 || light_arm_contour.size() < 5) return false;

  // 深大 RP26 用靶心和灯臂的联合轮廓做 PCA，再取四个极值点作为锚点。
  // 这一步不依赖网络角点的像素抖动，正是它与直接四点 IPPE 的区别。
  const int total_points = static_cast<int>(armor_contour.size() + light_arm_contour.size());
  cv::Mat data(total_points, 2, CV_64F);
  int row = 0;
  for (const auto & point : armor_contour) {
    data.at<double>(row, 0) = point.x;
    data.at<double>(row++, 1) = point.y;
  }
  for (const auto & point : light_arm_contour) {
    data.at<double>(row, 0) = point.x;
    data.at<double>(row++, 1) = point.y;
  }

  const cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Point2f center(
    static_cast<float>(pca.mean.at<double>(0, 0)),
    static_cast<float>(pca.mean.at<double>(0, 1)));
  cv::Point2f axis_x(
    static_cast<float>(pca.eigenvectors.at<double>(0, 0)),
    static_cast<float>(pca.eigenvectors.at<double>(0, 1)));
  const float axis_norm = cv::norm(axis_x);
  if (!std::isfinite(axis_norm) || axis_norm < 1e-5F) return false;
  axis_x *= 1.0F / axis_norm;
  const cv::Point2f axis_y(-axis_x.y, axis_x.x);

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  std::vector<cv::Point2f> all_points;
  all_points.reserve(static_cast<size_t>(total_points));
  for (const auto & point : armor_contour) all_points.emplace_back(point);
  for (const auto & point : light_arm_contour) all_points.emplace_back(point);
  for (const auto & point : all_points) {
    const cv::Point2f local = point - center;
    const float x = local.dot(axis_x);
    const float y = local.dot(axis_y);
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }
  if (max_x - min_x < 2.0F || max_y - min_y < 2.0F) return false;

  std::array<cv::Point2f, 4> box{
    center + min_x * axis_x + min_y * axis_y,
    center + min_x * axis_x + max_y * axis_y,
    center + max_x * axis_x + max_y * axis_y,
    center + max_x * axis_x + min_y * axis_y};

  // 离 R 更远的两个点是外侧，较近的两个点是内侧。
  std::array<int, 4> order{0, 1, 2, 3};
  std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
    return cv::norm(box[lhs] - r_center) > cv::norm(box[rhs] - r_center);
  });
  const std::array<int, 2> outer{order[0], order[1]};
  const std::array<int, 2> inner{order[2], order[3]};
  const cv::Point2f center_to_r = r_center - center;
  if (cv::norm(center_to_r) < 1e-3F) return false;

  const auto sort_side = [&](const std::array<int, 2> & pair) {
    // 与深大 RP26 的 sort_anchor_points 一致：叉乘正负决定左右，
    // 不依赖 PCA 特征向量本身可能发生的 180 度翻转。
    const float orientation = (box[pair[1]] - box[pair[0]]).cross(center_to_r);
    return orientation > 0.0F
      ? std::array<cv::Point2f, 2>{box[pair[0]], box[pair[1]]}
      : std::array<cv::Point2f, 2>{box[pair[1]], box[pair[0]]};
  };
  const auto outer_side = sort_side(outer);
  const auto inner_side = sort_side(inner);
  anchor_points = {outer_side[0], inner_side[0], inner_side[1], outer_side[1]};
  for (const auto & point : anchor_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
  }
  return true;
}

bool SzuRuneDetector::refine_rp26_detection(
  const cv::Mat & image, Detection & detection) const
{
  detection.rp26_anchor_points.clear();
  detection.rp26_semantic_valid = false;
  detection.rp26_contour_count = 0;
  if (image.type() != CV_8UC3 || detection.corners.size() != 4 ||
      !detection.network_r_valid) {
    // 当前只对小符未激活叶片生成 RP26 锚点；已激活轮廓继续使用原检测语义。
    return false;
  }

  std::vector<cv::Point2f> roi_points = detection.corners;
  roi_points.emplace_back(detection.network_r_center);
  const cv::RotatedRect network_rect = cv::minAreaRect(roi_points);
  if (network_rect.size.width < 2.0F || network_rect.size.height < 2.0F) return false;
  cv::RotatedRect expanded = network_rect;
  expanded.size *= rp26_roi_scale_;
  const cv::Rect roi = expanded.boundingRect() & cv::Rect(0, 0, image.cols, image.rows);
  if (roi.empty()) return false;

  std::vector<cv::Mat> channels;
  cv::split(image(roi), channels);
  cv::Mat color_difference;
  if (enemy_red_) cv::subtract(channels[2], channels[0], color_difference);
  else cv::subtract(channels[0], channels[2], color_difference);
  cv::GaussianBlur(color_difference, color_difference, cv::Size(5, 5), 0.0);
  const double threshold = enemy_red_ ? rp26_red_threshold_ : rp26_blue_threshold_;
  cv::threshold(color_difference, color_difference, threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    color_difference, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, roi.tl());
  detection.rp26_contour_count = static_cast<int>(contours.size());
  if (contours.empty()) return false;

  const cv::Point2f armor_center = detection.center;
  const cv::Point2f r_center = detection.network_r_center;
  const std::vector<cv::Point2f> corner_points = detection.corners;
  double nn_ellipse_area = 0.25 * cv::norm(corner_points[0] - corner_points[3]) *
    cv::norm(corner_points[1] - corner_points[2]) * CV_PI;
  nn_ellipse_area = std::max(1.0, nn_ellipse_area);

  std::vector<cv::Point> armor;
  std::vector<cv::Point> light_arm;
  std::vector<cv::Point> center_r;
  double best_armor_score = std::numeric_limits<double>::infinity();
  double best_light_area = 0.0;
  double best_r_area = std::numeric_limits<double>::infinity();
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;
    const double area = std::abs(cv::contourArea(contour));
    if (!std::isfinite(area) || area < 2.0) continue;
    const double solidity = contour_solidity(contour);
    const bool contains_armor_center = contour_contains(contour, armor_center);
    const bool contains_r = contour_contains(contour, r_center);

    if (contains_armor_center) {
      const double area_error = std::abs(area - nn_ellipse_area) / nn_ellipse_area;
      if (area_error <= rp26_armor_area_relative_error_ &&
          solidity >= rp26_armor_solidity_threshold_ && area_error < best_armor_score) {
        armor = contour;
        best_armor_score = area_error;
      }
    }

    bool contains_corner = false;
    for (const auto & corner : corner_points) {
      if (contour_contains(contour, corner)) {
        contains_corner = true;
        break;
      }
    }
    const bool is_light_arm = !contains_armor_center && !contains_r && !contains_corner &&
      line_passes_contour(armor_center, r_center, contour, rp26_line_samples_);
    const bool side_line_crosses = line_passes_contour(
      corner_points[1], corner_points[2], contour, rp26_line_samples_);
    if (is_light_arm && !side_line_crosses && solidity >= rp26_light_solidity_threshold_ &&
        area > best_light_area) {
      light_arm = contour;
      best_light_area = area;
    }

    if (contains_r && !contains_armor_center && !contains_corner && area < best_r_area) {
      center_r = contour;
      best_r_area = area;
    }
  }
  if (armor.empty() || light_arm.empty() || center_r.empty()) return false;

  const cv::Point2f visual_r = contour_center(center_r);
  if (!std::isfinite(visual_r.x) || !std::isfinite(visual_r.y) ||
      cv::norm(visual_r - detection.network_r_center) > traditional_r_max_shift_px_) {
    return false;
  }
  std::vector<cv::Point2f> anchors;
  if (!build_rp26_anchor_points(armor, light_arm, visual_r, anchors)) return false;

  // 轮廓中心用于目标角度，R 轮廓中心用于符心；只有语义轮廓完整时才替换网络 R。
  const cv::Point2f refined_armor_center = contour_center(armor);
  if (std::isfinite(refined_armor_center.x) && std::isfinite(refined_armor_center.y)) {
    detection.center = refined_armor_center;
  }
  detection.r_center = visual_r;
  detection.visual_r_center = visual_r;
  detection.visual_r_valid = true;
  detection.traditional_r_refined = true;
  detection.rp26_anchor_points = std::move(anchors);
  detection.rp26_semantic_valid = true;
  return true;
}

bool SzuRuneDetector::estimate_geometric_r_center(
  const std::vector<cv::Point2f> & corners, cv::Point2f & r_center) const
{
  if (corners.size() != 4) return false;

  // detection.corners 按原始编号 [0, 1, 3, 4] 保存：外端、逆时针侧、顺时针侧、内端。
  // 透视变换按模型坐标顺时针顺序 [0, 3, 4, 1] 取点。
  // 与 Solver 使用同一套 SZU 26 叶片几何；这里的单位是毫米，仅用于停用中的 GEO-R 诊断。
  const std::vector<cv::Point2f> object_points{
    {0.0F, 850.0F}, {150.0F, 700.0F}, {0.0F, 550.0F}, {-150.0F, 700.0F}};
  const std::vector<cv::Point2f> image_points{
    corners[0], corners[2], corners[3], corners[1]};
  // R 在四角包围区域外，先去畸变再做平面外推，避免把镜头畸变误当成透视变换。
  const std::vector<cv::Point2f> r_object_point{{0.0F, 0.0F}};
  try {
    std::vector<cv::Point2f> undistorted_points;
    cv::undistortPoints(image_points, undistorted_points, camera_matrix_, distort_coeffs_);
    const cv::Mat homography = cv::getPerspectiveTransform(object_points, undistorted_points);
    std::vector<cv::Point2f> projected_undistorted;
    cv::perspectiveTransform(r_object_point, projected_undistorted, homography);
    if (projected_undistorted.size() != 1 ||
        !std::isfinite(projected_undistorted.front().x) ||
        !std::isfinite(projected_undistorted.front().y)) {
      return false;
    }

    const cv::Point2f & normalized = projected_undistorted.front();
    const std::vector<cv::Point3f> normalized_ray{{normalized.x, normalized.y, 1.0F}};
    std::vector<cv::Point2f> projected_distorted;
    cv::projectPoints(
      normalized_ray, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0), camera_matrix_,
      distort_coeffs_, projected_distorted);
    if (projected_distorted.size() != 1 || !std::isfinite(projected_distorted.front().x) ||
        !std::isfinite(projected_distorted.front().y)) {
      return false;
    }
    r_center = projected_distorted.front();
  } catch (const cv::Exception &) {
    return false;
  }
  return true;
}

bool SzuRuneDetector::refine_visual_r_center(
  const cv::Mat & bgr_image, const cv::Point2f & seed, const std::vector<cv::Point2f> & corners,
  cv::Point2f & refined) const
{
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3 || corners.size() != 4 ||
      !std::isfinite(seed.x) || !std::isfinite(seed.y)) {
    return false;
  }

  std::vector<cv::Point2f> roi_points = corners;
  roi_points.emplace_back(seed);
  for (const auto & corner : corners) {
    if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) return false;
  }
  const cv::RotatedRect rotated_roi = cv::minAreaRect(roi_points);
  if (!std::isfinite(rotated_roi.size.width) || !std::isfinite(rotated_roi.size.height) ||
      rotated_roi.size.width < 1.0F || rotated_roi.size.height < 1.0F) {
    return false;
  }
  cv::RotatedRect expanded_roi = rotated_roi;
  expanded_roi.size *= r_color_roi_scale_;
  const cv::Rect roi_rect = expanded_roi.boundingRect() &
    cv::Rect(0, 0, bgr_image.cols, bgr_image.rows);
  if (roi_rect.empty()) return false;

  cv::Point2f blade_center(0.0F, 0.0F);
  for (const auto & corner : corners) blade_center += corner;
  blade_center *= 0.25F;
  // 参考 fuchen 的实现，R 轮廓必须靠近网络种子，并且不能吞掉靶心或四个角点。
  // 颜色分割优先；部分相机曝光下颜色差分会断裂，再用同一 ROI 内的亮度轮廓兜底。
  const auto find_center = [&](const cv::Mat & binary, cv::Point2f & center) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, roi_rect.tl());
    double best_score = std::numeric_limits<double>::infinity();
    cv::Point2f best_center;
    for (const auto & contour : contours) {
      if (contour.size() < 5) continue;
      const double area = std::abs(cv::contourArea(contour));
      if (!std::isfinite(area) || area <= 0.0) continue;

      // 网络 R 可能落在轮廓边缘；允许有限的边界距离，但不接受远处同色背景。
      const double seed_distance = cv::pointPolygonTest(contour, seed, true);
      if (!std::isfinite(seed_distance) || seed_distance < -traditional_r_max_shift_px_) {
        continue;
      }
      // 排除靶心和四个叶片角点所在的轮廓，防止主体亮斑替换 R。
      if (cv::pointPolygonTest(contour, blade_center, false) >= 0.0) continue;
      bool contains_corner = false;
      for (const auto & corner : corners) {
        if (cv::pointPolygonTest(contour, corner, false) >= 0.0) {
          contains_corner = true;
          break;
        }
      }
      if (contains_corner) continue;

      cv::RotatedRect ellipse;
      try {
        ellipse = cv::fitEllipse(contour);
      } catch (const cv::Exception &) {
        // 退化轮廓可能只有近似共线的像素，不能让它中断整帧检测。
        continue;
      }
      if (!std::isfinite(ellipse.center.x) || !std::isfinite(ellipse.center.y) ||
          ellipse.size.width <= 0.0F || ellipse.size.height <= 0.0F) {
        continue;
      }
      const double shift = cv::norm(ellipse.center - seed);
      if (!std::isfinite(shift) || shift > traditional_r_max_shift_px_) continue;
      // 轮廓越大越可能是相连的叶片区域；只作为很小的次级惩罚，不覆盖距离种子远近。
      const double outside_penalty = seed_distance < 0.0 ? -seed_distance : 0.0;
      const double score = shift + outside_penalty + 0.01 * std::sqrt(area);
      if (score < best_score) {
        best_score = score;
        best_center = ellipse.center;
      }
    }
    if (!std::isfinite(best_score)) return false;
    center = best_center;
    return std::isfinite(center.x) && std::isfinite(center.y);
  };

  std::vector<cv::Mat> channels;
  cv::split(bgr_image(roi_rect), channels);
  cv::Mat color_difference;
  if (enemy_red_) {
    cv::subtract(channels[2], channels[0], color_difference);
    cv::GaussianBlur(
      color_difference, color_difference, cv::Size(r_color_kernel_size_, r_color_kernel_size_), 0.0);
    cv::threshold(color_difference, color_difference, r_color_red_threshold_, 255, cv::THRESH_BINARY);
  } else {
    cv::subtract(channels[0], channels[2], color_difference);
    cv::GaussianBlur(
      color_difference, color_difference, cv::Size(r_color_kernel_size_, r_color_kernel_size_), 0.0);
    cv::threshold(color_difference, color_difference, r_color_blue_threshold_, 255, cv::THRESH_BINARY);
  }
  if (find_center(color_difference, refined)) return true;

  // 颜色差分断裂时只在网络 R 附近使用亮度轮廓，避免恢复成全图传统检测。
  cv::Mat gray;
  cv::cvtColor(bgr_image(roi_rect), gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(r_color_kernel_size_, r_color_kernel_size_), 0.0);
  cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  return find_center(gray, refined);
}

bool SzuRuneDetector::refine_radial_edge(
  const cv::Mat & gray, const cv::Mat & gradient_x, const cv::Mat & gradient_y,
  const cv::Point2f & r_center, const cv::Point2f & seed, cv::Point2f & refined) const
{
  if (!std::isfinite(seed.x) || !std::isfinite(seed.y) || !std::isfinite(r_center.x) ||
      !std::isfinite(r_center.y)) {
    return false;
  }

  cv::Point2f radial = seed - r_center;
  const float radial_norm = cv::norm(radial);
  if (!std::isfinite(radial_norm) || radial_norm < 5.0F) return false;
  radial *= 1.0F / radial_norm;

  const float step = 0.5F;
  const int sample_radius = static_cast<int>(std::ceil(traditional_max_shift_px_ / step));
  const int sample_count = sample_radius * 2 + 1;
  std::vector<double> responses(sample_count, 0.0);
  std::vector<double> intensities(sample_count, 0.0);

  const auto sample_float = [](const cv::Mat & image, const cv::Point2f & point) -> double {
    if (point.x < 0.0F || point.y < 0.0F || point.x >= image.cols - 1.0F ||
        point.y >= image.rows - 1.0F) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const int x0 = static_cast<int>(std::floor(point.x));
    const int y0 = static_cast<int>(std::floor(point.y));
    const float dx = point.x - static_cast<float>(x0);
    const float dy = point.y - static_cast<float>(y0);
    const double p00 = image.at<float>(y0, x0);
    const double p10 = image.at<float>(y0, x0 + 1);
    const double p01 = image.at<float>(y0 + 1, x0);
    const double p11 = image.at<float>(y0 + 1, x0 + 1);
    return (1.0 - dx) * (1.0 - dy) * p00 + dx * (1.0 - dy) * p10 +
           (1.0 - dx) * dy * p01 + dx * dy * p11;
  };
  const auto sample_gray = [](const cv::Mat & image, const cv::Point2f & point) -> double {
    if (point.x < 0.0F || point.y < 0.0F || point.x >= image.cols - 1.0F ||
        point.y >= image.rows - 1.0F) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const int x0 = static_cast<int>(std::floor(point.x));
    const int y0 = static_cast<int>(std::floor(point.y));
    const float dx = point.x - static_cast<float>(x0);
    const float dy = point.y - static_cast<float>(y0);
    const double p00 = image.at<unsigned char>(y0, x0);
    const double p10 = image.at<unsigned char>(y0, x0 + 1);
    const double p01 = image.at<unsigned char>(y0 + 1, x0);
    const double p11 = image.at<unsigned char>(y0 + 1, x0 + 1);
    return (1.0 - dx) * (1.0 - dy) * p00 + dx * (1.0 - dy) * p10 +
           (1.0 - dx) * dy * p01 + dx * dy * p11;
  };

  int best_index = -1;
  double best_response = 0.0;
  for (int i = 0; i < sample_count; ++i) {
    const float offset = static_cast<float>(i - sample_radius) * step;
    const cv::Point2f point = seed + radial * offset;
    const double gx = sample_float(gradient_x, point);
    const double gy = sample_float(gradient_y, point);
    const double intensity = sample_gray(gray, point);
    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(intensity)) continue;
    responses[i] = std::abs(gx * radial.x + gy * radial.y);
    intensities[i] = intensity;
    if (responses[i] > best_response) {
      best_response = responses[i];
      best_index = i;
    }
  }

  if (best_index <= 0 || best_index >= sample_count - 1 || best_response < 12.0) return false;
  const double contrast = std::abs(intensities[best_index + 1] - intensities[best_index - 1]);
  if (!std::isfinite(contrast) || contrast < 8.0) return false;

  double subpixel_offset = 0.0;
  const double left = responses[best_index - 1];
  const double middle = responses[best_index];
  const double right = responses[best_index + 1];
  const double denominator = left - 2.0 * middle + right;
  if (std::isfinite(denominator) && std::abs(denominator) > 1e-6) {
    subpixel_offset = 0.5 * (left - right) / denominator;
    subpixel_offset = std::clamp(subpixel_offset, -0.5, 0.5);
  }

  const float offset = static_cast<float>(best_index - sample_radius) * step +
    static_cast<float>(subpixel_offset * step);
  const cv::Point2f candidate = seed + radial * offset;
  if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y) ||
      cv::norm(candidate - seed) > traditional_max_shift_px_) {
    return false;
  }
  refined = candidate;
  return true;
}

bool SzuRuneDetector::refine_corner_subpix(
  const cv::Mat & gray, const cv::Mat & gradient_x, const cv::Mat & gradient_y,
  const cv::Point2f & seed, cv::Point2f & refined) const
{
  const int margin = traditional_corner_window_ + 2;
  if (seed.x < margin || seed.y < margin || seed.x >= gray.cols - margin ||
      seed.y >= gray.rows - margin) {
    return false;
  }

  const double seed_strength = corner_strength(gradient_x, gradient_y, seed);
  std::vector<cv::Point2f> points{seed};
  try {
    cv::cornerSubPix(
      gray, points, cv::Size(traditional_corner_window_, traditional_corner_window_),
      cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 20, 0.01));
  } catch (const cv::Exception &) {
    return false;
  }
  if (points.size() != 1 || !std::isfinite(points[0].x) || !std::isfinite(points[0].y)) {
    return false;
  }

  const double shift = cv::norm(points[0] - seed);
  if (!std::isfinite(shift) || shift > traditional_max_shift_px_) return false;
  if (points[0].x < 1.0F || points[0].y < 1.0F || points[0].x >= gray.cols - 1.0F ||
      points[0].y >= gray.rows - 1.0F) {
    return false;
  }

  const double refined_strength = corner_strength(gradient_x, gradient_y, points[0]);
  // 只有局部结构没有明显变差时才接受精修，避免被附近更强的背景边缘吸走。
  if (refined_strength < std::max(10.0, seed_strength * 0.8)) return false;

  refined = points[0];
  return true;
}

double SzuRuneDetector::corner_strength(
  const cv::Mat & gradient_x, const cv::Mat & gradient_y, const cv::Point2f & point) const
{
  const int radius = std::max(1, traditional_corner_window_ / 2);
  const int center_x = cvRound(point.x);
  const int center_y = cvRound(point.y);
  const int x0 = std::max(radius, center_x - radius);
  const int y0 = std::max(radius, center_y - radius);
  const int x1 = std::min(gradient_x.cols - radius - 1, center_x + radius);
  const int y1 = std::min(gradient_x.rows - radius - 1, center_y + radius);
  if (x0 > x1 || y0 > y1) return 0.0;

  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const double gx = gradient_x.at<float>(y, x);
      const double gy = gradient_y.at<float>(y, x);
      xx += gx * gx;
      xy += gx * gy;
      yy += gy * gy;
    }
  }

  const double trace = xx + yy;
  const double discriminant = std::max(0.0, (xx - yy) * (xx - yy) + 4.0 * xy * xy);
  return 0.5 * (trace - std::sqrt(discriminant));
}

}  // namespace auto_buff
