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
  traditional_corner_window_ = yaml_int(
    yaml, "szu_traditional_corner_window", traditional_corner_window_);
  traditional_max_shift_px_ = yaml_float(
    yaml, "szu_traditional_max_shift_px", traditional_max_shift_px_);
  traditional_r_max_shift_px_ = yaml_float(
    yaml, "szu_traditional_r_max_shift_px", traditional_r_max_shift_px_);
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
      !std::isfinite(traditional_r_max_shift_px_) || traditional_r_max_shift_px_ <= 0.0f) {
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

std::vector<SzuRuneDetector::Detection> SzuRuneDetector::detect(const cv::Mat & image)
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
      const bool radial_refined = refine_radial_edge(
        gray, gradient_x, gradient_y, detection.r_center, corner, refined);
      bool subpix_refined = false;
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

      const int original_index = detection.corner_indices[corner_index];
      const bool confidence_valid =
        original_index >= 0 &&
        original_index < static_cast<int>(detection.keypoint_confidences.size()) &&
        detection.keypoint_confidences[original_index] >= keypoint_confidence_threshold_;
      const bool coordinate_valid = std::isfinite(corner.x) && std::isfinite(corner.y) &&
        corner.x >= 0.0F && corner.y >= 0.0F && corner.x < image.cols && corner.y < image.rows;
      if (coordinate_valid) {
        ++finite_corner_count;
        // 低置信度角点只有找到真实边缘才算被传统视觉补回。
        if (confidence_valid || radial_refined_flags[corner_index]) ++evidence_corner_count;
      }
      if (!radial_refined && !subpix_refined) {
        ++debug_stats_.traditional_corner_fallback;
      }
    }

    detection.traditional_valid_corners = evidence_corner_count;
    // 精修失败时保留网络点；这里的几何有效只表示四个坐标仍然有限，不作为丢帧条件。
    detection.traditional_geometry_valid = finite_corner_count == 4;
    debug_stats_.traditional_corner_refined += refined_count;
    detection.center = cv::Point2f(0.0F, 0.0F);
    for (const auto & corner : detection.corners) detection.center += corner;
    detection.center *= 0.25F;

    cv::Point2f geometric_r_center;
    const bool geometric_r_valid =
      detection.traditional_geometry_valid &&
      estimate_geometric_r_center(detection.corners, geometric_r_center);
    if (geometric_r_valid) {
      ++debug_stats_.traditional_geometry_pass;
    }

    cv::Point2f visual_r_center;
    const bool visual_r_valid = traditional_r_refine_enabled_ &&
      refine_visual_r_center(gray, detection.r_center, detection.corners, visual_r_center);
    if (visual_r_valid) ++debug_stats_.traditional_r_geometry;

    const cv::Point2f network_r_center = detection.r_center;
    const bool network_r_valid = std::isfinite(network_r_center.x) &&
      std::isfinite(network_r_center.y) && network_r_center.x >= 0.0F &&
      network_r_center.y >= 0.0F && network_r_center.x < image.cols &&
      network_r_center.y < image.rows;
    const bool network_r_confidence_valid =
      r_center_index_ >= 0 && r_center_index_ < static_cast<int>(detection.keypoint_confidences.size()) &&
      detection.keypoint_confidences[r_center_index_] >= keypoint_confidence_threshold_;

    // 先用四角的平面几何求 R 中心，再用网络和局部亮斑做小幅融合，避免视觉误检把中心拉走。
    cv::Point2f selected_r_center = network_r_center;
    bool selected_r_refined = false;
    if (geometric_r_valid) {
      const double network_geometry_gap = network_r_valid
        ? cv::norm(geometric_r_center - network_r_center)
        : std::numeric_limits<double>::infinity();
      if (!network_r_valid || !network_r_confidence_valid ||
          network_geometry_gap <= traditional_r_max_shift_px_) {
        selected_r_center = geometric_r_center;
        selected_r_refined = true;
        if (network_r_valid && std::isfinite(network_geometry_gap)) {
          selected_r_center = selected_r_center * 0.70F + network_r_center * 0.30F;
        }
      }
    }
    if (visual_r_valid && std::isfinite(selected_r_center.x) && std::isfinite(selected_r_center.y) &&
        cv::norm(visual_r_center - selected_r_center) <= traditional_r_max_shift_px_) {
      selected_r_center = selected_r_center * 0.75F + visual_r_center * 0.25F;
      selected_r_refined = true;
    }
    if (std::isfinite(selected_r_center.x) && std::isfinite(selected_r_center.y) &&
        selected_r_center.x >= 0.0F && selected_r_center.y >= 0.0F &&
        selected_r_center.x < image.cols && selected_r_center.y < image.rows) {
      detection.r_center = selected_r_center;
      detection.traditional_r_refined = selected_r_refined;
      if (selected_r_refined) ++debug_stats_.traditional_r_refined;
    }
  }
}

bool SzuRuneDetector::estimate_geometric_r_center(
  const std::vector<cv::Point2f> & corners, cv::Point2f & r_center) const
{
  if (corners.size() != 4) return false;

  // detection.corners 的顺序是模型原始编号 [0, 1, 3, 4]：右、上、下、左。
  // 透视变换必须改成环向顺序 [1, 0, 3, 4]：上、右、下、左。
  const std::vector<cv::Point2f> object_points{
    {0.0F, 827.0F}, {127.0F, 700.0F}, {0.0F, 573.0F}, {-127.0F, 700.0F}};
  const std::vector<cv::Point2f> image_points{
    corners[1], corners[0], corners[2], corners[3]};
  const std::vector<cv::Point2f> r_object_point{{0.0F, 0.0F}};
  try {
    const cv::Mat homography = cv::getPerspectiveTransform(object_points, image_points);
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(r_object_point, projected, homography);
    if (projected.size() != 1 || !std::isfinite(projected.front().x) ||
        !std::isfinite(projected.front().y)) {
      return false;
    }
    r_center = projected.front();
  } catch (const cv::Exception &) {
    return false;
  }
  return true;
}

bool SzuRuneDetector::refine_visual_r_center(
  const cv::Mat & gray, const cv::Point2f & seed, const std::vector<cv::Point2f> & corners,
  cv::Point2f & refined) const
{
  if (gray.empty() || gray.type() != CV_8UC1 || corners.size() != 4 ||
      !std::isfinite(seed.x) || !std::isfinite(seed.y)) {
    return false;
  }

  std::vector<float> corner_distances;
  corner_distances.reserve(corners.size());
  for (const auto & corner : corners) {
    if (!std::isfinite(corner.x) || !std::isfinite(corner.y)) return false;
    corner_distances.emplace_back(cv::norm(corner - seed));
  }
  std::nth_element(
    corner_distances.begin(), corner_distances.begin() + corner_distances.size() / 2,
    corner_distances.end());
  const float median_radius = corner_distances[corner_distances.size() / 2];
  if (!std::isfinite(median_radius) || median_radius < 8.0F) return false;
  const int radius = std::clamp(cvRound(median_radius * 0.45F), 8, 96);
  const int x0 = std::max(0, cvFloor(seed.x) - radius);
  const int y0 = std::max(0, cvFloor(seed.y) - radius);
  const int x1 = std::min(gray.cols - 1, cvCeil(seed.x) + radius);
  const int y1 = std::min(gray.rows - 1, cvCeil(seed.y) + radius);
  if (x1 <= x0 || y1 <= y0) return false;

  const cv::Rect roi_rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
  cv::Mat roi = gray(roi_rect);
  cv::Mat binary;
  cv::Scalar mean_value;
  cv::Scalar stddev_value;
  cv::meanStdDev(roi, mean_value, stddev_value);
  const double adaptive_threshold = std::clamp(
    mean_value[0] + 0.45 * stddev_value[0], 25.0, 190.0);
  cv::threshold(roi, binary, adaptive_threshold, 255, cv::THRESH_BINARY);
  cv::morphologyEx(
    binary, binary, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  double best_score = std::numeric_limits<double>::infinity();
  cv::Point2f best_center;
  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    if (!std::isfinite(area) || area < 3.0 || area > static_cast<double>(roi_rect.area()) * 0.65) {
      continue;
    }
    const cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
    const cv::Point2f candidate = rotated_rect.center + cv::Point2f(
      static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y));
    const double shift = cv::norm(candidate - seed);
    if (!std::isfinite(shift) || shift > traditional_r_max_shift_px_) continue;
    const double area_penalty = area < 8.0 ? 8.0 - area : 0.0;
    const double score = shift + area_penalty;
    if (score < best_score) {
      best_score = score;
      best_center = candidate;
    }
  }
  if (!std::isfinite(best_score)) return false;
  refined = best_center;
  return std::isfinite(refined.x) && std::isfinite(refined.y);
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
