#include "szu_rune_detector.hpp"

#include <algorithm>
#include <cmath>
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
  confidence_threshold_ = yaml_float(yaml, "szu_confidence", confidence_threshold_);
  keypoint_confidence_threshold_ =
    yaml_float(yaml, "szu_keypoint_confidence", keypoint_confidence_threshold_);
  nms_distance_threshold_ = yaml_float(yaml, "szu_nms_distance", nms_distance_threshold_);
  min_valid_keypoints_ = yaml_int(yaml, "szu_min_valid_keypoints", min_valid_keypoints_);
  corner_indices_ = yaml_int_vector(yaml, "szu_corner_indices", corner_indices_);
  r_center_index_ = yaml_int(yaml, "szu_r_center_index", r_center_index_);

  model_ = core_.read_model(model_path);
  const auto model_input_shape = model_->input().get_shape();
  if (model_input_shape.size() != 4) {
    throw std::runtime_error("SZU 打符模型输入不是 4 维 NCHW");
  }
  input_height_ = static_cast<int>(model_input_shape[2]);
  input_width_ = static_cast<int>(model_input_shape[3]);

  compiled_model_ = core_.compile_model(model_, device_);
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
  if (corner_indices_.size() != 4) {
    throw std::runtime_error("szu_corner_indices 必须配置 4 个关键点索引");
  }
  for (int index : corner_indices_) {
    if (index < 0 || index >= num_keypoints_) {
      throw std::runtime_error("szu_corner_indices 超出 SZU 关键点范围");
    }
  }
  if (r_center_index_ < 0 || r_center_index_ >= num_keypoints_) {
    throw std::runtime_error("szu_r_center_index 超出 SZU 关键点范围");
  }
}

std::vector<SzuRuneDetector::Detection> SzuRuneDetector::detect(const cv::Mat & image)
{
  if (image.empty()) return {};

  float scale = 1.0f;
  int pad_w = 0;
  int pad_h = 0;
  cv::Mat input;
  preprocess_letterbox(image, input, scale, pad_w, pad_h);

  ov::Tensor input_tensor(
    ov::element::f32, {1, 3, static_cast<size_t>(input_height_), static_cast<size_t>(input_width_)});
  fill_nchw_rgb_float_tensor(input, input_tensor);
  infer_request_.set_input_tensor(input_tensor);
  infer_request_.infer();

  return postprocess(scale, pad_w, pad_h, image.cols, image.rows);
}

void SzuRuneDetector::preprocess_letterbox(
  const cv::Mat & src, cv::Mat & dst, float & scale, int & pad_w, int & pad_h) const
{
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
  float scale, int pad_w, int pad_h, int orig_w, int orig_h)
{
  std::vector<Detection> detections;
  const float * output_data = infer_request_.get_output_tensor().data<float>();

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
    if (best_class < 0 || best_confidence < confidence_threshold_) continue;

    Detection detection;
    detection.class_id = best_class;
    detection.confidence = best_confidence;
    detection.keypoint_confidences.reserve(num_keypoints_);
    std::vector<cv::Point2f> keypoints;
    keypoints.reserve(num_keypoints_);

    int valid_keypoints = 0;
    float keypoint_confidence_sum = 0.0f;
    cv::Point2f corner_sum(0.0f, 0.0f);
    bool invalid = false;
    for (int k = 0; k < num_keypoints_; ++k) {
      const int base = num_classes_ + k * keypoint_dim_;
      float x = (get_val(base, anchor) - static_cast<float>(pad_w)) / scale;
      float y = (get_val(base + 1, anchor) - static_cast<float>(pad_h)) / scale;
      const float keypoint_confidence = get_val(base + 2, anchor);
      if (x < 0.0f || y < 0.0f) {
        invalid = true;
        break;
      }
      x = std::clamp(x, 0.0f, static_cast<float>(orig_w - 1));
      y = std::clamp(y, 0.0f, static_cast<float>(orig_h - 1));

      const cv::Point2f point(x, y);
      detection.keypoint_confidences.push_back(keypoint_confidence);
      if (keypoint_confidence >= keypoint_confidence_threshold_) {
        ++valid_keypoints;
        keypoint_confidence_sum += keypoint_confidence;
      }
      keypoints.push_back(point);
    }
    if (invalid || valid_keypoints < std::min(min_valid_keypoints_, num_keypoints_)) continue;

    detection.corners.reserve(4);
    for (int index : corner_indices_) {
      detection.corners.push_back(keypoints[index]);
      corner_sum += keypoints[index];
    }
    detection.r_center = keypoints[r_center_index_];

    detection.center = corner_sum * 0.25f;
    const float mean_keypoint_confidence =
      valid_keypoints > 0 ? keypoint_confidence_sum / static_cast<float>(valid_keypoints) : 0.0f;
    detection.quality = detection.confidence * mean_keypoint_confidence;
    detections.emplace_back(std::move(detection));
  }

  return nms(detections);
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

}  // namespace auto_buff
