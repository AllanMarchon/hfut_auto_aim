#include "climber_rune_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace auto_buff_v2
{
ClimberRuneDetector::ClimberRuneDetector(const YAML::Node & config)
{
  const auto path = config["model"].as<std::string>();
  confidence_threshold_ = config["ConfidenceThreshold"].as<float>(confidence_threshold_);
  iou_threshold_ = config["IouThreshold"].as<float>(iou_threshold_);
  binary_threshold_ = config["binary_threshold"].as<int>(binary_threshold_);
  dilate_kernel_size_ = config["dilate_kernel_size"].as<int>(dilate_kernel_size_);
  dilate_iterations_ = config["dilate_iterations"].as<int>(dilate_iterations_);
  r_center_mask_ratio_ = config["r_center_mask_ratio"].as<double>(r_center_mask_ratio_);
  r_center_contour_div_ = config["r_center_contour_div"].as<double>(r_center_contour_div_);
  r_center_refine_max_shift_ =
    config["r_center_refine_max_shift"].as<double>(r_center_refine_max_shift_);
  dedup_distance_ = config["dedup_distance"].as<double>(dedup_distance_);
  max_bullseyes_ = config["max_bullseyes"].as<int>(max_bullseyes_);
  if (confidence_threshold_ < 0 || confidence_threshold_ > 1 || iou_threshold_ < 0 ||
      iou_threshold_ > 1 || binary_threshold_ < 0 || binary_threshold_ > 255 ||
      dilate_kernel_size_ <= 0 || dilate_iterations_ < 0 || r_center_mask_ratio_ <= 0 ||
      r_center_contour_div_ <= 0 || r_center_refine_max_shift_ < 0 || dedup_distance_ < 0 ||
      max_bullseyes_ < 1 || max_bullseyes_ > 2)
    throw std::invalid_argument("invalid buff_v2.detector.climber detector parameters");

  const auto device = config["device"].as<std::string>("CPU");
  model_ = core_.compile_model(core_.read_model(path), device);
  request_ = model_.create_infer_request();
  input_ = request_.get_input_tensor();
  const auto output = model_.output();
  const auto shape = output.get_shape();
  const auto input_shape = input_.get_shape();
  if (input_shape.size() != 4 || input_shape[0] != 1 || input_shape[1] != 3 ||
      input_shape[2] == 0 || input_shape[3] == 0 ||
      input_.get_element_type() != ov::element::f32 || shape.size() != 3 ||
      shape[0] != 1 || shape[1] != 33 || output.get_element_type() != ov::element::f32)
    throw std::invalid_argument("climber requires FP32 [1,3,H,W] -> [1,33,N] YOLO11-pose");
  input_size_ = {static_cast<int>(input_shape[3]), static_cast<int>(input_shape[2])};
}

std::vector<ClimberRuneDetector::Candidate> ClimberRuneDetector::infer(const cv::Mat & image)
{
  // 与 Climber 一致：左上对齐、黑色补边、RGB、归一化、CHW。
  const int height = input_size_.height, width = input_size_.width;
  const float scale = std::min(static_cast<float>(height) / image.rows,
                             static_cast<float>(width) / image.cols);
  const cv::Matx23f transform{scale, 0, 0, 0, scale, 0};
  cv::Mat blob;
  if (scale < 1) {
    cv::warpAffine(image, blob, transform, input_size_);
    blob.convertTo(blob, CV_32F, 1.0 / 255);
    cv::cvtColor(blob, blob, cv::COLOR_BGR2RGB);
  } else {
    image.convertTo(blob, CV_32F, 1.0 / 255);
    cv::cvtColor(blob, blob, cv::COLOR_BGR2RGB);
    cv::warpAffine(blob, blob, transform, input_size_);
  }
  float * data = input_.data<float>();
  for (int c = 0; c < 3; ++c)
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x)
        data[c * height * width + y * width + x] = blob.at<cv::Vec3f>(y, x)[c];
  request_.infer();
  const auto output = request_.get_output_tensor();
  const int count = static_cast<int>(output.get_shape()[2]);
  const float * values = output.data<const float>();
  auto at = [&](int row, int col) { return values[row * count + col]; };
  std::vector<Candidate> candidates;
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  // 与现有 Njust 检测器一致，直接选择接口传入的颜色。
  const int class_row = enemy_red_ ? 4 : 5;
  for (int i = 0; i < count; ++i) {
    const float score = at(class_row, i);
    if (!std::isfinite(score) || score <= confidence_threshold_) continue;
    std::array<cv::Point2f, 9> raw;
    bool valid = true;
    for (int p = 0; p < 9; ++p) {
      raw[p] = {at(6 + p * 3, i) / scale, at(7 + p * 3, i) / scale};
      valid = valid && std::isfinite(raw[p].x) && std::isfinite(raw[p].y);
    }
    if (!valid) continue;
    const float cx = at(0, i), cy = at(1, i), w = at(2, i), h = at(3, i);
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) ||
        !std::isfinite(h) || w <= 0 || h <= 0) continue;
    Candidate candidate;
    candidate.box = {static_cast<int>((cx - w * 0.5f) / scale),
                     static_cast<int>((cy - h * 0.5f) / scale),
                     static_cast<int>(w / scale), static_cast<int>(h / scale)};
    candidate.score = score;
    candidate.points[0] = (raw[4] + raw[5]) * 0.5f;
    candidate.points[1] = (raw[6] + raw[7]) * 0.5f;
    candidate.points[2] = (raw[1] + raw[8]) * 0.5f;
    candidate.points[3] = (raw[2] + raw[3]) * 0.5f;
    candidate.points[4] = (candidate.points[0] + candidate.points[1] +
                           candidate.points[2] + candidate.points[3]) * 0.25f;
    candidate.points[5] = raw[0];
    candidates.push_back(candidate);
    boxes.push_back(candidate.box);
    scores.push_back(score);
  }
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, confidence_threshold_, iou_threshold_, indices);
  std::vector<Candidate> kept;
  for (int i : indices) kept.push_back(candidates[i]);
  std::sort(kept.begin(), kept.end(),
            [](const auto & a, const auto & b) { return a.score > b.score; });
  return kept;
}

cv::Point2f ClimberRuneDetector::refine_center(
  const cv::Mat & image, const std::vector<Candidate> & candidates,
  std::vector<std::vector<cv::Point>> & contours)
{
  cv::Point2f initial(0, 0);
  for (const auto & candidate : candidates) initial += candidate.points[5];
  initial *= 1.0f / candidates.size();
  const double radius = cv::norm(candidates[0].points[2] - candidates[0].points[4]) *
                        r_center_mask_ratio_;
  if (radius <= 0) return initial;
  cv::Mat binary;
  cv::cvtColor(image, binary, cv::COLOR_BGR2GRAY);
  cv::threshold(binary, binary, binary_threshold_, 255, cv::THRESH_BINARY);
  cv::dilate(binary, binary,
             cv::getStructuringElement(cv::MORPH_RECT,
                                       {dilate_kernel_size_, dilate_kernel_size_}),
             {-1, -1}, dilate_iterations_);
  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U);
  cv::circle(mask, initial, static_cast<int>(radius), cv::Scalar(255), cv::FILLED);
  cv::bitwise_and(binary, mask, binary);
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  auto center = initial;
  double best = std::numeric_limits<double>::infinity();
  for (const auto & contour : contours) {
    const auto rect = cv::minAreaRect(contour);
    const double shorter = std::min(rect.size.width, rect.size.height);
    if (shorter <= 0) continue;
    const double score = std::max(rect.size.width, rect.size.height) / shorter +
                         cv::norm(rect.center - initial) / (radius / r_center_contour_div_);
    if (score < best) {
      best = score;
      center = rect.center;
    }
  }
  const double shift = cv::norm(center - initial);
  if (shift > r_center_refine_max_shift_ && shift > 1e-6)
    center = initial + (center - initial) * static_cast<float>(r_center_refine_max_shift_ / shift);
  return center;
}

RuneElements ClimberRuneDetector::detect(const cv::Mat & image)
{
  RuneElements elements;
  elements.climber_contours.emplace();
  if (image.empty()) return elements;
  elements.candidates = infer(image);
  const auto & candidates = elements.candidates;
  if (candidates.empty()) return elements;
  const auto center = refine_center(image, candidates, *elements.climber_contours);
  for (const auto & candidate : candidates) {
    const auto duplicate = std::any_of(
      elements.bullseyes.begin(), elements.bullseyes.end(), [&](const auto & bull) {
        return cv::norm(bull.center - candidate.points[4]) < dedup_distance_;
      });
    if (duplicate) continue;
    // 源模型输出可打靶心，buff_v2 中对应未激活符叶。
    elements.bullseyes.push_back({candidate.points[4],
      {candidate.points[0], candidate.points[1], candidate.points[2], candidate.points[3]},
      false, candidate.score});
    if (elements.bullseyes.size() == static_cast<std::size_t>(max_bullseyes_)) break;
  }
  elements.icons.push_back({center, candidates.front().score});
  return elements;
}
}  // namespace auto_buff_v2
