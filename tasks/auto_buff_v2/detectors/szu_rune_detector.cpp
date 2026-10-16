#include "szu_rune_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>

namespace auto_buff_v2
{
namespace
{
// 沿网络关键点方向计算椭圆边缘交点，按 RuneModel 所需的上、左、下、右顺序输出。
bool project_bullseye_corners(
  const cv::RotatedRect & ellipse, const std::array<cv::Point2f, 5> & points,
  const cv::Point2f & armor_center, std::array<cv::Point2f, 4> & corners)
{
  const double angle = ellipse.angle * CV_PI / 180;
  const cv::Point2f axis(std::cos(angle), std::sin(angle));
  const cv::Point2f perpendicular(-axis.y, axis.x);
  const double half_width = ellipse.size.width * 0.5;
  const double half_height = ellipse.size.height * 0.5;
  const std::array<int, 4> order{0, 1, 4, 3};  // 上、左、下、右。
  for (int i = 0; i < 4; ++i) {
    const auto direction = points[order[i]] - armor_center;
    const double u = direction.dot(axis) / half_width;
    const double v = direction.dot(perpendicular) / half_height;
    const double length = std::hypot(u, v);
    if (length <= 1e-6) return false;
    corners[i] = ellipse.center + direction * static_cast<float>(1 / length);
  }
  return true;
}
}  // namespace

SzuRuneDetector::SzuRuneDetector(const YAML::Node & config)
{
  confidence_threshold_ = config["confidence_threshold"].as<float>(confidence_threshold_);
  keypoint_confidence_threshold_ =
    config["keypoint_confidence_threshold"].as<float>(keypoint_confidence_threshold_);
  nms_distance_ = config["nms_distance_threshold"].as<float>(nms_distance_);
  min_valid_keypoints_ = config["min_valid_keypoints"].as<int>(min_valid_keypoints_);
  letterbox_value_ = config["letterbox_value"].as<int>(letterbox_value_);
  roi_scale_ = config["roi_scale"].as<double>(roi_scale_);
  gaussian_kernel_size_ = config["gaussian_kernel_size"].as<int>(gaussian_kernel_size_);
  gaussian_sigma_ = config["gaussian_sigma"].as<double>(gaussian_sigma_);
  red_threshold_ = config["red_minus_blue_threshold"].as<double>(red_threshold_);
  blue_threshold_ = config["blue_minus_red_threshold"].as<double>(blue_threshold_);
  armor_area_error_ = config["armor_area_relative_error"].as<double>(armor_area_error_);
  armor_solidity_ = config["armor_solidity"].as<double>(armor_solidity_);
  armor_border_margin_ = config["armor_border_margin"].as<int>(armor_border_margin_);
  // 检查概率参数是否为有限值且位于 [0,1]。
  auto probability = [](double value) {
    return std::isfinite(value) && value >= 0 && value <= 1;
  };
  if (!probability(confidence_threshold_) || !probability(keypoint_confidence_threshold_) ||
      !std::isfinite(nms_distance_) || nms_distance_ <= 0 ||
      min_valid_keypoints_ < 1 || min_valid_keypoints_ > 5 ||
      letterbox_value_ < 0 || letterbox_value_ > 255 ||
      !std::isfinite(roi_scale_) || roi_scale_ < 1 ||
      gaussian_kernel_size_ <= 0 || gaussian_kernel_size_ % 2 == 0 ||
      !std::isfinite(gaussian_sigma_) || gaussian_sigma_ < 0 ||
      !std::isfinite(red_threshold_) || red_threshold_ < 0 || red_threshold_ > 255 ||
      !std::isfinite(blue_threshold_) || blue_threshold_ < 0 || blue_threshold_ > 255 ||
      !std::isfinite(armor_area_error_) || armor_area_error_ < 0 ||
      !probability(armor_solidity_) || armor_border_margin_ < 0)
    throw std::invalid_argument("invalid buff_v2.detector.szu detector parameters");

  auto network = core_.read_model(config["model"].as<std::string>());
  if (network->inputs().size() != 1 || network->outputs().size() != 1)
    throw std::invalid_argument("szu requires one input and one output");
  const auto input_shape = network->input().get_shape();
  const auto output = network->output();
  const auto shape = output.get_shape();
  if (input_shape.size() != 4 || input_shape[0] != 1 || input_shape[1] != 3 ||
      input_shape[2] == 0 || input_shape[3] == 0 ||
      shape.size() != 3 || shape[0] != 1 ||
      (shape[1] != 18 && shape[2] != 18) || shape[1] == 0 || shape[2] == 0 ||
      output.get_element_type() != ov::element::f32)
    throw std::invalid_argument("szu requires [1,3,H,W] -> FP32 [1,18,N] or [1,N,18]");
  input_size_ = {static_cast<int>(input_shape[3]), static_cast<int>(input_shape[2])};
  output_nca_ = shape[1] == 18;

  // 保持源模型预处理：BGR 无符号八位 NHWC 转为 RGB 浮点 NCHW，并归一化至 [0,1]。
  ov::preprocess::PrePostProcessor processor(network);
  processor.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);
  processor.input().preprocess().convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB).scale(255.0f);
  processor.input().model().set_layout("NCHW");
  model_ = core_.compile_model(processor.build(), config["device"].as<std::string>("CPU"),
                             ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  request_ = model_.create_infer_request();
  input_ = request_.get_input_tensor();
}

// 居中缩放补边后推理，还原五点坐标，按关键点质量筛选并进行距离非极大值抑制。
std::vector<SzuRuneDetector::Candidate> SzuRuneDetector::infer(const cv::Mat & image)
{
  const float scale = std::min(static_cast<float>(input_size_.width) / image.cols,
                               static_cast<float>(input_size_.height) / image.rows);
  const cv::Size resized_size{std::max(1, cvRound(image.cols * scale)),
                              std::max(1, cvRound(image.rows * scale))};
  const int pad_x = (input_size_.width - resized_size.width) / 2;
  const int pad_y = (input_size_.height - resized_size.height) / 2;
  cv::Mat resized, letterbox;
  cv::resize(image, resized, resized_size);
  cv::copyMakeBorder(resized, letterbox, pad_y,
    input_size_.height - resized_size.height - pad_y, pad_x,
    input_size_.width - resized_size.width - pad_x, cv::BORDER_CONSTANT,
    cv::Scalar::all(letterbox_value_));
  std::memcpy(input_.data<uint8_t>(), letterbox.data, input_.get_byte_size());
  request_.infer();
  const auto output = request_.get_output_tensor();
  const auto shape = output.get_shape();
  const int count = static_cast<int>(shape[output_nca_ ? 2 : 1]);
  const float * data = output.data<const float>();
  // 根据模型输出布局读取指定候选的通道值。
  auto at = [&](int channel, int anchor) {
    return output_nca_ ? data[channel * count + anchor] : data[anchor * 18 + channel];
  };
  std::vector<Candidate> candidates;
  for (int i = 0; i < count; ++i) {
    int class_id = -1;
    float confidence = 0;
    for (int c = 0; c < 3; ++c) {
      const float score = at(c, i);
      if (std::isfinite(score) && score > confidence) {
        confidence = score;
        class_id = c;
      }
    }
    if (class_id < 0 || confidence < confidence_threshold_) continue;
    Candidate candidate;
    candidate.confidence = confidence;
    candidate.active = class_id != 0;
    int valid = 0;
    float quality = 0;
    bool finite = true;
    for (int p = 0; p < 5; ++p) {
      float x = (at(3 + p * 3, i) - pad_x) / scale;
      float y = (at(4 + p * 3, i) - pad_y) / scale;
      const float score = at(5 + p * 3, i);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(score) || x < 0 || y < 0) {
        finite = false;
        break;
      }
      candidate.points[p] = {std::min(x, static_cast<float>(image.cols - 1)),
                             std::min(y, static_cast<float>(image.rows - 1))};
      if (score >= keypoint_confidence_threshold_) {
        ++valid;
        quality += score;
        candidate.center += candidate.points[p];
      }
    }
    if (!finite || valid < min_valid_keypoints_) continue;
    candidate.center *= 1.0f / valid;
    candidate.quality = confidence * quality / valid;
    candidates.push_back(candidate);
  }
  std::stable_sort(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) { return a.quality > b.quality; });
  std::vector<Candidate> kept;
  for (const auto & candidate : candidates) {
    if (std::none_of(kept.begin(), kept.end(), [&](const auto & previous) {
          return cv::norm(previous.center - candidate.center) < nms_distance_;
        }))
      kept.push_back(candidate);
  }
  return kept;
}

// 在五点限定的区域内用敌方颜色筛选靶心和 R 标轮廓，拟合椭圆并生成接口观测。
bool SzuRuneDetector::refine(const cv::Mat & image, const Candidate & candidate,
                            RuneBullseye & bull, RuneIcon & icon) const
{
  const auto & p = candidate.points;
  const cv::Point2f armor_center = (p[0] + p[1] + p[3] + p[4]) * 0.25f;
  const double expected_area = cv::norm(p[0] - p[4]) * cv::norm(p[1] - p[3]) * CV_PI / 4;
  if (expected_area <= 1e-6) return false;
  auto rect = cv::minAreaRect(std::vector<cv::Point2f>(p.begin(), p.end()));
  rect.size *= static_cast<float>(roi_scale_);
  const auto roi = rect.boundingRect() & cv::Rect(0, 0, image.cols, image.rows);
  if (roi.empty()) return false;
  std::vector<cv::Mat> channels;
  cv::split(image(roi), channels);
  cv::Mat binary = enemy_red_ ? channels[2] - channels[0] : channels[0] - channels[2];
  cv::GaussianBlur(binary, binary, {gaussian_kernel_size_, gaussian_kernel_size_},
                   gaussian_sigma_);
  cv::threshold(binary, binary, enemy_red_ ? red_threshold_ : blue_threshold_,
                255, cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, roi.tl());
  const std::vector<cv::Point> * armor = nullptr;
  const std::vector<cv::Point> * center_r = nullptr;
  double best_error = std::numeric_limits<double>::infinity();
  double smallest_r = std::numeric_limits<double>::infinity();
  for (const auto & contour : contours) {
    if (contour.size() <= 5) continue;
    const double area = cv::contourArea(contour);
    const double error = std::abs(area - expected_area) / expected_area;
    if (cv::pointPolygonTest(contour, armor_center, false) > 0 &&
        error <= armor_area_error_ && error < best_error) {
      armor = &contour;
      best_error = error;
    }
    // R 标轮廓必须包含 R 标关键点，并排除靶心中心和所有靶心边缘关键点。
    if (area > 0 && area < smallest_r && cv::pointPolygonTest(contour, p[2], false) > 0 &&
        cv::pointPolygonTest(contour, armor_center, false) < 0 &&
        cv::pointPolygonTest(contour, p[0], false) < 0 &&
        cv::pointPolygonTest(contour, p[1], false) < 0 &&
        cv::pointPolygonTest(contour, p[3], false) < 0 &&
        cv::pointPolygonTest(contour, p[4], false) < 0) {
      center_r = &contour;
      smallest_r = area;
    }
  }
  if (!armor || !center_r) return false;
  std::vector<cv::Point> hull;
  cv::convexHull(*armor, hull);
  const double hull_area = cv::contourArea(hull);
  if (hull_area <= 0 || cv::contourArea(*armor) / hull_area <= armor_solidity_ ||
      std::any_of(armor->begin(), armor->end(), [&](const auto & point) {
        return std::min({point.x, point.y, image.cols - 1 - point.x,
                         image.rows - 1 - point.y}) <= armor_border_margin_;
      }))
    return false;
  const auto ellipse = cv::fitEllipse(*armor);
  if (ellipse.size.width <= 0 || ellipse.size.height <= 0) return false;
  bull.center = ellipse.center;
  bull.active = candidate.active;
  bull.score = candidate.confidence;
  if (!project_bullseye_corners(ellipse, p, armor_center, bull.corners)) return false;
  icon = {cv::fitEllipse(*center_r).center, candidate.confidence};
  return true;
}

RuneElements SzuRuneDetector::detect(const cv::Mat & image)
{
  RuneElements result;
  if (image.empty()) return result;
  if (image.type() != CV_8UC3)
    throw std::invalid_argument("szu detector expects a BGR CV_8UC3 image");
  for (const auto & candidate : infer(image)) {
    RuneBullseye bull;
    RuneIcon icon;
    if (!refine(image, candidate, bull, icon)) continue;
    result.bullseyes.push_back(bull);
    if (std::none_of(result.icons.begin(), result.icons.end(), [&](const auto & previous) {
          return cv::norm(previous.center - icon.center) < nms_distance_;
        }))
      result.icons.push_back(icon);
  }
  return result;
}
}  // namespace auto_buff_v2
