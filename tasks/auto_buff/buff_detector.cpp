#include "buff_detector.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <cmath>

#include "tools/logger.hpp"

namespace auto_buff
{
namespace
{
RuneDetectorBackend parse_backend(const YAML::Node & yaml)
{
  const std::string backend = yaml["detector_backend"] ? yaml["detector_backend"].as<std::string>() : "szu";
  if (backend == "szu" || backend == "SZU") return SZU;
  if (backend == "sp25" || backend == "SP25") return SP25;
  throw std::invalid_argument("buff detector_backend 必须是 szu 或 sp25");
}

int yaml_int(const YAML::Node & yaml, const char * key, int fallback)
{
  return yaml[key] ? yaml[key].as<int>() : fallback;
}

bool yaml_bool(const YAML::Node & yaml, const char * key, bool fallback)
{
  return yaml[key] ? yaml[key].as<bool>() : fallback;
}

double yaml_double(const YAML::Node & yaml, const char * key, double fallback)
{
  return yaml[key] ? yaml[key].as<double>() : fallback;
}

void validate_szu_class_id(const char * key, int class_id)
{
  if (class_id < 0 || class_id > 2) {
    throw std::runtime_error(std::string(key) + " 必须在 0..2 范围内");
  }
}
}  // namespace

Buff_Detector::Buff_Detector(const std::string & config)
: config_path_(config), status_(LOSE), lose_(0)
{
  const auto yaml = YAML::LoadFile(config);
  backend_ = parse_backend(yaml);
  szu_target_class_id_ = yaml_int(yaml, "szu_target_class_id", szu_target_class_id_);
  szu_small_target_class_id_ =
    yaml_int(yaml, "szu_small_target_class_id", szu_target_class_id_);
  szu_big_target_class_id_ = yaml_int(yaml, "szu_big_target_class_id", szu_target_class_id_);
  validate_szu_class_id("szu_target_class_id", szu_target_class_id_);
  validate_szu_class_id("szu_small_target_class_id", szu_small_target_class_id_);
  validate_szu_class_id("szu_big_target_class_id", szu_big_target_class_id_);
  szu_debug_log_ = yaml_bool(yaml, "szu_debug_log", szu_debug_log_);
  szu_debug_log_every_n_ =
    std::max(1, yaml_int(yaml, "szu_debug_log_every_n", szu_debug_log_every_n_));
  szu_reject_r_geometry_ =
    yaml_bool(yaml, "szu_reject_r_geometry", szu_reject_r_geometry_);
  szu_r_center_max_spread_px_ = yaml_double(
    yaml, "szu_r_center_max_spread_px", szu_r_center_max_spread_px_);
  szu_min_target_radius_px_ = yaml_double(
    yaml, "szu_min_target_radius_px", szu_min_target_radius_px_);
  if (!std::isfinite(szu_r_center_max_spread_px_) || szu_r_center_max_spread_px_ <= 0.0 ||
      !std::isfinite(szu_min_target_radius_px_) || szu_min_target_radius_px_ <= 0.0) {
    throw std::invalid_argument("SZU R 点几何阈值必须是有限正数");
  }
  if (backend_ == SZU) {
    szu_detector_ = std::make_unique<SzuRuneDetector>(config);
  } else {
    sp25_detector_ = std::make_unique<YOLO11_BUFF>(config);
  }
}

void Buff_Detector::handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img)
{
  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);  // 彩色图转灰度图
  // cv::imshow("gray", gray_img);  // 调试用

  // 进行二值化           :把高于100变成255，低于100变成0
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 100, 255, cv::THRESH_BINARY);
  // cv::imshow("binary", binary_img);  // 调试用

  // 膨胀
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));  // 使用矩形核
  cv::dilate(binary_img, dilated_img, kernel, cv::Point(-1, -1), 1);
  // cv::imshow("Dilated Image", dilated_img);  // 调试用
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img)
{
  /// error

  if (fanblades.empty()) {
    tools::logger()->debug("[Buff_Detector] 无法计算r_center!");
    return {0, 0};
  }

  /// 算出大概位置

  cv::Point2f r_center_t = {0, 0};
  for (auto & fanblade : fanblades) {
    auto point5 = fanblade.points[4];  // point5是扇叶的中心
    auto point6 = fanblade.points[5];
    r_center_t += (point6 - point5) * 1.4 + point5;  // TODO
    // r_center_t += 4.7 * point - (4.7 - 1) * fanblade.center;
  }
  r_center_t /= float(fanblades.size());

  /// 处理图片,mask选出大概范围

  cv::Mat dilated_img;
  handle_img(bgr_img, dilated_img);
  double radius = cv::norm(fanblades[0].points[2] - fanblades[0].center) * 0.8;
  cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);  // mask
  circle(mask, r_center_t, radius, cv::Scalar(255), -1);
  bitwise_and(dilated_img, mask, dilated_img);               // 将遮罩应用于二值化图像
  tools::draw_point(bgr_img, r_center_t, {255, 255, 0}, 5);  // 调试用
  // cv::imshow("Dilated Image", dilated_img);                // 调试用

  /// 获取轮廓点,矩阵框筛选  TODO

  std::vector<std::vector<cv::Point>> contours;
  auto r_center = r_center_t;
  cv::findContours(
    dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // external找外部区域
  double ratio_1 = INF;
  for (auto & it : contours) {
    auto rotated_rect = cv::minAreaRect(it);
    double ratio = rotated_rect.size.height > rotated_rect.size.width
                     ? rotated_rect.size.height / rotated_rect.size.width
                     : rotated_rect.size.width / rotated_rect.size.height;
    ratio += cv::norm(rotated_rect.center - r_center_t) / (radius / 3);
    if (ratio < ratio_1) {
      ratio_1 = ratio;
      r_center = rotated_rect.center;
    }
  }
  return r_center;
};

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
    last_powerrune_ = std::nullopt;
    return;
  }
  status_ = TEM_LOSE;
}

std::optional<PowerRune> Buff_Detector::detect_24(cv::Mat & bgr_img)
{
  return detect_sp25(bgr_img, true);
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  return detect_sp25(bgr_img, false);
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img, PowerRune_type rune_type)
{
  if (backend_ == SZU) return detect_szu(bgr_img, rune_type);
  return detect_sp25(bgr_img, rune_type == BIG);
}

std::optional<PowerRune> Buff_Detector::detect_sp25(cv::Mat & bgr_img, bool multi_candidate)
{
  /// onnx 模型检测

  if (!sp25_detector_) sp25_detector_ = std::make_unique<YOLO11_BUFF>(config_path_);
  std::vector<YOLO11_BUFF::Object> results = multi_candidate
                                               ? sp25_detector_->get_multicandidateboxes(bgr_img)
                                               : sp25_detector_->get_onecandidatebox(bgr_img);

  /// 处理未获得的情况

  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades;
  for (auto & result : results) {
    const FanBlade_type type = fanblades.empty() ? _target : _light;
    fanblades.emplace_back(
      FanBlade(result.kpt, result.kpt[4], type, result.label, result.prob));
  }

  /// 生成PowerRune
  auto r_center = get_r_center(fanblades, bgr_img);
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

std::optional<PowerRune> Buff_Detector::detect_szu(cv::Mat & bgr_img, PowerRune_type rune_type)
{
  if (!szu_detector_) return std::nullopt;
  const auto results = szu_detector_->detect(bgr_img, rune_type == BIG);
  const auto & stats = szu_detector_->debug_stats();
  if (results.empty()) {
    log_szu_debug("no_result", stats, results.size(), 0, 0);
    handle_lose();
    return std::nullopt;
  }

  std::vector<FanBlade> target_fanblades;
  std::vector<FanBlade> other_fanblades;
  target_fanblades.reserve(results.size());
  other_fanblades.reserve(results.size());
  cv::Point2f r_center_sum(0.0f, 0.0f);
  double quality_sum = 0.0;
  int r_center_count = 0;
  std::vector<std::pair<cv::Point2f, double>> network_r_samples;
  std::vector<std::pair<cv::Point2f, double>> geometric_r_samples;
  std::vector<std::pair<cv::Point2f, double>> visual_r_samples;
  double network_r_confidence_sum = 0.0;
  double network_r_confidence_weight = 0.0;
  struct TargetRSample
  {
    cv::Point2f blade_center{0.0F, 0.0F};
    cv::Point2f r_center{0.0F, 0.0F};
  };
  std::vector<TargetRSample> target_r_samples;

  for (const auto & result : results) {
    // 只保留模型提供了有效符心观测的检测结果。
    if (result.corners.size() != 4 || !result.network_r_valid) continue;
    FanBlade blade(
      result.corners, result.center, classify_szu_blade(result.class_id, rune_type), result.class_id,
      result.confidence);
    blade.point_indices = result.corner_indices;
    const double quality = std::max(1e-3, static_cast<double>(result.quality));
    r_center_sum += result.r_center * static_cast<float>(quality);
    quality_sum += quality;
    ++r_center_count;
    if (result.network_r_valid) {
      network_r_samples.emplace_back(result.network_r_center, quality);
      network_r_confidence_sum += result.network_r_confidence * quality;
      network_r_confidence_weight += quality;
    }
    if (result.geometric_r_valid) {
      geometric_r_samples.emplace_back(result.geometric_r_center, quality);
    }
    if (result.visual_r_valid) {
      visual_r_samples.emplace_back(result.visual_r_center, quality);
    }
    if (blade.type == _target) {
      // PnP 只使用当前目标叶片的角点，R 也必须优先取同一叶片的检测结果。
      target_r_samples.push_back({result.center, result.r_center});
      target_fanblades.emplace_back(std::move(blade));
    } else {
      other_fanblades.emplace_back(std::move(blade));
    }
  }

  if (target_fanblades.empty() || r_center_count == 0) {
    log_szu_debug(
      "target_missing", stats, results.size(), target_fanblades.size(), other_fanblades.size());
    handle_lose();
    return std::nullopt;
  }

  std::vector<FanBlade> fanblades;
  fanblades.reserve(target_fanblades.size() + other_fanblades.size());
  std::sort(target_fanblades.begin(), target_fanblades.end(), [](const FanBlade & a, const FanBlade & b) {
    return a.confidence > b.confidence;
  });
  if (last_powerrune_.has_value() && !last_powerrune_->fanblades.empty()) {
    const auto last_target_center = last_powerrune_->fanblades[0].center;
    auto closest_target = std::min_element(
      target_fanblades.begin(), target_fanblades.end(), [&](const FanBlade & a, const FanBlade & b) {
        return cv::norm(a.center - last_target_center) < cv::norm(b.center - last_target_center);
      });
    if (closest_target != target_fanblades.end()) std::iter_swap(target_fanblades.begin(), closest_target);
  }
  fanblades.insert(fanblades.end(), target_fanblades.begin(), target_fanblades.end());
  fanblades.insert(fanblades.end(), other_fanblades.begin(), other_fanblades.end());

  cv::Point2f r_center = r_center_sum * static_cast<float>(1.0 / quality_sum);
  std::vector<cv::Point2f> rp26_pose_image_points;
  bool rp26_pose_valid = false;
  if (!target_fanblades.empty()) {
    // 目标排序完成后，重新从同一检测结果取 RP26 锚点，避免多叶片时把锚点和 R 错配。
    const cv::Point2f selected_target_center = target_fanblades.front().center;
    const auto selected_result = std::min_element(
      results.begin(), results.end(), [&](const SzuRuneDetector::Detection & a,
                                          const SzuRuneDetector::Detection & b) {
        const bool a_target = classify_szu_blade(a.class_id, rune_type) == _target;
        const bool b_target = classify_szu_blade(b.class_id, rune_type) == _target;
        if (a_target != b_target) return a_target;
        return cv::norm(a.center - selected_target_center) <
               cv::norm(b.center - selected_target_center);
      });
    if (selected_result != results.end() && selected_result->rp26_semantic_valid &&
        selected_result->rp26_anchor_points.size() == 4) {
      rp26_pose_image_points = selected_result->rp26_anchor_points;
      rp26_pose_valid = true;
    }
  }
  if (!target_r_samples.empty()) {
    // 目标重排后，选择与最终 target 中心最近的 R 观测，避免多叶片平均把 PnP 的 R 约束错配。
    const auto & selected_target_center = target_fanblades.front().center;
    const auto selected_target_r = std::min_element(
      target_r_samples.begin(), target_r_samples.end(), [&](const TargetRSample & a,
                                                              const TargetRSample & b) {
        return cv::norm(a.blade_center - selected_target_center) <
               cv::norm(b.blade_center - selected_target_center);
      });
    if (selected_target_r != target_r_samples.end() &&
        std::isfinite(selected_target_r->r_center.x) &&
        std::isfinite(selected_target_r->r_center.y)) {
      r_center = selected_target_r->r_center;
    }
  }
  const auto summarize_r_source = [](
                                 const std::vector<std::pair<cv::Point2f, double>> & samples) {
    RPointSourceSummary summary;
    double weight_sum = 0.0;
    for (const auto & sample : samples) {
      summary.center += sample.first * static_cast<float>(sample.second);
      weight_sum += sample.second;
    }
    if (weight_sum <= 0.0) return summary;
    summary.center *= static_cast<float>(1.0 / weight_sum);
    summary.count = static_cast<int>(samples.size());
    for (const auto & sample : samples) {
      summary.spread_px = std::max(summary.spread_px, cv::norm(sample.first - summary.center));
    }
    return summary;
  };
  double r_center_spread = 0.0;
  for (const auto & result : results) {
    // 只统计已经进入正常打符链路的网络 R；被过滤的检测不能污染 R 一致性诊断。
    if (result.corners.size() == 4 && result.network_r_valid) {
      r_center_spread = std::max(r_center_spread, cv::norm(result.r_center - r_center));
    }
  }
  const auto target_center = target_fanblades.front().center;
  const double target_radius = cv::norm(target_center - r_center);
  const bool r_geometry_nonfinite = !std::isfinite(target_radius) ||
                                    !std::isfinite(r_center_spread);
  const bool r_geometry_outlier = target_radius < szu_min_target_radius_px_ ||
                                  r_center_spread > szu_r_center_max_spread_px_;
  // 先保留诊断信息；只有显式打开开关时才用这些经验阈值丢弃整帧。
  if (r_geometry_nonfinite || (szu_reject_r_geometry_ && r_geometry_outlier)) {
    log_szu_debug(
      "geometry_invalid", stats, results.size(), target_fanblades.size(), other_fanblades.size());
    handle_lose();
    return std::nullopt;
  }
  PowerRune powerrune(fanblades, r_center, last_powerrune_);
  powerrune.observation_quality = quality_sum / static_cast<double>(r_center_count);
  powerrune.r_center_consistency_px = target_radius;
  powerrune.r_center_spread_px = r_center_spread;
  powerrune.r_point_diagnostics.network = summarize_r_source(network_r_samples);
  powerrune.r_point_diagnostics.geometry = summarize_r_source(geometric_r_samples);
  powerrune.r_point_diagnostics.visual = summarize_r_source(visual_r_samples);
  powerrune.r_point_diagnostics.detection_count = r_center_count;
  powerrune.rp26_pose_image_points = std::move(rp26_pose_image_points);
  powerrune.rp26_pose_preferred = rune_type == SMALL;
  powerrune.rp26_pose_valid = rp26_pose_valid;
  if (network_r_confidence_weight > 0.0) {
    powerrune.r_point_diagnostics.network_confidence =
      network_r_confidence_sum / network_r_confidence_weight;
  }

  /// handle error
  if (powerrune.is_unsolve()) {
    log_szu_debug(
      "powerrune_unsolve", stats, results.size(), target_fanblades.size(),
      other_fanblades.size());
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  log_szu_debug("ok", stats, results.size(), target_fanblades.size(), other_fanblades.size());
  return P;
}

FanBlade_type Buff_Detector::classify_szu_blade(int class_id, PowerRune_type rune_type) const
{
  const int target_class = rune_type == SMALL ? szu_small_target_class_id_ : szu_big_target_class_id_;
  return class_id == target_class ? _target : _light;
}

void Buff_Detector::log_szu_debug(
  const char * stage, const SzuRuneDetector::DebugStats & stats, std::size_t raw_count,
  std::size_t target_count, std::size_t other_count)
{
  if (!szu_debug_log_) return;
  ++szu_debug_frame_;
  if (szu_debug_frame_ != 1 && szu_debug_frame_ % szu_debug_log_every_n_ != 0) return;

  tools::logger()->info(
    "[Buff_Detector] SZU frame={} stage={} results={} target={} other={} "
    "RP26={}/{} max_conf={:.3f} max_kpt={:.3f}",
    szu_debug_frame_, stage, raw_count, target_count, other_count,
    stats.rp26_valid, stats.rp26_attempted, stats.max_confidence,
    stats.max_keypoint_confidence);
}

std::optional<PowerRune> Buff_Detector::detect_debug(cv::Mat & bgr_img, cv::Point2f v)
{
  /// onnx 模型检测

  if (!sp25_detector_) sp25_detector_ = std::make_unique<YOLO11_BUFF>(config_path_);
  std::vector<YOLO11_BUFF::Object> results = sp25_detector_->get_multicandidateboxes(bgr_img);

  /// 处理未获得的情况

  if (results.empty()) return std::nullopt;

  /// results转扇叶FanBlade

  std::vector<FanBlade> fanblades_t;
  for (auto & result : results)
    fanblades_t.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));

  /// 计算r_center,筛选fanblade
  auto r_center = get_r_center(fanblades_t, bgr_img);
  std::vector<FanBlade> fanblades;
  for (auto & fanblade : fanblades_t) {
    if (cv::norm((fanblade.center - r_center) - v) < 10 || results.size() == 1) {
      fanblades.emplace_back(fanblade);
      break;
    }
  }
  if (fanblades.empty()) return std::nullopt;
  PowerRune powerrune(fanblades, r_center, std::nullopt);

  std::optional<PowerRune> P;
  P.emplace(powerrune);
  return P;
}

}  // namespace auto_buff
