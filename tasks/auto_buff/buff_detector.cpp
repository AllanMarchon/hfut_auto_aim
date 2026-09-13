#include "buff_detector.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

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
  const auto results = szu_detector_->detect(bgr_img);
  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  std::vector<FanBlade> target_fanblades;
  std::vector<FanBlade> other_fanblades;
  target_fanblades.reserve(results.size());
  other_fanblades.reserve(results.size());
  cv::Point2f r_center_sum(0.0f, 0.0f);
  int r_center_count = 0;

  for (const auto & result : results) {
    if (result.corners.size() != 4) continue;
    FanBlade blade(
      result.corners, result.center, classify_szu_blade(result.class_id, rune_type), result.class_id,
      result.confidence);
    r_center_sum += result.r_center;
    ++r_center_count;
    if (blade.type == _target) {
      target_fanblades.emplace_back(std::move(blade));
    } else {
      other_fanblades.emplace_back(std::move(blade));
    }
  }

  if (target_fanblades.empty() || r_center_count == 0) {
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

  const auto r_center = r_center_sum * (1.0f / static_cast<float>(r_center_count));
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

FanBlade_type Buff_Detector::classify_szu_blade(int class_id, PowerRune_type rune_type) const
{
  const int target_class = rune_type == SMALL ? szu_small_target_class_id_ : szu_big_target_class_id_;
  return class_id == target_class ? _target : _light;
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
