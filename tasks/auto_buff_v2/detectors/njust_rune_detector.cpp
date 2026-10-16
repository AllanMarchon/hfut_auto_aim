#include "njust_rune_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

namespace auto_buff_v2
{
NjustRuneDetector::Config NjustRuneDetector::Config::load(const YAML::Node & node)
{
  Config result;
  if (!node) return result;
  result.min_distance = node["min_distance"].as<double>(result.min_distance);
  result.max_distance = node["max_distance"].as<double>(result.max_distance);
  result.active_threshold = node["active_threshold"].as<double>(result.active_threshold);
  result.match_threshold = node["match_threshold"].as<double>(result.match_threshold);
  result.max_perspective = node["max_perspective"].as<double>(result.max_perspective);
  return result;
}

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kRadialBins = 32;
constexpr int kThetaBins = 180;
using PolarBins = std::array<double, kRadialBins * kThetaBins>;

cv::Mat extract_channel(const cv::Mat & image, bool enemy_red)
{
  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  const int primary = enemy_red ? 2 : 0;
  const int other = enemy_red ? 0 : 2;
  cv::Mat difference, purity, bright, result;
  cv::subtract(channels[primary], channels[other], difference);
  cv::subtract(channels[primary], channels[1], purity);
  cv::threshold(difference, difference, 40, 255, cv::THRESH_BINARY);
  cv::threshold(purity, purity, 20, 255, cv::THRESH_BINARY);
  cv::threshold(channels[primary], bright, 80, 255, cv::THRESH_BINARY);
  cv::bitwise_and(difference, purity, result);
  cv::bitwise_and(result, bright, result);
  cv::dilate(result, result, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
  return result;
}

double icon_score(const cv::Mat & image, NjustIconScoreMeasurement * measurement)
{
  cv::Mat gray, binary, skeleton;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::ximgproc::thinning(binary, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);
  cv::Mat labels;
  if (cv::connectedComponents(skeleton, labels, 8) != 2) return 0;
  int endpoints = 0, lower_endpoints = 0, branches = 0;
  for (int y = 1; y < skeleton.rows - 1; ++y) {
    for (int x = 1; x < skeleton.cols - 1; ++x) {
      if (!skeleton.at<std::uint8_t>(y, x)) continue;
      int neighbors = 0;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
          if ((dx || dy) && skeleton.at<std::uint8_t>(y + dy, x + dx)) ++neighbors;
      if (neighbors == 1) {
        ++endpoints;
        if (y >= skeleton.rows / 2) ++lower_endpoints;
      } else if (neighbors >= 3) {
        ++branches;
      }
    }
  }
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  int largest_outer = -1;
  double largest_area = 0;
  for (std::size_t i = 0; i < contours.size(); ++i) {
    if (hierarchy[i][3] != -1) continue;
    const double area = cv::contourArea(contours[i]);
    if (largest_outer == -1 || area > largest_area) {
      largest_area = area;
      largest_outer = static_cast<int>(i);
    }
  }
  int holes = 0;
  for (const auto & h : hierarchy)
    if (h[3] == largest_outer) ++holes;
  if (measurement) {
    measurement->endpoints = endpoints;
    measurement->lower_endpoints = lower_endpoints;
    measurement->branches = branches;
    measurement->holes = holes;
  }
  if (endpoints >= 1 && lower_endpoints >= 1 && branches >= 8 && branches <= 50 && holes <= 2)
    return 0.5 + 0.1 * endpoints;
  return 0;
}

double arm_length(const PolarBins & polar, int theta_bin, double radius)
{
  std::array<double, kRadialBins> profile{};
  for (int r = 0; r < kRadialBins; ++r)
    for (int dt = -3; dt <= 3; ++dt)
      profile[r] += polar[r * kThetaBins +
                          ((theta_bin + dt) % kThetaBins + kThetaBins) % kThetaBins];
  const double threshold = *std::max_element(profile.begin(), profile.end()) * 0.25;
  int boundary = -1;
  for (int r = kRadialBins - 1; r >= 0; --r)
    if (profile[r] >= threshold) { boundary = r; break; }
  double exact = 0;
  if (boundary >= kRadialBins - 1) {
    exact = 1;
  } else if (boundary < 0) {
    exact = std::distance(profile.begin(), std::max_element(profile.begin(), profile.end())) /
            static_cast<double>(kRadialBins);
  } else {
    const double inside = profile[boundary];
    const double outside = profile[boundary + 1];
    exact = inside > outside ?
      (boundary + 1.0 - (threshold - outside) / (inside - outside)) / kRadialBins :
      (boundary + 1.0) / kRadialBins;
  }
  return radius * exact;
}

std::optional<RuneBullseye> bullseye_feature(
  const cv::Mat & image, const cv::Rect & roi, const std::vector<cv::Point> & contour,
  cv::Point2f center, double active_threshold)
{
  auto local_contour = contour;
  for (auto & point : local_contour) point -= roi.tl();
  cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
  cv::drawContours(mask, std::vector<std::vector<cv::Point>>{local_contour}, -1,
                   {255}, cv::FILLED);
  cv::Mat masked;
  image(roi).copyTo(masked, mask);
  cv::Mat gray;
  cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);
  const int square_size = std::max(gray.rows, gray.cols);
  const int top = (square_size - gray.rows) / 2;
  const int bottom = square_size - gray.rows - top;
  const int left = (square_size - gray.cols) / 2;
  const int right = square_size - gray.cols - left;
  cv::copyMakeBorder(gray, gray, top, bottom, left, right, cv::BORDER_CONSTANT, 0);
  const cv::Point2f local_center(center.x - roi.x + left, center.y - roi.y + top);
  const double radius = square_size * 0.5;
  PolarBins polar{};
  for (int y = 0; y < gray.rows; ++y) {
    for (int x = 0; x < gray.cols; ++x) {
      const double dx = x - local_center.x;
      const double dy = y - local_center.y;
      const double r = std::hypot(dx, dy);
      if (r >= radius) continue;
      const int rb = std::clamp(static_cast<int>(r / radius * kRadialBins), 0, kRadialBins - 1);
      const int tb = std::clamp(static_cast<int>((std::atan2(dy, dx) + kPi) /
                                                 (2 * kPi) * kThetaBins), 0, kThetaBins - 1);
      polar[rb * kThetaBins + tb] += gray.at<std::uint8_t>(y, x);
    }
  }
  std::array<double, kThetaBins> angular{};
  std::array<double, kRadialBins> radial{};
  for (int r = 0; r < kRadialBins; ++r)
    for (int t = 0; t < kThetaBins; ++t) {
      angular[t] += polar[r * kThetaBins + t] * (r + 1);
      radial[r] += polar[r * kThetaBins + t];
    }
  double sum_cos = 0, sum_sin = 0;
  for (int t = 0; t < kThetaBins; ++t) {
    const double angle = 4.0 * t / kThetaBins * 2 * kPi;
    sum_cos += angular[t] * std::cos(angle);
    sum_sin += angular[t] * std::sin(angle);
  }
  const double phase = std::atan2(sum_sin, sum_cos) / 4.0;
  double peaks = 0, valleys = 0;
  for (int p = 0; p < 4; ++p) {
    for (int dt = -2; dt <= 2; ++dt) {
      const auto at = [&](double theta) {
        const int index = static_cast<int>(std::llround((theta + kPi) /
                                                       (2 * kPi) * kThetaBins));
        return angular[((index + dt) % kThetaBins + kThetaBins) % kThetaBins];
      };
      peaks += at(phase + p * kPi / 2);
      valleys += at(phase + (p + 0.5) * kPi / 2);
    }
  }
  const double pvd = (peaks - valleys) / (peaks + valleys + 1e-10);
  RuneBullseye result;
  result.center = center;
  result.score = pvd;
  result.active = pvd < active_threshold;
  if (result.active) {
    const int radial_peak =
      std::distance(radial.begin(), std::max_element(radial.begin(), radial.end()));
    const double arm = radius * (radial_peak + 1.0) / kRadialBins;
    for (int p = 0; p < 4; ++p) {
      const double angle = -kPi / 2 + p * kPi / 2;
      result.corners[p] = center + cv::Point2f(arm * std::cos(angle), arm * std::sin(angle));
    }
    return result;
  }
  std::array<double, 4> lengths{};
  double mean = 0;
  for (int p = 0; p < 4; ++p) {
    const double angle = phase + p * kPi / 2;
    const int theta_bin =
      static_cast<int>(std::llround((angle + kPi) / (2 * kPi) * kThetaBins));
    lengths[p] = arm_length(polar, theta_bin, radius);
    mean += lengths[p];
    result.corners[p] =
      center + cv::Point2f(lengths[p] * std::cos(angle), lengths[p] * std::sin(angle));
  }
  mean *= 0.25;
  double variance = 0;
  for (const double length : lengths) variance += (length - mean) * (length - mean);
  if (std::sqrt(variance * 0.25) > mean * 0.3) return std::nullopt;
  return result;
}
}  // namespace

RuneElements NjustRuneDetector::detect(const cv::Mat & image)
{
  RuneElements result;
  if (measurements) {
    *measurements = {};
    measurements->match_threshold = config.match_threshold;
  }
  if (image.empty() || image.type() != CV_8UC3 || config.fx <= 0 || config.fy <= 0) return result;
  auto binary = extract_channel(image, config.enemy_red);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const double focal = (config.fx + config.fy) * 0.5;
  const double cosine = std::cos(config.max_perspective * kPi / 180);
  const double min_radius = focal * kRuneBullseyeRadius * cosine /
                            config.max_distance * 0.9;
  const double max_radius = focal * kRuneBullseyeRadius /
                            (config.min_distance * cosine) * 1.1;
  const double min_icon_area = focal * focal * 0.05 * 0.05 * cosine /
                               (config.max_distance * config.max_distance) * 0.9 * kPi * 0.2;
  const double max_icon_area = focal * focal * 0.05 * 0.05 /
                               (config.min_distance * config.min_distance * cosine) * 1.1 * kPi;
  if (measurements) {
    measurements->min_radius = min_radius;
    measurements->max_radius = max_radius;
    measurements->min_icon_area = min_icon_area;
    measurements->max_icon_area = max_icon_area;
  }
  struct BullCandidate
  {
    std::vector<cv::Point> contour;
    cv::Point2f center;
    double major;
  };
  std::vector<BullCandidate> bull_candidates;
  std::vector<std::vector<cv::Point>> icon_candidates;
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;
    const double area = cv::contourArea(contour);
    const double perimeter = cv::arcLength(contour, true);
    if (area <= 0 || perimeter <= 0) continue;
    const double radius = std::sqrt(area / kPi);
    const bool radius_pass = radius >= min_radius && radius <= max_radius;
    if (measurements) {
      measurements->candidates.push_back(
        {cv::minAreaRect(contour).center, radius, area,
         radius_pass,
         area >= min_icon_area && area <= max_icon_area});
    }
    const auto ellipse = cv::fitEllipse(contour);
    const double major = std::max(ellipse.size.width, ellipse.size.height);
    const double minor = std::min(ellipse.size.width, ellipse.size.height);
    if (major <= 0 || minor / major < cosine) continue;
    if (radius >= min_radius && radius <= max_radius) {
      const double ellipse_area = kPi * major * minor * 0.25;
      const double circularity = 4 * kPi * area / (perimeter * perimeter);
      if (area / ellipse_area < 0.7 || area / ellipse_area > 1.3 ||
          circularity < 0.7 * 2 * cosine / (1 + cosine * cosine)) continue;
      const auto moments = cv::moments(contour);
      const cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
      bull_candidates.push_back({contour, center, major});
    } else if (area >= min_icon_area && area <= max_icon_area) {
      icon_candidates.push_back(contour);
    }
  }
  std::vector<bool> grouped(bull_candidates.size(), false);
  std::vector<BullCandidate> selected_bulls;
  for (std::size_t i = 0; i < bull_candidates.size(); ++i) {
    if (grouped[i]) continue;
    grouped[i] = true;
    std::size_t largest = i;
    for (std::size_t j = i + 1; j < bull_candidates.size(); ++j) {
      if (grouped[j]) continue;
      const auto & a = bull_candidates[i];
      const auto & b = bull_candidates[j];
      if (cv::norm(a.center - b.center) < std::max(a.major, b.major) * 0.15) {
        grouped[j] = true;
        if (b.major > bull_candidates[largest].major) largest = j;
      }
    }
    selected_bulls.push_back(bull_candidates[largest]);
  }
  for (const auto & bull : selected_bulls) {
    auto roi = cv::boundingRect(bull.contour);
    roi.x -= 20;
    roi.y -= 20;
    roi.width += 40;
    roi.height += 40;
    roi &= cv::Rect(0, 0, image.cols, image.rows);
    const auto feature = bullseye_feature(image, roi, bull.contour, bull.center,
                                         config.active_threshold);
    if (feature) result.bullseyes.push_back(*feature);
  }
  for (const auto & contour : icon_candidates) {
    const cv::Point2f center = cv::minAreaRect(contour).center;
    const bool inside_bull = std::any_of(selected_bulls.begin(), selected_bulls.end(),
                                         [&](const auto & bull) {
                                           return cv::pointPolygonTest(bull.contour, center,
                                                                       false) >= 0;
                                         });
    if (inside_bull) continue;
    auto roi = cv::boundingRect(contour);
    roi.x -= 5;
    roi.y -= 5;
    roi.width += 10;
    roi.height += 10;
    roi &= cv::Rect(0, 0, image.cols, image.rows);
    NjustIconScoreMeasurement icon_measurement;
    const double score = icon_score(image(roi), measurements ? &icon_measurement : nullptr);
    if (measurements) {
      icon_measurement.center = center;
      icon_measurement.score = score;
      measurements->icon_scores.push_back(icon_measurement);
    }
    if (score >= config.match_threshold) {
      result.icons.push_back({center, score});
      if (measurements) measurements->icon_rois.push_back(roi);
    }
  }
  return result;
}
}  // namespace auto_buff_v2
