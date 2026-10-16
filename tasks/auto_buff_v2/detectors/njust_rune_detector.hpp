#ifndef AUTO_BUFF_V2__NJUST_RUNE_DETECTOR_HPP
#define AUTO_BUFF_V2__NJUST_RUNE_DETECTOR_HPP

#include <optional>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rune_detector.hpp"

namespace auto_buff_v2
{
struct NjustContourMeasurement
{
  cv::Point2f center;
  double radius = 0;
  double area = 0;
  bool radius_pass = false;
  bool icon_area_pass = false;
};

struct NjustIconScoreMeasurement
{
  cv::Point2f center;
  double score = 0;
  int endpoints = 0;
  int lower_endpoints = 0;
  int branches = 0;
  int holes = 0;
};

struct NjustDetectorMeasurements
{
  double min_radius = 0;
  double max_radius = 0;
  double min_icon_area = 0;
  double max_icon_area = 0;
  double match_threshold = 0;
  std::vector<NjustContourMeasurement> candidates;
  std::vector<cv::Rect> icon_rois;
  std::vector<NjustIconScoreMeasurement> icon_scores;
};

class NjustRuneDetector : public RuneDetector
{
public:
  struct Config
  {
    double min_distance = 1.7;
    double max_distance = 5.0;
    double active_threshold = 0.2;
    double match_threshold = 0.5;
    double max_perspective = 60.0;
    double fx = 0;
    double fy = 0;
    bool enemy_red = false;

    static Config load(const YAML::Node & node);
  } config;

  void set_enemy_red(bool enemy_red) override { config.enemy_red = enemy_red; }
  RuneElements detect(const cv::Mat & image) override;

  // 调试入口 emplace 后采集诊断数据，主链路默认不采集。
  std::optional<NjustDetectorMeasurements> measurements;
};

}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__NJUST_RUNE_DETECTOR_HPP
