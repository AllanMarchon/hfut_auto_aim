#ifndef AUTO_BUFF_V2__RUNE_HPP
#define AUTO_BUFF_V2__RUNE_HPP

#include <array>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace auto_buff_v2
{
constexpr double kRuneGlobalRadius = 0.7;
constexpr double kRuneBullseyeRadius = 0.15;
constexpr double kRuneIconProminentDistance = 0.1;

struct RuneBullseye
{
  cv::Point2f center;
  std::array<cv::Point2f, 4> corners;
  bool active = false;
  double score = 0;
};

struct RuneIcon
{
  cv::Point2f center;
  double score = 0;
};

// 保留网络候选结果，供选择靶心前的可视化使用。
struct RuneCandidate
{
  cv::Rect box;
  float score;
  std::array<cv::Point2f, 6> points;  // 上、左、下、右、靶心、R 标。
};

struct RuneElements
{
  std::vector<RuneBullseye> bullseyes;
  std::vector<RuneIcon> icons;
  // 以下服务于 foxglove 可视化
  std::vector<RuneCandidate> candidates;
  std::optional<std::vector<std::vector<cv::Point>>> climber_contours;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__RUNE_HPP
