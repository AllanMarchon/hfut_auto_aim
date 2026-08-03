// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ROS-free port of the traditional light-bar armor types (from
// hfut_rm_auto_aim_ws/src/rm_auto_aim/armor_detector). Object-point builders
// and the unused Sophus include were dropped: pose estimation reuses the
// shared ArmorPoseEstimatorAdapter.

#ifndef ARMOR_DETECTOR_TRADITIONAL_TYPES_HPP_
#define ARMOR_DETECTOR_TRADITIONAL_TYPES_HPP_

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"

namespace fyt::auto_aim {

// Armor size, Unit: m
constexpr double SMALL_ARMOR_WIDTH = 133.0 / 1000.0;  // 135
constexpr double SMALL_ARMOR_HEIGHT = 50.0 / 1000.0;  // 55
constexpr double LARGE_ARMOR_WIDTH = 225.0 / 1000.0;
constexpr double LARGE_ARMOR_HEIGHT = 50.0 / 1000.0;  // 55

// Armor type
enum class ArmorType { SMALL, LARGE, INVALID };
inline std::string armorTypeToString(const ArmorType &type) {
  switch (type) {
    case ArmorType::SMALL:
      return "small";
    case ArmorType::LARGE:
      return "large";
    default:
      return "invalid";
  }
}

// Struct used to store the light bar
struct Light : public cv::RotatedRect {
  Light() = default;
  explicit Light(const std::vector<cv::Point> &contour)
  : cv::RotatedRect(cv::minAreaRect(contour)), color(EnemyColor::WHITE) {
    FYT_ASSERT(contour.size() > 0);

    center = std::accumulate(
      contour.begin(),
      contour.end(),
      cv::Point2f(0, 0),
      [n = static_cast<float>(contour.size())](const cv::Point2f &a, const cv::Point &b) {
        return a + cv::Point2f(b.x, b.y) / n;
      });

    cv::Point2f p[4];
    this->points(p);
    std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    axis = top - bottom;
    axis = axis / cv::norm(axis);

    // Calculate the tilt angle
    // The angle is the angle between the light bar and the horizontal line
    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }
  EnemyColor color;
  cv::Point2f top, bottom, center;
  cv::Point2f axis;
  double length;
  double width;
  float tilt_angle;
};

// Struct used to store the armor
struct Armor {
  Armor() = default;
  Armor(const Light &l1, const Light &l2) {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }

    center = (left_light.center + right_light.center) / 2;
  }

  // Corner points in the shared pose-adapter order: LB, LT, RT, RB.
  std::array<cv::Point2f, 4> corners() const {
    return {left_light.bottom, left_light.top, right_light.top, right_light.bottom};
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type = ArmorType::INVALID;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence = 0.0F;
  std::string classfication_result;
};

}  // namespace fyt::auto_aim
#endif  // ARMOR_DETECTOR_TRADITIONAL_TYPES_HPP_
