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
// ROS-free port of the traditional light-bar detector (from
// hfut_rm_auto_aim_ws/src/rm_auto_aim/armor_detector). Differences from the
// ROS2 original:
// - class renamed Detector -> LightBarDetector (avoid the generic name);
// - debug message members (DebugLights/DebugArmors) removed;
// - input is BGR8 (the bridge wire format), classification runs sequentially
//   so no TBB/execution-policy dependency is introduced.

#ifndef ARMOR_DETECTOR_TRADITIONAL_LIGHT_BAR_DETECTOR_HPP_
#define ARMOR_DETECTOR_TRADITIONAL_LIGHT_BAR_DETECTOR_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include "rm_utils/common.hpp"

#include "armor_detector_traditional/light_corner_corrector.hpp"
#include "armor_detector_traditional/number_classifier.hpp"
#include "armor_detector_traditional/types.hpp"

namespace fyt::auto_aim {

class LightBarDetector {
 public:
  struct LightParams {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    // judge color
    int color_diff_thresh;
  };

  struct ArmorParams {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  LightBarDetector(const int &bin_thres, const EnemyColor &color,
                   const LightParams &l, const ArmorParams &a);

  // input must be BGR8.
  std::vector<Armor> detect(const cv::Mat &input) noexcept;

  cv::Mat preprocessImage(const cv::Mat &input) noexcept;
  std::vector<Light> findLights(const cv::Mat &bgr_img,
                                const cv::Mat &binary_img) noexcept;
  std::vector<Armor> matchLights(const std::vector<Light> &lights) noexcept;

  // For debug usage
  cv::Mat getAllNumbersImage() const noexcept;
  void drawResults(cv::Mat &img) const noexcept;

  // Parameters
  int binary_thres;
  EnemyColor detect_color;
  LightParams light_params;
  ArmorParams armor_params;

  std::unique_ptr<NumberClassifier> classifier;
  std::unique_ptr<LightCornerCorrector> corner_corrector;

  // Last binary image (debug visualization).
  cv::Mat binary_img;

 private:
  bool isLight(const Light &possible_light) noexcept;
  bool containLight(const int i, const int j, const std::vector<Light> &lights) noexcept;
  ArmorType isArmor(const Light &light_1, const Light &light_2) noexcept;

  cv::Mat gray_img_;

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_DETECTOR_TRADITIONAL_LIGHT_BAR_DETECTOR_HPP_
