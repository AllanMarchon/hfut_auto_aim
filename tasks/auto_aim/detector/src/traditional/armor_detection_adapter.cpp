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

#include "armor_detector_traditional/armor_detection_adapter.hpp"

#include <algorithm>

namespace fyt::auto_aim {

ArmorDetection toArmorDetection(const Armor &armor) {
  ArmorDetection detection;
  detection.model_label = armor.number;
  detection.publish_number = armor.number;
  detection.publish_type = armorTypeToString(armor.type);
  detection.color = armor.left_light.color;
  detection.confidence = armor.confidence;
  detection.keypoints = armor.corners();  // LB, LT, RT, RB (adapter convention)
  detection.center = armor.center;
  detection.bbox = cv::boundingRect(
      std::vector<cv::Point2f>(detection.keypoints.begin(), detection.keypoints.end()));
  // The traditional chain has no 2D tracker; pose estimation takes the
  // track-less path, same as untracked NN detections.
  detection.track_id = -1;
  return detection;
}

std::vector<ArmorDetection> toArmorDetections(const std::vector<Armor> &armors) {
  std::vector<ArmorDetection> detections;
  detections.reserve(armors.size());
  for (const auto &armor : armors) {
    detections.push_back(toArmorDetection(armor));
  }
  return detections;
}

}  // namespace fyt::auto_aim
