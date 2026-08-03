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
// Converts traditional light-bar detections into the shared ArmorDetection
// type so the traditional chain reuses the NN path's pose estimation
// (ArmorPoseEstimatorAdapter) and everything downstream of it unchanged.

#ifndef ARMOR_DETECTOR_TRADITIONAL_ARMOR_DETECTION_ADAPTER_HPP_
#define ARMOR_DETECTOR_TRADITIONAL_ARMOR_DETECTION_ADAPTER_HPP_

#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_traditional/types.hpp"

namespace fyt::auto_aim {

ArmorDetection toArmorDetection(const Armor &armor);
std::vector<ArmorDetection> toArmorDetections(const std::vector<Armor> &armors);

}  // namespace fyt::auto_aim
#endif  // ARMOR_DETECTOR_TRADITIONAL_ARMOR_DETECTION_ADAPTER_HPP_
