// ROS-free loader for DetectorConfig from a YAML file. Replaces the node's
// declare_parameter() wall. A default-constructed DetectorConfig is overlaid
// with any keys present under the `detector:` root, so the YAML only needs the
// fields it wants to change. Mirrors armor_detector_nn_node.cpp param mapping.
#ifndef HFUT_DETECTOR_CONFIG_LOADER_HPP
#define HFUT_DETECTOR_CONFIG_LOADER_HPP

#include <string>

#include <yaml-cpp/yaml.h>

#include "armor_detector_nn/core/detector_config.hpp"

namespace hfut::detector {

// Reads `root["detector"]` (or `root` itself if no such key) into a config.
fyt::auto_aim::DetectorConfig loadDetectorConfig(const YAML::Node& root);

// Convenience: load from a YAML file path.
fyt::auto_aim::DetectorConfig loadDetectorConfigFile(const std::string& path);

}  // namespace hfut::detector

#endif  // HFUT_DETECTOR_CONFIG_LOADER_HPP
