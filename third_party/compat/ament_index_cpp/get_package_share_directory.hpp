// ROS-free shim for ament_index_cpp::get_package_share_directory.
//
// The detector resolves `package://armor_detector_nn/<rel>` URLs for its model
// and label_map. With no ament index, we map the package name to a base
// directory provided at compile time via HFUT_DETECTOR_ASSET_DIR (the detector
// source dir, which holds model/ and config/). Unknown packages fall back to
// $HFUT_DETECTOR_ASSET_DIR or the env override HFUT_DETECTOR_ASSET_DIR.
#ifndef HFUT_COMPAT_AMENT_INDEX_CPP_HPP
#define HFUT_COMPAT_AMENT_INDEX_CPP_HPP

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace ament_index_cpp {

inline std::string get_package_share_directory(const std::string& package_name) {
  // Runtime override wins.
  if (const char* env = std::getenv("HFUT_DETECTOR_ASSET_DIR");
      env != nullptr && env[0] != '\0') {
    return std::string(env);
  }
#ifdef HFUT_DETECTOR_ASSET_DIR
#define HFUT_COMPAT_STR2(x) #x
#define HFUT_COMPAT_STR(x) HFUT_COMPAT_STR2(x)
  (void)package_name;
  return std::string(HFUT_COMPAT_STR(HFUT_DETECTOR_ASSET_DIR));
#undef HFUT_COMPAT_STR
#undef HFUT_COMPAT_STR2
#else
  throw std::runtime_error(
      "ament_index_cpp shim: HFUT_DETECTOR_ASSET_DIR not set; cannot resolve "
      "package '" + package_name + "'");
#endif
}

}  // namespace ament_index_cpp

#endif  // HFUT_COMPAT_AMENT_INDEX_CPP_HPP
