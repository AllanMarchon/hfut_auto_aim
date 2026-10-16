#include "camera2gimbal_extrinsic.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "math_tools.hpp"

namespace tools
{
Camera2GimbalExtrinsic load_camera2gimbal_extrinsic(const YAML::Node & yaml)
{
  const auto mode = yaml["camera2gimbal_mode"].as<std::string>("matrix");
  if (mode == "matrix") {
    const auto rotation = yaml["R_camera2gimbal"].as<std::vector<double>>();
    const auto translation = yaml["t_camera2gimbal"].as<std::vector<double>>();
    if (rotation.size() != 9 || translation.size() != 3) {
      throw std::invalid_argument("R_camera2gimbal must have 9 elements and t_camera2gimbal 3 elements");
    }
    return {
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(rotation.data()),
      Eigen::Map<const Eigen::Vector3d>(translation.data())};
  }
  if (mode == "xyz_ypr") {
    const auto xyz = yaml["camera2gimbal_xyz"].as<std::vector<double>>();
    const auto ypr = yaml["camera2gimbal_ypr"].as<std::vector<double>>();
    if (xyz.size() != 3 || ypr.size() != 3) {
      throw std::invalid_argument("camera2gimbal_xyz and camera2gimbal_ypr must each have 3 elements");
    }
    Eigen::Matrix3d R_optical2gimbal;
    R_optical2gimbal <<
       0,  0,  1,
      -1,  0,  0,
       0, -1,  0;
    return {
      rotation_matrix(Eigen::Map<const Eigen::Vector3d>(ypr.data())) * R_optical2gimbal,
      Eigen::Map<const Eigen::Vector3d>(xyz.data())};
  }
  throw std::invalid_argument("camera2gimbal_mode must be matrix or xyz_ypr");
}
}  // namespace tools
