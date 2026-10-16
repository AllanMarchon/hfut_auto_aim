#ifndef AUTO_BUFF__BUFF_CONFIG_HPP
#define AUTO_BUFF__BUFF_CONFIG_HPP

#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include "tools/ballistic_solver.hpp"

namespace auto_buff
{
enum class RunePredictionMethod { weighted_least_squares, ekf };

struct BuffConfig
{
  struct Camera
  {
    cv::Mat camera_matrix;
    cv::Mat distort_coeffs;
    Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();
  } camera;

  struct Detector
  {
    std::string type = "njust";
    double fx = 0;
    double fy = 0;
    YAML::Node parameters;  // buff.detector，由所选检测器解析其专属参数。
  } detector;

  struct MotionFit
  {
    double window_seconds = 6.0;
    double min_fit_seconds = 1.5;
    double weight_half_life_seconds = 3.0;
  };

  struct Model
  {
    std::string type = "njust";
    YAML::Node parameters;  // buff.model，由所选模型解析其专属参数。
  } model;

  struct Ballistic
  {
    std::string type = "njust";
    double njust_air_resistance = 0.003;
    double climber_air_resistance_k = 0;
  };

  struct Planner
  {
    std::string mode = "mpc";
    std::string aimer = "njust";
    YAML::Node aimer_parameters;  // buff.aimer，由所选 aimer 解析专属参数。
    double yaw_offset = 0;
    double pitch_offset = 0;
    double bullet_speed_min = 10;
    double bullet_speed_max = 25;
    double bullet_speed_default = 23.4;
    Ballistic ballistic;
  } planner;

  static BuffConfig load(const std::string & path);
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__BUFF_CONFIG_HPP
