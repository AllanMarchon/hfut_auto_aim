#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"
namespace auto_buff
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(std::optional<PowerRune> & ps) const;

  // 调试用
  cv::Point2f point_buff2pixel(cv::Point3f x);

  std::vector<cv::Point2f> reproject_buff(
    const Eigen::Vector3d & xyz_in_world, double yaw, double row) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  mutable cv::Vec3d rvec_, tvec_;
  mutable bool pose_valid_{false};
  double szu_corner_reprojection_max_px_{15.0};
  double szu_r_reprojection_max_px_{50.0};
  bool szu_use_r_in_pnp_{false};

  // 打符叶片四角的菱形几何模型，原点为 R 中心；SZU 映射在求解时按 [1, 0, 3, 4] 恢复。
  const std::vector<cv::Point3f> OBJECT_POINTS = {
    cv::Point3f(0, 0, 827e-3), cv::Point3f(0, 127e-3, 700e-3),
    cv::Point3f(0, 0, 573e-3), cv::Point3f(0, -127e-3, 700e-3),
    cv::Point3f(0, 0, 700e-3), cv::Point3f(0, 0, 220e-3),
    cv::Point3f(0, 0, 0)};  // 单位：米

};
}  // namespace auto_buff
#endif  // AUTO_BUFF__SOLVER_HPP
