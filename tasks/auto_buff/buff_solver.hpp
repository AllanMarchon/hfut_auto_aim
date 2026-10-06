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

  void reset_pose() const;

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
  mutable int pose_rejection_count_{0};
  // 保留符盘平面法向；R 中心向量以云台原点为参考并转换到世界轴，用于连续性判别。
  mutable bool plane_normal_prior_valid_{false};
  mutable Eigen::Vector3d plane_normal_world_prior_{1.0, 0.0, 0.0};
  mutable Eigen::Vector3d r_center_from_gimbal_world_prior_{0.0, 0.0, 0.0};
  double szu_corner_reprojection_max_px_{15.0};
  double szu_r_reprojection_max_px_{50.0};
  double szu_r_reprojection_margin_px_{20.0};
  double szu_pose_max_jump_rad_{30.0 * CV_PI / 180.0};
  double szu_pose_max_translation_jump_m_{0.75};
  int szu_pose_reacquire_after_rejections_{20};
  bool szu_use_r_in_pnp_{false};
  // R 标比符盘旋转中心更靠近相机；这里的正负号沿用符盘模型的 x 轴约定。
  // 当前实测诊断支持负方向，保留为配置项便于以后用同一套程序对照验证。
  double szu_r_object_normal_offset_m_{-0.1};
  mutable int r_offset_diagnostic_counter_{0};

  // 前四点按外端、顺时针侧、内端、逆时针侧排列；坐标原点是符盘旋转中心，700 mm 点是叶片中心。
  const std::vector<cv::Point3f> OBJECT_POINTS = {
    cv::Point3f(0, 0, 827e-3), cv::Point3f(0, 127e-3, 700e-3),
    cv::Point3f(0, 0, 573e-3), cv::Point3f(0, -127e-3, 700e-3),
    cv::Point3f(0, 0, 700e-3), cv::Point3f(0, 0, 220e-3),
    cv::Point3f(0, 0, 0)};  // 单位：米

};
}  // namespace auto_buff
#endif  // AUTO_BUFF__SOLVER_HPP
