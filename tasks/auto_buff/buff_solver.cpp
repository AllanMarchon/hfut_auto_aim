#include "buff_solver.hpp"

#include <cmath>

#include "tools/logger.hpp"

namespace auto_buff
{
cv::Matx33f Solver::rotation_matrix(double angle) const
{
  return cv::Matx33f(
    1, 0, 0, 0, std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle));
}

void Solver::compute_rotated_points(std::vector<std::vector<cv::Point3f>> & object_points)
{
  const std::vector<cv::Point3f> & base_points = object_points[0];
  for (int i = 1; i < 5; ++i) {
    double angle = i * THETA;
    cv::Matx33f R = rotation_matrix(angle);
    std::vector<cv::Point3f> rotated_points;
    for (const auto & point : base_points) {
      cv::Vec3f vec(point.x, point.y, point.z);
      cv::Vec3f rotated_vec = R * vec;
      rotated_points.emplace_back(rotated_vec[0], rotated_vec[1], rotated_vec[2]);
    }
    object_points[i] = rotated_points;
  }
}

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);

  // compute_rotated_points(OBJECT_POINTS);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

void Solver::solve(std::optional<PowerRune> & ps) const
{
  if (!ps.has_value()) return;
  PowerRune & p = ps.value();
  // std::vector<cv::Point2f> image_points;
  // std::vector<cv::Point3f> object_points;
  // int i = 0;
  // for (auto & fanblade : p.fanblades) {
  //   if (fanblade.type != _unlight) {
  //     image_points.insert(image_points.end(), fanblade.points.begin(), fanblade.points.end());
  //     image_points.emplace_back(fanblade.center);
  //     object_points.insert(object_points.end(), OBJECT_POINTS[i].begin(), OBJECT_POINTS[i].end());
  //   }
  //   ++i;
  // }
  // image_points.emplace_back(p.r_center);  //r_center
  // object_points.emplace_back(cv::Point3f(0, 0, 0));
  if (p.fanblades.empty() || p.target().points.size() < 4) {
    p.mark_unsolvable();
    tools::logger()->debug("[BuffSolver] PnP 输入角点不足: {}", p.fanblades.empty() ? 0 : p.target().points.size());
    return;
  }

  std::vector<cv::Point2f> image_points = p.target().points;
  // image_points.emplace_back(p.target().center);
  image_points.emplace_back(p.r_center);

  std::vector<cv::Point2f> image_points_fourth(image_points.begin(), image_points.begin() + 4);
  std::vector<cv::Point3f> OBJECT_POINTS_FOURTH(OBJECT_POINTS.begin(), OBJECT_POINTS.begin() + 4);
  const bool solved = cv::solvePnP(
    OBJECT_POINTS_FOURTH, image_points_fourth, camera_matrix_, distort_coeffs_, rvec_, tvec_, false,
    cv::SOLVEPNP_IPPE);
  if (!solved || !std::isfinite(tvec_[0]) || !std::isfinite(tvec_[1]) ||
      !std::isfinite(tvec_[2])) {
    p.mark_unsolvable();
    tools::logger()->debug("[BuffSolver] solvePnP 失败或返回非有限平移");
    return;
  }

  // 四个角点共面，因此继续使用 IPPE。R 点暂不参与姿态选择，
  // 这里先保留四角重投影误差，用于独立检查关键点编号和模型几何。
  std::vector<cv::Point2f> projected_points;
  cv::projectPoints(
    OBJECT_POINTS_FOURTH, rvec_, tvec_, camera_matrix_, distort_coeffs_, projected_points);
  double squared_error = 0.0;
  for (size_t i = 0; i < projected_points.size(); ++i) {
    squared_error += cv::norm(projected_points[i] - image_points_fourth[i]) *
                     cv::norm(projected_points[i] - image_points_fourth[i]);
  }
  p.pnp_reprojection_error_px =
    std::sqrt(squared_error / static_cast<double>(projected_points.size()));
  // R 点是能量机关旋转中心，对应模型原点，不是距离原点 700 mm 的待击打叶片中心。
  std::vector<cv::Point3f> r_object_point{OBJECT_POINTS.back()};
  std::vector<cv::Point2f> r_projected_point;
  cv::projectPoints(
    r_object_point, rvec_, tvec_, camera_matrix_, distort_coeffs_, r_projected_point);
  p.pnp_r_reprojection_error_px =
    r_projected_point.empty() ? 0.0 : cv::norm(r_projected_point.front() - p.r_center);
  p.pnp_center_distance_m = std::sqrt(
    tvec_[0] * tvec_[0] + tvec_[1] * tvec_[1] + tvec_[2] * tvec_[2]);
  if (!std::isfinite(p.pnp_reprojection_error_px) ||
      !std::isfinite(p.pnp_r_reprojection_error_px) || p.pnp_reprojection_error_px > 8.0 ||
      p.pnp_r_reprojection_error_px > 8.0) {
    tools::logger()->debug(
      "[BuffSolver] PnP 重投影误差偏大: corners={:.2f}px R={:.2f}px origin={:.3f}m",
      p.pnp_reprojection_error_px, p.pnp_r_reprojection_error_px, p.pnp_center_distance_m);
  }

  Eigen::Vector3d t_buff2camera;
  cv::cv2eigen(tvec_, t_buff2camera);
  cv::Mat rmat;
  cv::Rodrigues(rvec_, rmat);
  Eigen::Matrix3d R_buff2camera;
  cv::cv2eigen(rmat, R_buff2camera);

  Eigen::Vector3d blade_xyz_in_buff{{0, 0, 700e-3}};

  // buff -> camera
  Eigen::Vector3d xyz_in_camera = t_buff2camera;
  Eigen::Vector3d blade_xyz_in_camera = R_buff2camera * blade_xyz_in_buff + t_buff2camera;

  // camera -> gimbal
  Eigen::Matrix3d R_buff2gimbal = R_camera2gimbal_ * R_buff2camera;
  Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  Eigen::Vector3d blade_xyz_in_gimbal = R_camera2gimbal_ * blade_xyz_in_camera + t_camera2gimbal_;

  /// gimbal -> world
  Eigen::Matrix3d R_buff2world = R_gimbal2world_ * R_buff2gimbal;

  p.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
  p.ypd_in_world = tools::xyz2ypd(p.xyz_in_world);

  p.blade_xyz_in_world = R_gimbal2world_ * blade_xyz_in_gimbal;
  p.blade_ypd_in_world = tools::xyz2ypd(p.blade_xyz_in_world);
  // 记录与 Aimer 使用的同一类水平距离，便于区分 PnP 和跟踪器的误差。
  p.pnp_blade_horizontal_distance_m =
    std::hypot(p.blade_xyz_in_world[0], p.blade_xyz_in_world[1]);

  p.ypr_in_world = tools::eulers(R_buff2world, 2, 1, 0);
}

// 调试用
cv::Point2f Solver::point_buff2pixel(cv::Point3f x)
{
  // buff坐标系(单位:m)到像素坐标系
  std::vector<cv::Point3d> world_points;
  std::vector<cv::Point2d> image_points;
  world_points.push_back(x);
  cv::projectPoints(world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  return image_points.back();
}

// xyz_in_world2xyz_in_pix
std::vector<cv::Point2f> Solver::reproject_buff(
  const Eigen::Vector3d & xyz_in_world, double yaw, double row) const
{
  auto R_buff2world = tools::rotation_matrix(Eigen::Vector3d(yaw, 0.0, row));
  // clang-format on

  // get R_buff2camera t_buff2camera
  const Eigen::Vector3d & t_buff2world = xyz_in_world;
  Eigen::Matrix3d R_buff2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_buff2world;
  Eigen::Vector3d t_buff2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_buff2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_buff2camera_cv;
  cv::eigen2cv(R_buff2camera, R_buff2camera_cv);
  cv::Rodrigues(R_buff2camera_cv, rvec);
  cv::Vec3d tvec(t_buff2camera[0], t_buff2camera[1], t_buff2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(OBJECT_POINTS, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}
}  // namespace auto_buff
