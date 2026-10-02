#include "buff_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <stdexcept>

#include "tools/logger.hpp"

namespace auto_buff
{
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  szu_corner_reprojection_max_px_ = yaml["szu_corner_reprojection_max_px"]
                                      ? yaml["szu_corner_reprojection_max_px"].as<double>()
                                      : szu_corner_reprojection_max_px_;
  szu_r_reprojection_max_px_ = yaml["szu_r_reprojection_max_px"]
                                 ? yaml["szu_r_reprojection_max_px"].as<double>()
                                 : szu_r_reprojection_max_px_;
  szu_use_r_in_pnp_ = yaml["szu_use_r_in_pnp"]
                        ? yaml["szu_use_r_in_pnp"].as<bool>()
                        : szu_use_r_in_pnp_;
  if (!std::isfinite(szu_corner_reprojection_max_px_) ||
      !std::isfinite(szu_r_reprojection_max_px_) || szu_corner_reprojection_max_px_ <= 0.0 ||
      szu_r_reprojection_max_px_ <= 0.0) {
    throw std::invalid_argument("SZU PnP 重投影误差阈值必须是有限正数");
  }

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
  if (p.fanblades.empty() || p.target().points.size() < 4) {
    pose_valid_ = false;
    p.mark_unsolvable();
    tools::logger()->debug("[BuffSolver] PnP 输入角点不足: {}", p.fanblades.empty() ? 0 : p.target().points.size());
    return;
  }

  // 只取四个符叶角点，避免兼容 SP25 六点检测结果时把额外点传入四点模型。
  const bool use_szu_point_indices = p.target().point_indices.size() >= 4;
  std::vector<cv::Point2f> image_points_corners;
  image_points_corners.reserve(4);
  if (use_szu_point_indices) {
    // README 标注的环向顺序为 1(上)、0(右)、3(下)、4(左)。
    // 直接按原始关键点编号恢复物理顺序，不能按像素极角或八种排列猜测。
    constexpr std::array<int, 4> szu_keypoint_order{1, 0, 3, 4};
    for (const int expected_index : szu_keypoint_order) {
      const auto it = std::find(
        p.target().point_indices.begin(), p.target().point_indices.end(), expected_index);
      if (it == p.target().point_indices.end()) {
        pose_valid_ = false;
        p.mark_unsolvable();
        tools::logger()->debug("[BuffSolver] SZU 角点缺少原始编号: {}", expected_index);
        return;
      }
      const size_t point_index = static_cast<size_t>(
        std::distance(p.target().point_indices.begin(), it));
      if (point_index >= p.target().points.size()) {
        pose_valid_ = false;
        p.mark_unsolvable();
        tools::logger()->debug("[BuffSolver] SZU 角点编号与坐标数量不一致");
        return;
      }
      image_points_corners.emplace_back(p.target().points[point_index]);
    }
  } else {
    image_points_corners.assign(p.target().points.begin(), p.target().points.begin() + 4);
  }
  const std::vector<cv::Point3f> object_points_corners(
    OBJECT_POINTS.begin(), OBJECT_POINTS.begin() + 4);
  // R 标中心是模型坐标原点，700 mm 点是待击打叶片中心。
  const std::vector<cv::Point3f> r_object_point{OBJECT_POINTS.back()};
  std::vector<cv::Point3f> szu_object_points = object_points_corners;
  szu_object_points.emplace_back(OBJECT_POINTS.back());

  struct PnpCandidate
  {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::vector<cv::Point2f> image_points;
    double corner_error = 0.0;
    double r_error = 0.0;
    double score = 0.0;
    double continuity_score = 0.0;
    bool valid = false;
  };

  PnpCandidate best_geometry;
  PnpCandidate best_continuous;
  PnpCandidate best_four_corner;
  const auto evaluate_pose = [&](const std::vector<cv::Point2f> & candidate_points,
                                 const cv::Vec3d & candidate_rvec,
                                 const cv::Vec3d & candidate_tvec,
                                 bool four_corner_initial) {
    if (!std::isfinite(candidate_tvec[0]) || !std::isfinite(candidate_tvec[1]) ||
        !std::isfinite(candidate_tvec[2]) || candidate_tvec[2] <= 0.0) {
      return;
    }

    std::vector<cv::Point2f> projected_corners;
    cv::projectPoints(
      object_points_corners, candidate_rvec, candidate_tvec, camera_matrix_, distort_coeffs_,
      projected_corners);
    if (projected_corners.size() != candidate_points.size()) return;

    double corner_squared_error = 0.0;
    for (size_t i = 0; i < projected_corners.size(); ++i) {
      const double error = cv::norm(projected_corners[i] - candidate_points[i]);
      corner_squared_error += error * error;
    }
    const double corner_error = std::sqrt(corner_squared_error / 4.0);

    std::vector<cv::Point2f> projected_r;
    cv::projectPoints(
      r_object_point, candidate_rvec, candidate_tvec, camera_matrix_, distort_coeffs_,
      projected_r);
    if (projected_r.empty()) return;
    const double r_error = cv::norm(projected_r.front() - p.r_center);
    if (!std::isfinite(corner_error) || !std::isfinite(r_error)) return;

    double continuity_score = 0.0;
    if (use_szu_point_indices && pose_valid_) {
      cv::Mat candidate_rotation;
      cv::Mat previous_rotation;
      cv::Rodrigues(candidate_rvec, candidate_rotation);
      cv::Rodrigues(rvec_, previous_rotation);
      const cv::Vec3d translation_delta{
        candidate_tvec[0] - tvec_[0], candidate_tvec[1] - tvec_[1],
        candidate_tvec[2] - tvec_[2]};
      const double translation_delta_norm = std::sqrt(
        translation_delta[0] * translation_delta[0] + translation_delta[1] * translation_delta[1] +
        translation_delta[2] * translation_delta[2]);
      continuity_score = cv::norm(candidate_rotation - previous_rotation) +
                         0.25 * translation_delta_norm;
    }

    const bool use_r_constraint = use_szu_point_indices && szu_use_r_in_pnp_;
    const double score = use_r_constraint ? r_error + 0.25 * corner_error : corner_error;
    const auto save_candidate = [&](PnpCandidate & destination) {
      destination.rvec = candidate_rvec;
      destination.tvec = candidate_tvec;
      destination.image_points = candidate_points;
      destination.corner_error = corner_error;
      destination.r_error = r_error;
      destination.score = score;
      destination.continuity_score = continuity_score;
      destination.valid = true;
    };

    // 单独保留四角初始解，用来判断 R 点是否把本来正确的四角姿态拉坏。
    if (four_corner_initial &&
        (!best_four_corner.valid || corner_error < best_four_corner.corner_error)) {
      save_candidate(best_four_corner);
    }

    // 保留残差最小的候选，首帧或连续性失效时使用它恢复跟踪。
    if (!best_geometry.valid || score < best_geometry.score) {
      save_candidate(best_geometry);
    }

    // 已有有效姿态时，只在误差合格的候选中优先选择最接近上一帧的姿态。
    // 四点共面存在镜像解，单帧残差不能保证解在帧间连续。
    const bool reprojection_valid =
      !use_szu_point_indices ||
      (corner_error <= szu_corner_reprojection_max_px_ &&
       (!szu_use_r_in_pnp_ || r_error <= szu_r_reprojection_max_px_));
    if (use_szu_point_indices && pose_valid_ && reprojection_valid &&
        (!best_continuous.valid || continuity_score < best_continuous.continuity_score)) {
      save_candidate(best_continuous);
    }
  };

  const auto try_candidate = [&](const std::vector<cv::Point2f> & candidate_points) {
    if (!use_szu_point_indices) {
      cv::Vec3d candidate_rvec;
      cv::Vec3d candidate_tvec;
      const bool solved = cv::solvePnP(
        object_points_corners, candidate_points, camera_matrix_, distort_coeffs_, candidate_rvec,
        candidate_tvec, false, cv::SOLVEPNP_IPPE);
      if (solved) evaluate_pose(candidate_points, candidate_rvec, candidate_tvec, true);
      return;
    }

    // 先用四角 IPPE 得到平面姿态；R 点只有显式开启时才参与迭代细化。
    std::vector<cv::Mat> candidate_rvecs;
    std::vector<cv::Mat> candidate_tvecs;
    const int solution_count = cv::solvePnPGeneric(
      object_points_corners, candidate_points, camera_matrix_, distort_coeffs_, candidate_rvecs,
      candidate_tvecs, false, cv::SOLVEPNP_IPPE);
    const size_t count = std::min(
      static_cast<size_t>(std::max(solution_count, 0)),
      std::min(candidate_rvecs.size(), candidate_tvecs.size()));
    for (size_t i = 0; i < count; ++i) {
      cv::Mat rvec_mat;
      cv::Mat tvec_mat;
      candidate_rvecs[i].convertTo(rvec_mat, CV_64F);
      candidate_tvecs[i].convertTo(tvec_mat, CV_64F);
      if (rvec_mat.total() != 3 || tvec_mat.total() != 3) continue;
      rvec_mat = rvec_mat.reshape(1, 3);
      tvec_mat = tvec_mat.reshape(1, 3);
      const cv::Vec3d initial_rvec{
        rvec_mat.at<double>(0, 0), rvec_mat.at<double>(1, 0), rvec_mat.at<double>(2, 0)};
      const cv::Vec3d initial_tvec{
        tvec_mat.at<double>(0, 0), tvec_mat.at<double>(1, 0),
        tvec_mat.at<double>(2, 0)};

      evaluate_pose(candidate_points, initial_rvec, initial_tvec, true);
      if (szu_use_r_in_pnp_) {
        std::vector<cv::Point2f> image_points = candidate_points;
        image_points.emplace_back(p.r_center);
        cv::Vec3d refined_rvec = initial_rvec;
        cv::Vec3d refined_tvec = initial_tvec;
        const bool refined = cv::solvePnP(
          szu_object_points, image_points, camera_matrix_, distort_coeffs_, refined_rvec,
          refined_tvec, true, cv::SOLVEPNP_ITERATIVE);
        if (refined) evaluate_pose(candidate_points, refined_rvec, refined_tvec, false);
      }
    }
  };

  try_candidate(image_points_corners);

  const PnpCandidate & best = best_continuous.valid ? best_continuous : best_geometry;
  if (!best.valid) {
    pose_valid_ = false;
    p.mark_unsolvable();
    tools::logger()->debug("[BuffSolver] solvePnP 失败或返回非有限平移");
    return;
  }
  if (use_szu_point_indices &&
      (best.corner_error > szu_corner_reprojection_max_px_ ||
       (szu_use_r_in_pnp_ && best.r_error > szu_r_reprojection_max_px_))) {
    pose_valid_ = false;
    p.mark_unsolvable();
    tools::logger()->debug(
      "[BuffSolver] SZU 五点 PnP 重投影误差偏大: corners={:.2f}px R={:.2f}px",
      best.corner_error, best.r_error);
    if (best_four_corner.valid) {
      tools::logger()->debug(
        "[BuffSolver] 四角初始解诊断: corners={:.2f}px R={:.2f}px",
        best_four_corner.corner_error, best_four_corner.r_error);
    }
    return;
  }

  rvec_ = best.rvec;
  tvec_ = best.tvec;
  pose_valid_ = true;

  // SZU 默认由四角求姿态，R 点保留为一致性诊断；显式开启时才使用五点约束。
  std::vector<cv::Point2f> projected_points;
  cv::projectPoints(
    object_points_corners, rvec_, tvec_, camera_matrix_, distort_coeffs_, projected_points);
  double squared_error = 0.0;
  for (size_t i = 0; i < projected_points.size(); ++i) {
    const double error = cv::norm(projected_points[i] - best.image_points[i]);
    squared_error += error * error;
  }
  p.pnp_reprojection_error_px =
    std::sqrt(squared_error / static_cast<double>(projected_points.size()));
  // R 点是能量机关旋转中心，对应模型坐标原点；700 mm 点是待击打叶片中心。
  std::vector<cv::Point2f> r_projected_point;
  cv::projectPoints(
    r_object_point, rvec_, tvec_, camera_matrix_, distort_coeffs_, r_projected_point);
  p.pnp_r_reprojection_error_px =
    r_projected_point.empty() ? 0.0 : cv::norm(r_projected_point.front() - p.r_center);
  if (!r_projected_point.empty()) p.pnp_r_projected_pixel = r_projected_point.front();
  if (use_szu_point_indices && p.pnp_r_reprojection_error_px > szu_r_reprojection_max_px_) {
    tools::logger()->debug(
      "[BuffSolver] 四角 PnP 已通过，但 R 点不一致: corners={:.2f}px R={:.2f}px",
      p.pnp_reprojection_error_px, p.pnp_r_reprojection_error_px);
  }
  p.pnp_center_distance_m = std::sqrt(
    tvec_[0] * tvec_[0] + tvec_[1] * tvec_[1] + tvec_[2] * tvec_[2]);
  if (!std::isfinite(p.pnp_reprojection_error_px) ||
      !std::isfinite(p.pnp_r_reprojection_error_px)) {
    tools::logger()->debug(
      "[BuffSolver] PnP 重投影误差非有限: corners={:.2f}px R={:.2f}px origin={:.3f}m",
      p.pnp_reprojection_error_px, p.pnp_r_reprojection_error_px, p.pnp_center_distance_m);
    p.mark_unsolvable();
    pose_valid_ = false;
    return;
  }

  Eigen::Vector3d t_buff2camera;
  cv::cv2eigen(tvec_, t_buff2camera);
  cv::Mat rmat;
  cv::Rodrigues(rvec_, rmat);
  Eigen::Matrix3d R_buff2camera;
  cv::cv2eigen(rmat, R_buff2camera);

  Eigen::Vector3d blade_xyz_in_buff{{0, 0, 700e-3}};

  // 打符坐标系到相机坐标系
  Eigen::Vector3d xyz_in_camera = t_buff2camera;
  Eigen::Vector3d blade_xyz_in_camera = R_buff2camera * blade_xyz_in_buff + t_buff2camera;

  // 相机坐标系到云台坐标系
  Eigen::Matrix3d R_buff2gimbal = R_camera2gimbal_ * R_buff2camera;
  Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  Eigen::Vector3d blade_xyz_in_gimbal = R_camera2gimbal_ * blade_xyz_in_camera + t_camera2gimbal_;

  /// 云台坐标系到世界坐标系
  Eigen::Matrix3d R_buff2world = R_gimbal2world_ * R_buff2gimbal;

  p.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
  p.ypd_in_world = tools::xyz2ypd(p.xyz_in_world);

  p.blade_xyz_in_world = R_gimbal2world_ * blade_xyz_in_gimbal;
  p.blade_ypd_in_world = tools::xyz2ypd(p.blade_xyz_in_world);
  // 记录与 Aimer 使用的同一类水平距离，便于区分 PnP 和跟踪器的误差。
  p.pnp_blade_horizontal_distance_m =
    std::hypot(p.blade_xyz_in_world[0], p.blade_xyz_in_world[1]);

  p.ypr_in_world = tools::eulers(R_buff2world, 2, 1, 0);
  if (!p.xyz_in_world.allFinite() || !p.ypd_in_world.allFinite() ||
      !p.blade_xyz_in_world.allFinite() || !p.blade_ypd_in_world.allFinite() ||
      !p.ypr_in_world.allFinite()) {
    tools::logger()->debug("[BuffSolver] PnP 坐标或姿态出现非有限值");
    p.mark_unsolvable();
    pose_valid_ = false;
  }
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

// 世界坐标转换为像素坐标
std::vector<cv::Point2f> Solver::reproject_buff(
  const Eigen::Vector3d & xyz_in_world, double yaw, double row) const
{
  auto R_buff2world = tools::rotation_matrix(Eigen::Vector3d(yaw, 0.0, row));
  // clang-format on

  // 计算打符到相机的旋转和平移
  const Eigen::Vector3d & t_buff2world = xyz_in_world;
  Eigen::Matrix3d R_buff2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_buff2world;
  Eigen::Vector3d t_buff2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_buff2world - t_camera2gimbal_);

  // 转换为 OpenCV 的旋转向量和平移向量
  cv::Vec3d rvec;
  cv::Mat R_buff2camera_cv;
  cv::eigen2cv(R_buff2camera, R_buff2camera_cv);
  cv::Rodrigues(R_buff2camera_cv, rvec);
  cv::Vec3d tvec(t_buff2camera[0], t_buff2camera[1], t_buff2camera[2]);

  // 重新投影
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(OBJECT_POINTS, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}
}  // namespace auto_buff
