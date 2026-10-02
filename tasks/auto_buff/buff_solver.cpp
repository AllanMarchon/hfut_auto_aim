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
  szu_r_reprojection_margin_px_ = yaml["szu_r_reprojection_margin_px"]
                                    ? yaml["szu_r_reprojection_margin_px"].as<double>()
                                    : szu_r_reprojection_margin_px_;
  const double pose_max_jump_deg = yaml["szu_pose_max_jump_deg"]
    ? yaml["szu_pose_max_jump_deg"].as<double>() : szu_pose_max_jump_rad_ * 180.0 / CV_PI;
  szu_pose_max_jump_rad_ = pose_max_jump_deg * CV_PI / 180.0;
  szu_pose_max_translation_jump_m_ = yaml["szu_pose_max_translation_jump_m"]
    ? yaml["szu_pose_max_translation_jump_m"].as<double>()
    : szu_pose_max_translation_jump_m_;
  szu_pose_reacquire_after_rejections_ = yaml["szu_pose_reacquire_after_rejections"]
    ? yaml["szu_pose_reacquire_after_rejections"].as<int>()
    : szu_pose_reacquire_after_rejections_;
  szu_use_r_in_pnp_ = yaml["szu_use_r_in_pnp"]
                        ? yaml["szu_use_r_in_pnp"].as<bool>()
                        : szu_use_r_in_pnp_;
  if (!std::isfinite(szu_corner_reprojection_max_px_) ||
      !std::isfinite(szu_r_reprojection_max_px_) ||
      !std::isfinite(szu_r_reprojection_margin_px_) ||
      !std::isfinite(szu_pose_max_jump_rad_) ||
      !std::isfinite(szu_pose_max_translation_jump_m_) ||
      szu_corner_reprojection_max_px_ <= 0.0 || szu_r_reprojection_max_px_ <= 0.0 ||
      szu_r_reprojection_margin_px_ < 0.0 || szu_pose_max_jump_rad_ <= 0.0 ||
      szu_pose_max_translation_jump_m_ <= 0.0 || szu_pose_reacquire_after_rejections_ <= 0) {
    throw std::invalid_argument("SZU PnP 配置必须是有限值且处于有效范围");
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
    // 先按原始关键点编号恢复一组物理顺序，后面再用 R 点确认方向。
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
    double rotation_delta_rad = 0.0;
    double translation_delta_m = 0.0;
    bool valid = false;
  };

  PnpCandidate best_geometry;
  PnpCandidate best_r_candidate;
  PnpCandidate best_r_unconstrained;
  PnpCandidate best_continuous;
  PnpCandidate best_four_corner;
  constexpr double r_error_tie_px = 3.0;
  const auto prefer_r_candidate = [&](double candidate_r_error,
                                      double candidate_continuity_score,
                                      const PnpCandidate & current) {
    if (!current.valid) return true;
    // R 误差明显更小时优先相信物理对应关系，只有几像素内才用帧间连续性打破平局。
    if (candidate_r_error + r_error_tie_px < current.r_error) return true;
    if (current.r_error + r_error_tie_px < candidate_r_error) return false;
    return candidate_continuity_score < current.continuity_score;
  };
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
    double candidate_rotation_delta_rad = 0.0;
    double candidate_translation_delta_m = 0.0;
    bool candidate_continuity_valid = true;
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
      const cv::Mat relative_rotation = candidate_rotation * previous_rotation.t();
      const double trace = relative_rotation.at<double>(0, 0) +
        relative_rotation.at<double>(1, 1) + relative_rotation.at<double>(2, 2);
      const double rotation_cos = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
      const double rotation_delta_rad = std::acos(rotation_cos);
      continuity_score = rotation_delta_rad + 0.25 * translation_delta_norm;
      candidate_rotation_delta_rad = rotation_delta_rad;
      candidate_translation_delta_m = translation_delta_norm;
      candidate_continuity_valid = rotation_delta_rad <= szu_pose_max_jump_rad_ &&
        translation_delta_norm <= szu_pose_max_translation_jump_m_;
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
      destination.rotation_delta_rad = candidate_rotation_delta_rad;
      destination.translation_delta_m = candidate_translation_delta_m;
      destination.valid = true;
    };

    // 单独保留四角初始解，用来判断 R 点是否把本来正确的四角姿态拉坏。
    if (four_corner_initial &&
        (!best_four_corner.valid || corner_error < best_four_corner.corner_error)) {
      save_candidate(best_four_corner);
    }

    const bool corner_candidate_valid = use_szu_point_indices &&
      corner_error <= szu_corner_reprojection_max_px_;
    if (corner_candidate_valid && prefer_r_candidate(
          r_error, continuity_score, best_r_unconstrained)) {
      save_candidate(best_r_unconstrained);
    }

    // 保留只看四角误差的候选，R 点模型不一致时仍能保持原有四角链路。
    if (!best_geometry.valid || score < best_geometry.score) {
      save_candidate(best_geometry);
    }

    // R 点不参与五点迭代，但用来在八种角点对应关系中选择物理方向。
    // 只记录四角误差和帧间变化合格的候选；R 误差优先，接近时才比较连续性。
    const bool r_candidate_valid = corner_candidate_valid &&
      r_error <= szu_r_reprojection_max_px_ + szu_r_reprojection_margin_px_ &&
      candidate_continuity_valid;
    if (r_candidate_valid && prefer_r_candidate(
          r_error, continuity_score, best_r_candidate)) {
      save_candidate(best_r_candidate);
    }

    // 已有有效姿态时，只在误差合格的候选中优先选择最接近上一帧的姿态。
    // 四点共面存在镜像解，单帧残差不能保证解在帧间连续。
    const bool reprojection_valid =
      !use_szu_point_indices || corner_error <= szu_corner_reprojection_max_px_;
    if (use_szu_point_indices && pose_valid_ && candidate_continuity_valid && reprojection_valid &&
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

  if (use_szu_point_indices) {
    // 先按网络原始编号恢复一组语义顺序，再枚举循环起点和方向。
    // 四个共面点单独拟合时八种对应关系都可能有很小的角点误差，R 点负责确认物理方向。
    for (int reversed = 0; reversed < 2; ++reversed) {
      for (int shift = 0; shift < 4; ++shift) {
        std::vector<cv::Point2f> candidate_points(4);
        for (int i = 0; i < 4; ++i) {
          const int index = reversed ? (shift - i + 4) % 4 : (shift + i) % 4;
          candidate_points[i] = image_points_corners[index];
        }
        try_candidate(candidate_points);
      }
    }
  } else {
    try_candidate(image_points_corners);
  }

  const PnpCandidate * best_ptr = nullptr;
  const bool had_previous_pose = pose_valid_;
  const bool better_r_candidate_rejected_by_continuity =
    use_szu_point_indices && had_previous_pose && best_r_candidate.valid &&
    best_r_unconstrained.valid &&
    best_r_unconstrained.r_error + r_error_tie_px < best_r_candidate.r_error;
  if (use_szu_point_indices && had_previous_pose) {
    if (best_r_candidate.valid && !better_r_candidate_rejected_by_continuity) {
      best_ptr = &best_r_candidate;
    } else if (!better_r_candidate_rejected_by_continuity && best_continuous.valid &&
               best_continuous.r_error <=
                 szu_r_reprojection_max_px_ + szu_r_reprojection_margin_px_) {
      best_ptr = &best_continuous;
    }
  } else if (use_szu_point_indices && best_r_candidate.valid) {
    best_ptr = &best_r_candidate;
  } else if (!use_szu_point_indices && best_geometry.valid) {
    best_ptr = &best_geometry;
  }
  if (best_ptr == nullptr) {
    p.mark_unsolvable();
    if (had_previous_pose) {
      if (better_r_candidate_rejected_by_continuity) {
        tools::logger()->debug(
          "[BuffSolver] 最低 R 误差候选被连续性门限挡住: best_R={:.2f}px "
          "delta={:.1f}deg/{:.3f}m; 连续候选 R={:.2f}px，本帧不输出姿态",
          best_r_unconstrained.r_error,
          best_r_unconstrained.rotation_delta_rad * 180.0 / CV_PI,
          best_r_unconstrained.translation_delta_m, best_r_candidate.r_error);
      }
      // 当前帧的姿态跳变超过限制时保留上一帧姿态，避免错误解污染跟踪器。
      ++pose_rejection_count_;
      if (best_geometry.valid) {
        tools::logger()->debug(
          "[BuffSolver] 拒绝 SZU PnP 候选: R={:.2f}px rotation_delta={:.1f}deg "
          "translation_delta={:.3f}m rejected={}/{}，保留上一帧姿态",
          best_geometry.r_error, best_geometry.rotation_delta_rad * 180.0 / CV_PI,
          best_geometry.translation_delta_m, pose_rejection_count_,
          szu_pose_reacquire_after_rejections_);
      } else {
        tools::logger()->debug(
          "[BuffSolver] 本帧没有有效 SZU PnP 候选 rejected={}/{}，保留上一帧姿态",
          pose_rejection_count_, szu_pose_reacquire_after_rejections_);
      }
      if (pose_rejection_count_ >= szu_pose_reacquire_after_rejections_) {
        pose_valid_ = false;
        pose_rejection_count_ = 0;
        tools::logger()->debug("[BuffSolver] 连续姿态拒绝达到上限，等待 R 合格的新姿态重新初始化");
      }
    } else if (use_szu_point_indices) {
      pose_valid_ = false;
      tools::logger()->debug(
        "[BuffSolver] SZU PnP 初始化候选的 R 误差超过 {:.1f}px，等待有效姿态",
        szu_r_reprojection_max_px_ + szu_r_reprojection_margin_px_);
    } else {
      pose_valid_ = false;
      tools::logger()->debug("[BuffSolver] solvePnP 失败或返回非有限平移");
    }
    return;
  }
  const PnpCandidate & best = *best_ptr;
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
  pose_rejection_count_ = 0;

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
