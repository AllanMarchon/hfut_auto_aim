#include "buff_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

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

void Solver::reset_pose() const
{
  // 保留符盘法向先验，短暂丢帧后仍可选择正确的平面姿态分支。
  pose_valid_ = false;
  pose_rejection_count_ = 0;
}

void Solver::solve(std::optional<PowerRune> & ps) const
{
  if (!ps.has_value()) return;
  PowerRune & p = ps.value();
  // 当前观测无效时保留上一帧有效姿态；由外层连续丢帧计数决定何时真正重置。
  const bool had_previous_pose = pose_valid_;
  p.pnp_r_projection_valid = false;
  if (p.fanblades.empty() || p.target().points.size() < 4) {
    p.mark_unsolvable();
    tools::logger()->debug("[BuffSolver] PnP 输入角点不足: {}", p.fanblades.empty() ? 0 : p.target().points.size());
    return;
  }

  // 只取四个符叶角点，避免兼容 SP25 六点检测结果时把额外点传入四点模型。
  const bool use_szu_point_indices = p.target().point_indices.size() >= 4;
  std::vector<cv::Point2f> image_points_corners;
  image_points_corners.reserve(4);
  if (use_szu_point_indices) {
    // 模型坐标按顺时针排列：0 是远端，3 是顺时针侧点，4 是近端，1 是逆时针侧点。
    constexpr std::array<int, 4> szu_keypoint_order{0, 3, 4, 1};
    for (const int expected_index : szu_keypoint_order) {
      const auto it = std::find(
        p.target().point_indices.begin(), p.target().point_indices.end(), expected_index);
      if (it == p.target().point_indices.end()) {
        p.mark_unsolvable();
        tools::logger()->debug("[BuffSolver] SZU 角点缺少原始编号: {}", expected_index);
        return;
      }
      const size_t point_index = static_cast<size_t>(
        std::distance(p.target().point_indices.begin(), it));
      if (point_index >= p.target().points.size()) {
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
  // PnP 原点是符盘旋转中心；网络 R 点对应盘面前方的可见 R 标，二者相差 100 mm。
  const std::vector<cv::Point3f> r_object_point{VISIBLE_R_OBJECT_POINT};
  // 仅用于诊断 R 点法向符号，不能参与最终姿态选择。
  const std::vector<cv::Point3f> opposite_r_object_point{{0.1F, 0.0F, 0.0F}};
  std::vector<cv::Point3f> szu_object_points = object_points_corners;
  // 五点约束必须使用可见 R 标的三维位置，否则会把旋转中心误当成 R 标。
  szu_object_points.emplace_back(VISIBLE_R_OBJECT_POINT);

  struct PnpCandidate
  {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::Point2f projected_r{0.0F, 0.0F};
    std::vector<cv::Point2f> image_points;
    double corner_error = 0.0;
    double r_error = 0.0;
    double score = 0.0;
    double continuity_score = 0.0;
    double plane_normal_delta_rad = 0.0;
    double r_center_delta_m = 0.0;
    Eigen::Vector3d plane_normal_world{1.0, 0.0, 0.0};
    Eigen::Vector3d r_center_from_gimbal_world{0.0, 0.0, 0.0};
    bool valid = false;
  };

  PnpCandidate best_geometry;
  PnpCandidate best_r_candidate;
  PnpCandidate best_r_unconstrained;
  PnpCandidate best_continuous;
  PnpCandidate best_four_corner;
  PnpCandidate best_five_point;
  // 保存两个四角 IPPE 候选，诊断 R 点偏移时需要分别比较它们。
  std::vector<PnpCandidate> four_corner_candidates;
  four_corner_candidates.reserve(2);
  constexpr double r_error_tie_px = 3.0;
  // 记录两个四角 IPPE 候选的 R 误差，用相对差异判断镜像分支，而不是要求 R 绝对误差必须接近零。
  double r_branch_best_error_px = std::numeric_limits<double>::infinity();
  double r_branch_second_error_px = std::numeric_limits<double>::infinity();
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
    double candidate_plane_normal_delta_rad = 0.0;
    double candidate_r_center_delta_m = 0.0;
    Eigen::Vector3d candidate_plane_normal_world{1.0, 0.0, 0.0};
    Eigen::Vector3d candidate_r_center_from_gimbal_world{0.0, 0.0, 0.0};
    bool candidate_continuity_valid = true;
    if (use_szu_point_indices) {
      cv::Mat candidate_rotation;
      cv::Rodrigues(candidate_rvec, candidate_rotation);
      Eigen::Matrix3d buff2camera;
      cv::cv2eigen(candidate_rotation, buff2camera);
      const Eigen::Matrix3d buff2world = R_gimbal2world_ * R_camera2gimbal_ * buff2camera;
      const Eigen::Vector3d camera_translation{
        candidate_tvec[0], candidate_tvec[1], candidate_tvec[2]};
      candidate_r_center_from_gimbal_world =
        R_gimbal2world_ * (R_camera2gimbal_ * camera_translation + t_camera2gimbal_);
      candidate_plane_normal_world = buff2world.col(0).normalized();

      if (plane_normal_prior_valid_) {
        // 随机换扇叶只改变盘面内 roll，盘面法向的方向和 R 中心都应保持连续。
        // 不能用法向绝对值比较，否则共面 IPPE 的反向镜像解也会被当成同一姿态。
        const double normal_cos = std::clamp(
          candidate_plane_normal_world.dot(plane_normal_world_prior_), -1.0, 1.0);
        const double plane_normal_delta_rad = std::acos(normal_cos);
        candidate_plane_normal_delta_rad = plane_normal_delta_rad;
        continuity_score = plane_normal_delta_rad;
        // 先验存在时始终用法向和 R 中心给候选排序；只有当前姿态仍有效时才硬拒绝跳变。
        const double r_center_delta_norm =
          (candidate_r_center_from_gimbal_world - r_center_from_gimbal_world_prior_).norm();
        candidate_r_center_delta_m = r_center_delta_norm;
        continuity_score += 0.25 * r_center_delta_norm;
        if (pose_valid_) {
          candidate_continuity_valid = plane_normal_delta_rad <= szu_pose_max_jump_rad_ &&
            r_center_delta_norm <= szu_pose_max_translation_jump_m_;
        }
      } else {
        // 世界 Z 轴为竖直方向；符盘实体竖直，初次解算优先选择水平法向候选。
        continuity_score = std::abs(candidate_plane_normal_world.z());
      }
    }

    const bool use_r_constraint = use_szu_point_indices && szu_use_r_in_pnp_;
    const double score = use_r_constraint ? r_error + 0.25 * corner_error : corner_error;
    const auto save_candidate = [&](PnpCandidate & destination) {
      destination.rvec = candidate_rvec;
      destination.tvec = candidate_tvec;
      destination.projected_r = projected_r.front();
      destination.image_points = candidate_points;
      destination.corner_error = corner_error;
      destination.r_error = r_error;
      destination.score = score;
      destination.continuity_score = continuity_score;
      destination.plane_normal_delta_rad = candidate_plane_normal_delta_rad;
      destination.r_center_delta_m = candidate_r_center_delta_m;
      destination.plane_normal_world = candidate_plane_normal_world;
      destination.r_center_from_gimbal_world = candidate_r_center_from_gimbal_world;
      destination.valid = true;
    };

    if (four_corner_initial && use_szu_point_indices && !szu_use_r_in_pnp_) {
      PnpCandidate snapshot;
      save_candidate(snapshot);
      four_corner_candidates.emplace_back(std::move(snapshot));
    }

    if (!four_corner_initial && use_r_constraint &&
        (!best_five_point.valid || score < best_five_point.score)) {
      save_candidate(best_five_point);
    }

    // 单独保留四角初始解，用来判断 R 点是否把本来正确的四角姿态拉坏。
    if (four_corner_initial &&
        (!best_four_corner.valid || corner_error < best_four_corner.corner_error)) {
      save_candidate(best_four_corner);
    }

    const bool corner_candidate_valid = use_szu_point_indices &&
      corner_error <= szu_corner_reprojection_max_px_;
    if (corner_candidate_valid) {
      if (r_error < r_branch_best_error_px) {
        r_branch_second_error_px = r_branch_best_error_px;
        r_branch_best_error_px = r_error;
      } else if (r_error < r_branch_second_error_px) {
        r_branch_second_error_px = r_error;
      }
    }
    if (corner_candidate_valid && prefer_r_candidate(
          r_error, continuity_score, best_r_unconstrained)) {
      save_candidate(best_r_unconstrained);
    }

    // 单独保留四角误差最小的候选，R 点模型不一致时仍能保持原有四角链路。
    if (!best_geometry.valid || corner_error < best_geometry.corner_error) {
      save_candidate(best_geometry);
    }

    // 只有显式开启 R 约束时，才用 R 误差筛选 IPPE 候选；关闭时 R 仅保留为诊断。
    // 开启 R 约束时要求四角、R 和帧间连续性同时合格，R 误差优先，接近时才比较连续性。
    const bool r_candidate_valid = corner_candidate_valid &&
      r_error <= szu_r_reprojection_max_px_ + szu_r_reprojection_margin_px_ &&
      candidate_continuity_valid;
    if (use_r_constraint && r_candidate_valid && prefer_r_candidate(
          r_error, continuity_score, best_r_candidate)) {
      save_candidate(best_r_candidate);
    }

    // 已有有效姿态时，只在误差合格的候选中优先选择最接近上一帧的姿态。
    // 四点共面存在镜像解，单帧残差不能保证解在帧间连续。
    const bool reprojection_valid =
      !use_szu_point_indices || corner_error <= szu_corner_reprojection_max_px_;
    if (use_szu_point_indices && (pose_valid_ || plane_normal_prior_valid_) &&
        candidate_continuity_valid && reprojection_valid &&
        (!best_continuous.valid || continuity_score < best_continuous.continuity_score)) {
      save_candidate(best_continuous);
    }
  };

  const auto try_candidate = [&](const std::vector<cv::Point2f> & candidate_points) {
    // SP25 和显式开启 R 约束时沿用原有单个 IPPE 解；SZU 默认四角链路无论首次初始化、
    // 短暂丢帧重获还是连续跟踪，都枚举两个共面候选，避免首个镜像分支污染距离和姿态。
    if (!use_szu_point_indices || szu_use_r_in_pnp_) {
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

  // SZU 模型的角点通道有固定语义，已按外端、顺时针侧、内端、逆时针侧恢复。
  try_candidate(image_points_corners);

  const auto log_r_source_diagnostics = [&](const char * stage, const cv::Point2f & pnp_r,
                                             double corner_error) {
    const auto source_error = [&](const RPointSourceSummary & source) {
      return source.count > 0 ? cv::norm(pnp_r - source.center) : -1.0;
    };
    const auto source_delta = [](const RPointSourceSummary & a, const RPointSourceSummary & b) {
      return a.count > 0 && b.count > 0 ? cv::norm(a.center - b.center) : -1.0;
    };
    const auto source_x = [](const RPointSourceSummary & source) {
      return source.count > 0 ? source.center.x : -1.0F;
    };
    const auto source_y = [](const RPointSourceSummary & source) {
      return source.count > 0 ? source.center.y : -1.0F;
    };
    const auto & r_diagnostics = p.r_point_diagnostics;
    tools::logger()->debug(
      "[BuffSolver] {}: corners={:.2f}px "
      "R(final/net/geo/visual)={:.2f}/{:.2f}/{:.2f}/{:.2f}px "
      "src_delta(net-geo/net-vis/geo-vis)={:.1f}/{:.1f}/{:.1f}px "
      "pixels pnp=({:.1f},{:.1f}) net=({:.1f},{:.1f}) geo=({:.1f},{:.1f}) "
      "visual=({:.1f},{:.1f}) final=({:.1f},{:.1f}) "
      "spread(final/net/geo/vis)={:.1f}/{:.1f}/{:.1f}/{:.1f}px "
      "observations={} count(net/geo/vis)={}/{}/{} net_conf={:.3f}",
      stage, corner_error, cv::norm(pnp_r - p.r_center), source_error(r_diagnostics.network),
      source_error(r_diagnostics.geometry), source_error(r_diagnostics.visual),
      source_delta(r_diagnostics.network, r_diagnostics.geometry),
      source_delta(r_diagnostics.network, r_diagnostics.visual),
      source_delta(r_diagnostics.geometry, r_diagnostics.visual), pnp_r.x, pnp_r.y,
      source_x(r_diagnostics.network), source_y(r_diagnostics.network),
      source_x(r_diagnostics.geometry), source_y(r_diagnostics.geometry),
      source_x(r_diagnostics.visual), source_y(r_diagnostics.visual), p.r_center.x, p.r_center.y,
      p.r_center_spread_px,
      r_diagnostics.network.spread_px, r_diagnostics.geometry.spread_px,
      r_diagnostics.visual.spread_px, r_diagnostics.detection_count,
      r_diagnostics.network.count, r_diagnostics.geometry.count, r_diagnostics.visual.count,
      r_diagnostics.network_confidence);
  };

  const PnpCandidate * best_ptr = nullptr;
  const auto & r_diagnostics = p.r_point_diagnostics;
  // 网络和视觉 R 相互印证时，才允许它们共同覆盖旧姿态的连续性判断。
  constexpr double r_source_consensus_max_px = 12.0;
  constexpr double strong_r_reprojection_max_px = 8.0;
  // 可见 R 的模型位置与网络像素中心可能存在固定建模误差；只要 R 候选在宽松保护范围内，
  // 且明显优于另一个 IPPE 镜像解，就可以用它判支，不能再用 8px 的绝对门槛把 R 完全禁用。
  constexpr double r_branch_min_separation_px = 12.0;
  const bool has_r_source_consensus =
    r_diagnostics.network.count > 0 && r_diagnostics.visual.count > 0 &&
    r_diagnostics.network_confidence >= 0.8 &&
    cv::norm(r_diagnostics.network.center - r_diagnostics.visual.center) <=
      r_source_consensus_max_px &&
    best_r_unconstrained.valid &&
    cv::norm(best_r_unconstrained.projected_r - r_diagnostics.network.center) <=
      r_source_consensus_max_px &&
    cv::norm(best_r_unconstrained.projected_r - r_diagnostics.visual.center) <=
      r_source_consensus_max_px;
  const bool best_r_candidate_better_than_continuous =
    szu_use_r_in_pnp_ && best_r_unconstrained.valid &&
    (!best_r_candidate.valid ||
     best_r_unconstrained.r_error + r_error_tie_px < best_r_candidate.r_error);
  const bool better_r_candidate_rejected_by_continuity =
    use_szu_point_indices && had_previous_pose && best_r_candidate_better_than_continuous;
  const bool accept_strong_r_reacquisition =
    better_r_candidate_rejected_by_continuity && has_r_source_consensus &&
    best_r_unconstrained.corner_error <= szu_corner_reprojection_max_px_ &&
    best_r_unconstrained.r_error <= strong_r_reprojection_max_px &&
    best_r_unconstrained.plane_normal_delta_rad <= szu_pose_max_jump_rad_ &&
    best_r_unconstrained.r_center_delta_m <= szu_pose_max_translation_jump_m_;
  // R 不参与最终四角 PnP，但它是区分共面 IPPE 正反镜像解的唯一同帧依据。
  // 用绝对误差上限和两个候选的相对差异共同判定；R 有系统偏差时仍能选出正确镜像，
  // 两个候选接近或 R 明显异常时则退回角点和历史连续性。
  const bool strong_r_branch_for_four_corner =
    use_szu_point_indices && !szu_use_r_in_pnp_ && best_r_unconstrained.valid &&
    r_diagnostics.network.count > 0 && r_diagnostics.network_confidence >= 0.8 &&
    best_r_unconstrained.corner_error <= szu_corner_reprojection_max_px_ &&
    best_r_unconstrained.r_error <= szu_r_reprojection_max_px_ &&
    (!std::isfinite(r_branch_second_error_px) ||
     r_branch_second_error_px - r_branch_best_error_px >= r_branch_min_separation_px);
  if (use_szu_point_indices && had_previous_pose) {
    if (!szu_use_r_in_pnp_) {
      // 小符未激活时目标扇叶会真实切换，不能比较盘面内 roll；但法向和 R 中心
      // 仍应连续，用它们挡住四角 IPPE 的镜像解或明显错误的角点姿态。
      if (strong_r_branch_for_four_corner) {
        best_ptr = &best_r_unconstrained;
      } else if (best_continuous.valid) {
        best_ptr = &best_continuous;
      }
    } else if (accept_strong_r_reacquisition) {
      best_ptr = &best_r_unconstrained;
      tools::logger()->debug(
        "[BuffSolver] 网络与视觉 R 共识支持新姿态，接受 PnP 重获: R={:.2f}px "
        "plane_normal_delta={:.1f}deg R_from_gimbal_delta={:.3f}m",
        best_r_unconstrained.r_error,
        best_r_unconstrained.plane_normal_delta_rad * 180.0 / CV_PI,
        best_r_unconstrained.r_center_delta_m);
    } else if (best_r_candidate.valid && !better_r_candidate_rejected_by_continuity) {
      best_ptr = &best_r_candidate;
    } else if (!better_r_candidate_rejected_by_continuity && best_continuous.valid &&
               best_continuous.r_error <=
                 szu_r_reprojection_max_px_ + szu_r_reprojection_margin_px_) {
      best_ptr = &best_continuous;
    }
  } else if (use_szu_point_indices && szu_use_r_in_pnp_ && best_r_candidate.valid) {
    best_ptr = &best_r_candidate;
  } else if (use_szu_point_indices && !szu_use_r_in_pnp_) {
    // 首次启动没有历史姿态时优先用 R 判别 IPPE 分支；R 不可靠时才退回角点误差。
    best_ptr = strong_r_branch_for_four_corner
      ? &best_r_unconstrained
      : (best_geometry.valid ? &best_geometry : nullptr);
  } else if (!use_szu_point_indices && best_geometry.valid) {
    best_ptr = &best_geometry;
  }
  if (best_ptr == nullptr) {
    p.mark_unsolvable();
    if (use_szu_point_indices) {
      const bool have_five_point = best_five_point.valid;
      const bool five_point_corners_ok = have_five_point &&
        best_five_point.corner_error <= szu_corner_reprojection_max_px_;
      const bool five_point_r_ok = have_five_point &&
        best_five_point.r_error <= szu_r_reprojection_max_px_;
      const bool five_point_continuity_ok = have_five_point &&
        best_five_point.plane_normal_delta_rad <= szu_pose_max_jump_rad_ &&
        best_five_point.r_center_delta_m <= szu_pose_max_translation_jump_m_;
      tools::logger()->debug(
        "[BuffSolver] 五点细化候选: enabled={} found={} corners={:.2f}px R={:.2f}px "
        "pnp=({:.1f},{:.1f}) pass(corners/R/continuity)={}/{}/{}",
        szu_use_r_in_pnp_, have_five_point,
        have_five_point ? best_five_point.corner_error : -1.0,
        have_five_point ? best_five_point.r_error : -1.0,
        have_five_point ? best_five_point.projected_r.x : -1.0F,
        have_five_point ? best_five_point.projected_r.y : -1.0F,
        five_point_corners_ok, five_point_r_ok, five_point_continuity_ok);
    }
    const PnpCandidate * diagnostic_candidate = best_r_unconstrained.valid
      ? &best_r_unconstrained
      : (best_geometry.valid ? &best_geometry : nullptr);
    if (use_szu_point_indices && diagnostic_candidate != nullptr) {
      std::vector<cv::Point2f> diagnostic_projected_r;
      cv::projectPoints(
        r_object_point, diagnostic_candidate->rvec, diagnostic_candidate->tvec, camera_matrix_,
        distort_coeffs_, diagnostic_projected_r);
      if (!diagnostic_projected_r.empty()) {
        p.pnp_r_projected_pixel = diagnostic_projected_r.front();
        p.pnp_r_projection_valid = true;
        log_r_source_diagnostics(
          "PnP 候选被拒绝的 R 来源诊断", diagnostic_projected_r.front(),
          diagnostic_candidate->corner_error);
      }
    }
    if (had_previous_pose) {
      if (better_r_candidate_rejected_by_continuity) {
        tools::logger()->debug(
          "[BuffSolver] 最低 R 误差候选被连续性门限挡住: best_R={:.2f}px "
          "plane_normal_delta={:.1f}deg R_from_gimbal_delta={:.3f}m; 连续候选 R={:.2f}px，本帧不输出姿态",
          best_r_unconstrained.r_error,
          best_r_unconstrained.plane_normal_delta_rad * 180.0 / CV_PI,
          best_r_unconstrained.r_center_delta_m,
          best_r_candidate.valid ? best_r_candidate.r_error : -1.0);
      }
      // 当前帧的姿态跳变超过限制时保留上一帧姿态，避免错误解污染跟踪器。
      ++pose_rejection_count_;
      if (best_geometry.valid) {
        tools::logger()->debug(
          "[BuffSolver] 拒绝 SZU PnP 候选: corners={:.2f}px plane_normal_delta={:.1f}deg "
          "R_from_gimbal_delta={:.3f}m rejected={}/{}，保留上一帧姿态",
          best_geometry.corner_error, best_geometry.plane_normal_delta_rad * 180.0 / CV_PI,
          best_geometry.r_center_delta_m, pose_rejection_count_,
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
    } else if (use_szu_point_indices && !szu_use_r_in_pnp_) {
      pose_valid_ = false;
      tools::logger()->debug(
        "[BuffSolver] SZU 四角 PnP 姿态连续性不通过，等待法向和 R 中心重新稳定");
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
    if (!had_previous_pose) pose_valid_ = false;
    p.mark_unsolvable();
    std::vector<cv::Point2f> rejected_projected_r;
    cv::projectPoints(
      r_object_point, best.rvec, best.tvec, camera_matrix_, distort_coeffs_, rejected_projected_r);
    if (!rejected_projected_r.empty()) {
      p.pnp_r_projected_pixel = rejected_projected_r.front();
      p.pnp_r_projection_valid = true;
    }
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
  plane_normal_world_prior_ = best.plane_normal_world.normalized();
  r_center_from_gimbal_world_prior_ = best.r_center_from_gimbal_world;
  plane_normal_prior_valid_ = true;

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
  // 这里反投影的是可见 R 标，不是符盘旋转中心；旋转中心距离仍由 tvec 单独计算。
  std::vector<cv::Point2f> r_projected_point;
  cv::projectPoints(
    r_object_point, rvec_, tvec_, camera_matrix_, distort_coeffs_, r_projected_point);
  p.pnp_r_reprojection_error_px =
    r_projected_point.empty() ? 0.0 : cv::norm(r_projected_point.front() - p.r_center);
  if (!r_projected_point.empty()) {
    p.pnp_r_projected_pixel = r_projected_point.front();
    p.pnp_r_projection_valid = true;
  }
  if (use_szu_point_indices && !r_projected_point.empty() &&
      p.pnp_r_reprojection_error_px > szu_r_reprojection_max_px_) {
    log_r_source_diagnostics(
      "四角 PnP 已通过，但 R 点不一致", r_projected_point.front(),
      p.pnp_reprojection_error_px);
  }
  if (use_szu_point_indices && !r_projected_point.empty() &&
      p.pnp_r_reprojection_error_px > 8.0 &&
      (++r_offset_diagnostic_counter_ % 30 == 0)) {
    const auto project_r_error = [&](const PnpCandidate & candidate,
                                     const std::vector<cv::Point3f> & object_point) {
      if (!candidate.valid) return -1.0;
      std::vector<cv::Point2f> projected;
      cv::projectPoints(
        object_point, candidate.rvec, candidate.tvec, camera_matrix_, distort_coeffs_, projected);
      return projected.empty() ? -1.0 : cv::norm(projected.front() - p.r_center);
    };
    const double opposite_error = project_r_error(best, opposite_r_object_point);
    const double best_r_opposite_error =
      project_r_error(best_r_unconstrained, opposite_r_object_point);
    const double continuous_opposite_error =
      project_r_error(best_continuous, opposite_r_object_point);
    const double geometry_opposite_error =
      project_r_error(best_geometry, opposite_r_object_point);
    const double r_branch_gap = std::isfinite(r_branch_second_error_px)
      ? r_branch_second_error_px - r_branch_best_error_px : -1.0;
    const auto & network = p.r_point_diagnostics.network;
    tools::logger()->debug(
      "[BuffSolver] R 偏移对照: selected(-/+)= {:.2f}/{:.2f}px "
      "bestR(-/+)= {:.2f}/{:.2f}px continuous(-/+)= {:.2f}/{:.2f}px "
      "geometry(-/+)= {:.2f}/{:.2f}px branch(best/second/gap)={:.2f}/{:.2f}/{:.2f}px "
      "net=({:.1f},{:.1f}) conf={:.3f}",
      p.pnp_r_reprojection_error_px, opposite_error,
      best_r_unconstrained.valid ? best_r_unconstrained.r_error : -1.0,
      best_r_opposite_error,
      best_continuous.valid ? best_continuous.r_error : -1.0,
      continuous_opposite_error,
      best_geometry.valid ? best_geometry.r_error : -1.0,
      geometry_opposite_error,
      r_branch_best_error_px, r_branch_second_error_px, r_branch_gap,
      network.count > 0 ? network.center.x : -1.0F,
      network.count > 0 ? network.center.y : -1.0F,
      p.r_point_diagnostics.network_confidence);

    struct ROffsetFit
    {
      double offset_m = 0.0;
      double error_px = std::numeric_limits<double>::infinity();
      cv::Point2f projected{0.0F, 0.0F};
      bool valid = false;
      bool at_scan_edge = false;
    };
    const auto scan_r_offset = [&](const PnpCandidate & candidate) {
      ROffsetFit result;
      if (!candidate.valid) return result;

      // 只扫描法向位置，不把扫描结果用于控制或候选选择，避免诊断改变实车行为。
      constexpr double scan_min_m = -0.30;
      constexpr double scan_max_m = 0.30;
      constexpr double scan_step_m = 0.01;
      const int scan_count = static_cast<int>(std::lround(
        (scan_max_m - scan_min_m) / scan_step_m));
      std::vector<cv::Point3f> scan_points;
      scan_points.reserve(static_cast<size_t>(scan_count + 1));
      for (int i = 0; i <= scan_count; ++i) {
        scan_points.emplace_back(
          static_cast<float>(scan_min_m + scan_step_m * static_cast<double>(i)), 0.0F, 0.0F);
      }
      std::vector<cv::Point2f> projected_points;
      cv::projectPoints(
        scan_points, candidate.rvec, candidate.tvec, camera_matrix_, distort_coeffs_,
        projected_points);
      if (projected_points.size() != scan_points.size()) return result;

      for (size_t i = 0; i < projected_points.size(); ++i) {
        const double error = cv::norm(projected_points[i] - p.r_center);
        if (error < result.error_px) {
          result.offset_m = scan_points[i].x;
          result.error_px = error;
          result.projected = projected_points[i];
          result.valid = true;
        }
      }
      result.at_scan_edge = result.valid &&
        (std::abs(result.offset_m - scan_min_m) < 0.5 * scan_step_m ||
         std::abs(result.offset_m - scan_max_m) < 0.5 * scan_step_m);
      return result;
    };
    const auto candidate_origin_distance = [](const PnpCandidate & candidate) {
      return candidate.valid
        ? std::sqrt(
            candidate.tvec[0] * candidate.tvec[0] + candidate.tvec[1] * candidate.tvec[1] +
            candidate.tvec[2] * candidate.tvec[2])
        : 0.0;
    };
    const auto selected_fit = scan_r_offset(best);
    const auto best_r_fit = scan_r_offset(best_r_unconstrained);
    const auto continuous_fit = scan_r_offset(best_continuous);
    const auto geometry_fit = scan_r_offset(best_geometry);
    const auto branch0_fit = four_corner_candidates.size() > 0
      ? scan_r_offset(four_corner_candidates[0]) : ROffsetFit{};
    const auto branch1_fit = four_corner_candidates.size() > 1
      ? scan_r_offset(four_corner_candidates[1]) : ROffsetFit{};
    const auto format_fit = [](const ROffsetFit & fit) {
      return fit.valid ? fit.offset_m : 0.0;
    };
    const auto error_fit = [](const ROffsetFit & fit) {
      return fit.valid ? fit.error_px : -1.0;
    };
    const auto edge_fit = [](const ROffsetFit & fit) { return fit.valid && fit.at_scan_edge; };
    const cv::Point2f selected_delta = best.valid ? best.projected_r - p.r_center : cv::Point2f{};
    tools::logger()->debug(
      "[BuffSolver] R 偏移扫描: selected(off/min/origin)={:.3f}/{:.2f}/{:.3f} "
      "delta(pnp-net)=({:.1f},{:.1f}) "
      "branch0(off/min/origin)={:.3f}/{:.2f}/{:.3f} "
      "branch1(off/min/origin)={:.3f}/{:.2f}/{:.3f} "
      "bestR={:.3f}/{:.2f} continuous={:.3f}/{:.2f} geometry={:.3f}/{:.2f} "
      "edge={}/{}/{}/{} net=({:.1f},{:.1f}) final=({:.1f},{:.1f})",
      format_fit(selected_fit), error_fit(selected_fit), candidate_origin_distance(best),
      selected_delta.x, selected_delta.y,
      format_fit(branch0_fit), error_fit(branch0_fit),
      four_corner_candidates.size() > 0 ? candidate_origin_distance(four_corner_candidates[0]) : 0.0,
      format_fit(branch1_fit), error_fit(branch1_fit),
      four_corner_candidates.size() > 1 ? candidate_origin_distance(four_corner_candidates[1]) : 0.0,
      format_fit(best_r_fit), error_fit(best_r_fit),
      format_fit(continuous_fit), error_fit(continuous_fit),
      format_fit(geometry_fit), error_fit(geometry_fit),
      edge_fit(selected_fit), edge_fit(branch0_fit), edge_fit(branch1_fit), edge_fit(best_r_fit),
      network.count > 0 ? network.center.x : -1.0F,
      network.count > 0 ? network.center.y : -1.0F, p.r_center.x, p.r_center.y);
  }
  p.pnp_center_distance_m = std::sqrt(
    tvec_[0] * tvec_[0] + tvec_[1] * tvec_[1] + tvec_[2] * tvec_[2]);
  if (!std::isfinite(p.pnp_reprojection_error_px) ||
      !std::isfinite(p.pnp_r_reprojection_error_px)) {
    tools::logger()->debug(
      "[BuffSolver] PnP 重投影误差非有限: corners={:.2f}px R={:.2f}px origin={:.3f}m",
      p.pnp_reprojection_error_px, p.pnp_r_reprojection_error_px, p.pnp_center_distance_m);
    p.mark_unsolvable();
    if (!had_previous_pose) pose_valid_ = false;
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
  p.pnp_blade_camera_distance_m = blade_xyz_in_camera.norm();

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
    if (!had_previous_pose) pose_valid_ = false;
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
