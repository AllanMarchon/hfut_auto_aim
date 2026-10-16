#include "sp_rune_solver.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "tools/math_tools.hpp"

namespace auto_buff
{
SPRuneSolver::SPRuneSolver(BuffConfig::Camera camera) : camera_(std::move(camera)) {}

std::optional<SPRuneObservation> SPRuneSolver::solve(const RuneElements & elements) const
{
  if (elements.icons.empty()) return std::nullopt;
  const auto bull = std::max_element(elements.bullseyes.begin(), elements.bullseyes.end(),
    [](const auto & a, const auto & b) {
      if (a.active != b.active) return a.active;
      return a.score < b.score;
    });
  if (bull == elements.bullseyes.end() || bull->active) return std::nullopt;
  const auto icon = std::min_element(elements.icons.begin(), elements.icons.end(),
    [&](const auto & a, const auto & b) {
      return cv::norm(a.center - bull->center) < cv::norm(b.center - bull->center);
    });
  auto corners = bull->corners;
  const auto top = std::max_element(
    corners.begin(), corners.end(), [&](const auto & a, const auto & b) {
    return cv::norm(a - icon->center) < cv::norm(b - icon->center);
  });
  const auto bottom = std::min_element(
    corners.begin(), corners.end(), [&](const auto & a, const auto & b) {
    return cv::norm(a - icon->center) < cv::norm(b - icon->center);
  });
  if (top == bottom || cv::norm(*top - *bottom) < 1e-6) return std::nullopt;
  std::vector<cv::Point2f> sides;
  for (auto it = corners.begin(); it != corners.end(); ++it)
    if (it != top && it != bottom) sides.push_back(*it);
  const auto up = *top - *bottom;
  const cv::Point2f right(up.y, -up.x);
  if ((sides[0] - bull->center).dot(right) < (sides[1] - bull->center).dot(right))
    std::swap(sides[0], sides[1]);
  // 保留比赛符盘的平面 127 mm 菱形和 700 mm 符臂尺寸。
  const std::vector<cv::Point3f> object{{0, 0, 0.827f}, {0, 0.127f, 0.7f},
    {0, 0, 0.573f}, {0, -0.127f, 0.7f}};
  const std::vector<cv::Point2f> image{*top, sides[0], *bottom, sides[1]};
  cv::Vec3d rvec, tvec;
  try {
    if (!cv::solvePnP(object, image, camera_.camera_matrix, camera_.distort_coeffs,
                      rvec, tvec, false, cv::SOLVEPNP_IPPE) || tvec[2] <= 0)
      return std::nullopt;
  } catch (const cv::Exception &) { return std::nullopt; }
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_buff2camera;
  cv::cv2eigen(rmat, R_buff2camera);
  const Eigen::Matrix3d R_camera2world =
    q_gimbal2world_.toRotationMatrix() * camera_.R_camera2gimbal;
  const Eigen::Vector3d center = q_gimbal2world_ *
    (camera_.R_camera2gimbal * Eigen::Vector3d(tvec[0], tvec[1], tvec[2]) +
     camera_.t_camera2gimbal);
  const Eigen::Vector3d blade =
    center + R_camera2world * R_buff2camera * Eigen::Vector3d(0, 0, 0.7);
  SPRuneObservation result{tools::xyz2ypd(center),
    tools::eulers(R_camera2world * R_buff2camera, 2, 1, 0), tools::xyz2ypd(blade)};
  if (!result.ypd_in_world.allFinite() || !result.ypr_in_world.allFinite() ||
      !result.blade_ypd_in_world.allFinite()) return std::nullopt;
  return result;
}

std::optional<cv::Point2f> SPRuneSolver::project(const Eigen::Vector3d & world) const
{
  const Eigen::Vector3d point = camera_.R_camera2gimbal.transpose() *
    (q_gimbal2world_.conjugate() * world - camera_.t_camera2gimbal);
  if (!point.allFinite() || point.z() <= 0.1) return std::nullopt;
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(
    std::vector<cv::Point3f>{{static_cast<float>(point.x()), static_cast<float>(point.y()),
                            static_cast<float>(point.z())}},
    cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_.camera_matrix, camera_.distort_coeffs, pixels);
  return pixels.front();
}
}  // namespace auto_buff
