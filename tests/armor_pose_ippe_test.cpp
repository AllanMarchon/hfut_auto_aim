#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>

#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"

namespace {

using fyt::auto_aim::ArmorDetection;
using fyt::auto_aim::ArmorPoseEstimatorAdapter;
using fyt::auto_aim::PoseConfig;

double reprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const cv::Mat& camera_matrix) {
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
      object_points, rvec, tvec, camera_matrix, cv::Mat(), projected);
  double error = 0.0;
  for (size_t index = 0; index < image_points.size(); ++index) {
    error += cv::norm(image_points[index] - projected[index]);
  }
  return error;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "armor pose IPPE test failed: %s\n", message);
  return code;
}

}  // namespace

int main() {
  PoseConfig config;
  config.pnp_method = "ippe";
  config.refiner.mode = "none";
  ArmorPoseEstimatorAdapter estimator(config);

  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      1739.130435, 0.0, 719.5,
      0.0, 1739.130435, 539.5,
      0.0, 0.0, 1.0);
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.k = {
      1739.130435, 0.0, 719.5,
      0.0, 1739.130435, 539.5,
      0.0, 0.0, 1.0};
  camera_info.d.assign(5, 0.0);

  const auto object_points = ArmorPoseEstimatorAdapter::getObjectPoints(
      "small", config.small_armor_width, config.small_armor_height,
      config.large_armor_width, config.large_armor_height);
  const std::array<Eigen::Vector3d, 4> attitudes{{
      {0.10, 0.20, 0.35},
      {-0.12, 0.18, -0.55},
      {0.16, -0.10, 0.75},
      {-0.08, -0.14, -0.30},
  }};
  const std::array<cv::Point2f, 4> pixel_offsets{{
      {-0.8F, 0.4F}, {0.5F, -0.6F}, {0.9F, 0.3F}, {-0.4F, 0.7F}}};

  for (const auto& attitude : attitudes) {
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(attitude.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(attitude.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(attitude.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    cv::Mat rotation_cv(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotation_cv.at<double>(row, column) = rotation(row, column);
      }
    }
    cv::Mat rvec;
    cv::Rodrigues(rotation_cv, rvec);
    const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.06, -0.03, 2.4);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
        object_points, rvec, tvec, camera_matrix, cv::Mat(), image_points);
    for (size_t index = 0; index < image_points.size(); ++index) {
      image_points[index] += pixel_offsets[index];
    }

    std::vector<cv::Mat> candidates_rvec, candidates_tvec;
    cv::solvePnPGeneric(
        object_points, image_points, camera_matrix, cv::Mat(),
        candidates_rvec, candidates_tvec, false, cv::SOLVEPNP_IPPE);
    double best_positive_error = INFINITY;
    for (size_t index = 0; index < candidates_rvec.size(); ++index) {
      if (candidates_tvec[index].at<double>(2) <= 0.0) continue;
      best_positive_error = std::min(
          best_positive_error,
          reprojectionError(
              object_points, image_points, candidates_rvec[index],
              candidates_tvec[index], camera_matrix));
    }
    if (!std::isfinite(best_positive_error)) {
      return fail("OpenCV produced no positive-depth candidate", 1);
    }

    ArmorDetection detection;
    detection.publish_type = "small";
    detection.publish_number = "4";
    std::copy(image_points.begin(), image_points.end(), detection.keypoints.begin());
    const auto result = estimator.estimate(
        detection, camera_info, Eigen::Matrix3d::Identity());
    if (!result.valid) {
      return fail("adapter rejected a valid tilted projection", 2);
    }
    if (std::abs(result.reprojection_error - best_positive_error) > 1e-6) {
      return fail("adapter did not select minimum-error positive-depth IPPE solution", 3);
    }
  }

  // IPPE ambiguity reporting regression: real keypoints from a 100 fps clip
  // at ~1.5 m where minimum-error selection flickers between the two
  // twisted-pair branches every frame (runner-up error ratio ~1.02, yaw
  // separation ~0.86 rad). The adapter must report a large equivalent yaw
  // uncertainty so downstream filters can distrust the flickering yaw.
  {
    PoseConfig ambiguity_config;
    ambiguity_config.pnp_method = "ippe";
    ambiguity_config.refiner.mode = "none";
    ambiguity_config.small_armor_width = 0.135;
    ArmorPoseEstimatorAdapter ambiguity_estimator(ambiguity_config);

    sensor_msgs::msg::CameraInfo video_info;
    video_info.k = {1125.0, 0.0, 720.0, 0.0, 1125.0, 540.0, 0.0, 0.0, 1.0};
    video_info.d.assign(5, 0.0);

    ArmorDetection frame_a;
    frame_a.publish_type = "small";
    frame_a.publish_number = "3";
    frame_a.keypoints = {cv::Point2f(661.4238F, 667.9716F),
                         cv::Point2f(662.8907F, 631.7054F),
                         cv::Point2f(751.6390F, 636.9826F),
                         cv::Point2f(749.4709F, 673.1351F)};

    const auto result_a = ambiguity_estimator.estimate(
        frame_a, video_info, Eigen::Matrix3d::Identity());
    if (!result_a.valid) {
      return fail("ambiguity regression frame rejected", 4);
    }
    if (result_a.ippe_yaw_ambiguity < 0.2) {
      std::fprintf(stderr, "ippe_yaw_ambiguity=%.3f\n",
                   result_a.ippe_yaw_ambiguity);
      return fail("near-degenerate IPPE pair not reported as ambiguous", 5);
    }
  }
  return 0;
}
