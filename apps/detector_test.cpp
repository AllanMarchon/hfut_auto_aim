// Phase B verification: load the NN detector from configs/detector.yaml, decode a real
// captured sim frame, run detectBatch + pose estimation (the anti-fling IPPE +
// single-yaw path), and print detections with 3D poses.
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "hfut_auto_aim/camera_frame.hpp"
#include "io/camera/webots_bridge_camera.hpp"
#include "config_loader.hpp"
#include "armor_detector_nn/core/armor_detector_nn.hpp"
#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"
#include "armor_detector_nn/core/pose_refine/pose_refiner.hpp"
#include "rm_utils/logger/log.hpp"

using namespace fyt::auto_aim;

int main(int argc, char** argv) {
  // Register the named loggers the detector logs to (the ROS node did this).
  FYT_REGISTER_LOGGER("armor_detector", "/tmp/hfut_auto_aim_log", INFO);
  FYT_REGISTER_LOGGER("armor_detector_nn", "/tmp/hfut_auto_aim_log", INFO);

  const std::string cfg_path = argc > 1 ? argv[1] : "configs/detector.yaml";
  const std::string bridge_dir = argc > 2 ? argv[2] : "/tmp/hfut_detector_test";

  DetectorConfig cfg = hfut::detector::loadDetectorConfigFile(cfg_path);
  std::printf("[test] loaded config: backend=%s model=%s strategy=%s refiner=%s\n",
              backendTypeToString(cfg.backend.type).c_str(), cfg.backend.model_path.c_str(),
              cfg.postprocess.strategy.c_str(), cfg.pose.refiner.mode.c_str());

  ArmorDetectorNN detector(cfg);
  if (!detector.initialize()) { std::printf("[test] FAIL: detector init\n"); return 1; }
  std::printf("[test] detector initialized, backend=%s\n",
              detector.backendInfo().backend_name.c_str());

  ArmorPoseEstimatorAdapter pose_adapter(cfg.pose);
  if (cfg.pose.refiner.mode == "single_yaw") {
    pose_adapter.setRefiner(std::make_shared<SingleYawRefiner>(cfg.pose.single_yaw, cfg.pose.gate));
  }

  hfut::io::WebotsBridgeCamera camera(bridge_dir);
  hfut::CameraFrame frame;
  if (!camera.read(frame, std::chrono::milliseconds(2000))) {
    std::printf("[test] FAIL: no frame from %s\n", bridge_dir.c_str());
    return 1;
  }
  std::printf("[test] frame seq=%llu %dx%d fx=%.1f\n",
              (unsigned long long)frame.seq, frame.image.cols, frame.image.rows,
              frame.intrinsics.fx);

  // Build the CameraInfo shim from the frame intrinsics.
  sensor_msgs::msg::CameraInfo cam_info;
  cam_info.width = frame.intrinsics.width;
  cam_info.height = frame.intrinsics.height;
  cam_info.k = {frame.intrinsics.fx, 0, frame.intrinsics.cx,
                0, frame.intrinsics.fy, frame.intrinsics.cy, 0, 0, 1};
  cam_info.d.assign(frame.intrinsics.distortion, frame.intrinsics.distortion + 5);

  std_msgs::msg::Header header;
  auto results = detector.detectBatch({frame.image}, {header});
  if (results.empty()) { std::printf("[test] FAIL: detectBatch empty result\n"); return 1; }

  const auto& dets = results[0].detections;
  std::printf("[test] detections: %zu\n", dets.size());

  Eigen::Matrix3d R_cam2world = frame.R_cam2world();
  auto poses = pose_adapter.estimateBatch(dets, cam_info, R_cam2world);

  for (size_t i = 0; i < dets.size(); ++i) {
    const auto& d = dets[i];
    std::printf("  armor[%zu] %s/%s conf=%.2f center=(%.0f,%.0f)", i,
                d.publish_number.c_str(), d.publish_type.c_str(), d.confidence,
                d.center.x, d.center.y);

    // Keypoint pixel spans. Corner order: left_bottom, left_top, right_top,
    // right_bottom. Width span = left->right, height span = bottom->top. For a
    // Compare this span against fx * physical_width / optical_depth. Do not use
    // the target robot center distance: the camera has an extrinsic offset and
    // each rotating armor is displaced from the robot center.
    const auto& k = d.keypoints;
    auto px = [](const cv::Point2f& a, const cv::Point2f& b) {
      return std::hypot(a.x - b.x, a.y - b.y);
    };
    double w_left = px(k[0], k[1]);   // left bar length
    double w_right = px(k[2], k[3]);  // right bar length
    double span_lr = 0.5 * (px(k[1], k[2]) + px(k[0], k[3]));  // top edge + bottom edge
    std::printf(" | kpts=[(%.0f,%.0f)(%.0f,%.0f)(%.0f,%.0f)(%.0f,%.0f)] "
                "bar_len=%.0f/%.0f width_px=%.0f",
                k[0].x, k[0].y, k[1].x, k[1].y, k[2].x, k[2].y, k[3].x, k[3].y,
                w_left, w_right, span_lr);

    if (i < poses.size() && poses[i].valid) {
      const auto& p = poses[i];
      std::printf(" | pose xyz=(%.3f,%.3f,%.3f) yaw=%.3f reproj=%.2f mode=%d",
                  p.translation.x(), p.translation.y(), p.translation.z(), p.yaw,
                  p.reprojection_error, (int)p.mode);
    } else {
      std::printf(" | pose INVALID");
    }
    std::printf("\n");
  }

  std::printf("[test] %s\n", dets.empty() ? "NO DETECTIONS (check target in view)" : "DETECTOR OK");
  return 0;
}
