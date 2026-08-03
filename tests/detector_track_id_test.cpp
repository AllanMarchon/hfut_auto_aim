#include <cstdio>
#include <vector>

#include "armor_detector_nn/core/tracker/internal_iou_tracker_strategy.hpp"

namespace {

using fyt::auto_aim::ArmorDetection;
using fyt::auto_aim::InternalIoUTrackerStrategy;
using fyt::auto_aim::TrackerConfig;

ArmorDetection detection(float x) {
  ArmorDetection result;
  result.bbox = cv::Rect2f(x, 100.0F, 80.0F, 40.0F);
  result.center = cv::Point2f(x + 40.0F, 120.0F);
  result.publish_number = "4";
  result.publish_type = "small";
  return result;
}

int fail(const char* message, int code) {
  std::fprintf(stderr, "detector track id test failed: %s\n", message);
  return code;
}

}  // namespace

int main() {
  TrackerConfig config;
  config.iou_threshold = 0.2;
  config.max_center_dist_px = 100;
  InternalIoUTrackerStrategy tracker(config);

  const auto first = tracker.associate(
      {detection(100.0F), detection(300.0F)}, rclcpp::Time(1000000000LL));
  if (first.size() != 2 || first[0].track_id == first[1].track_id) {
    return fail("initial detections did not receive distinct IDs", 1);
  }
  const int left_id = first[0].track_id;
  const int right_id = first[1].track_id;

  const auto reordered = tracker.associate(
      {detection(304.0F), detection(104.0F)}, rclcpp::Time(1032000000LL));
  if (reordered.size() != 2) {
    return fail("reordered frame lost a detection", 2);
  }
  if (reordered[0].track_id != right_id || reordered[1].track_id != left_id) {
    return fail("IDs followed container order instead of spatial tracks", 3);
  }
  if (reordered[0].det.track_id != right_id ||
      reordered[1].det.track_id != left_id) {
    return fail("assigned IDs were not copied into ArmorDetection", 4);
  }
  return 0;
}
