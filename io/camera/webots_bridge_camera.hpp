// Reads camera_frame.bin and, in armor-pose mode, armor_pose_frame.bin written
// by the Webots bridge. The image is attached only when both packets share seq.
#ifndef HFUT_AUTO_AIM_WEBOTS_BRIDGE_CAMERA_HPP
#define HFUT_AUTO_AIM_WEBOTS_BRIDGE_CAMERA_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "hfut_auto_aim/camera_frame.hpp"

namespace hfut::io {

enum class BridgeInputMode { vision, armor_pose };

class WebotsBridgeCamera {
 public:
  // bridge_dir defaults to $WEBOTS_ROS_FREE_BRIDGE_DIR or the protocol default.
  explicit WebotsBridgeCamera(
      const std::string& bridge_dir = "",
      BridgeInputMode input_mode = BridgeInputMode::vision);

  // Blocks until a frame with seq > last delivered is readable, or until
  // `timeout` elapses. Returns true and fills `frame` on success.
  bool read(CameraFrame& frame,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

  uint64_t last_seq() const { return last_seq_; }
  BridgeInputMode inputMode() const { return input_mode_; }

 private:
  bool tryReadOnce(CameraFrame& frame);
  bool tryReadVisionOnce(CameraFrame& frame);
  bool tryReadArmorPoseOnce(CameraFrame& frame);

  std::string image_frame_path_;
  std::string armor_pose_frame_path_;
  BridgeInputMode input_mode_{BridgeInputMode::vision};
  uint64_t last_seq_ = 0;
  std::vector<unsigned char> buffer_;  // reused read scratch
};

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_WEBOTS_BRIDGE_CAMERA_HPP
