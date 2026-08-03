#include "webots_bridge_camera.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "../bridge_protocol.hpp"

namespace hfut::io {

namespace {
std::string resolveBridgeDir(const std::string& bridge_dir) {
  if (!bridge_dir.empty()) return bridge_dir;
  const char* env = std::getenv("WEBOTS_ROS_FREE_BRIDGE_DIR");
  if (env != nullptr && env[0] != '\0') return env;
  return bridge::kDefaultBridgeDir;
}

bool readImageFile(
    const std::string& path,
    std::vector<unsigned char>& buffer,
    bridge::FrameHeader& header,
    cv::Mat& image) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;

  in.read(reinterpret_cast<char*>(&header), sizeof(header));
  if (!in || static_cast<size_t>(in.gcount()) != sizeof(header)) return false;
  if (std::memcmp(header.magic, bridge::kFrameMagic, 8) != 0) return false;
  if (header.version != bridge::kProtocolVersion) return false;
  if (header.width == 0 || header.height == 0 || header.data_size == 0) return false;

  const size_t expected = header.data_size;
  buffer.resize(expected);
  in.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(expected));
  if (!in || static_cast<size_t>(in.gcount()) != expected) return false;

  const int width = static_cast<int>(header.width);
  const int height = static_cast<int>(header.height);
  const int channels = header.channels;
  if ((channels != 3 && channels != 4) ||
      static_cast<size_t>(width) * height * channels != expected) {
    return false;
  }

  cv::Mat src(height, width, channels == 4 ? CV_8UC4 : CV_8UC3, buffer.data());
  switch (static_cast<bridge::ImageEncoding>(header.encoding)) {
    case bridge::ImageEncoding::bgr8:
      image = src.clone();
      return true;
    case bridge::ImageEncoding::rgb8:
      cv::cvtColor(src, image, cv::COLOR_RGB2BGR);
      return true;
    case bridge::ImageEncoding::bgra8:
      cv::cvtColor(src, image, cv::COLOR_BGRA2BGR);
      return true;
    default:
      return false;
  }
}
}  // namespace

WebotsBridgeCamera::WebotsBridgeCamera(
    const std::string& bridge_dir,
    BridgeInputMode input_mode) : input_mode_(input_mode) {
  std::string dir = resolveBridgeDir(bridge_dir);
  if (!dir.empty() && dir.back() == '/') dir.pop_back();
  image_frame_path_ = dir + "/" + bridge::kFrameFile;
  armor_pose_frame_path_ = dir + "/" + bridge::kArmorPoseFrameFile;
}

bool WebotsBridgeCamera::read(CameraFrame& frame,
                              std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;) {
    if (tryReadOnce(frame)) return true;
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool WebotsBridgeCamera::tryReadOnce(CameraFrame& frame) {
  return input_mode_ == BridgeInputMode::armor_pose
      ? tryReadArmorPoseOnce(frame) : tryReadVisionOnce(frame);
}

bool WebotsBridgeCamera::tryReadVisionOnce(CameraFrame& frame) {
  bridge::FrameHeader header{};
  cv::Mat image;
  if (!readImageFile(image_frame_path_, buffer_, header, image)) return false;
  if (header.seq <= last_seq_) return false;  // not a new frame yet

  const int w = static_cast<int>(header.width);
  const int h = static_cast<int>(header.height);

  frame.image = std::move(image);
  frame.seq = header.seq;
  frame.input_mode = FrameInputMode::vision;
  frame.direct_armors.clear();
  frame.sim_time_s = header.sim_time_s;
  frame.intrinsics.width = w;
  frame.intrinsics.height = h;
  frame.intrinsics.fx = header.fx;
  frame.intrinsics.fy = header.fy;
  frame.intrinsics.cx = header.cx;
  frame.intrinsics.cy = header.cy;
  for (int i = 0; i < 5; ++i) frame.intrinsics.distortion[i] = header.distortion[i];

  frame.gimbal_yaw = header.gimbal_yaw;
  frame.gimbal_pitch = header.gimbal_pitch;
  frame.gimbal_yaw_vel = header.gimbal_yaw_vel;
  frame.gimbal_pitch_vel = header.gimbal_pitch_vel;

  frame.q_cam2world =
      Eigen::Quaterniond(header.cam_qw, header.cam_qx, header.cam_qy, header.cam_qz);
  frame.q_cam2world.normalize();
  frame.t_cam2world = Eigen::Vector3d(header.cam_tx, header.cam_ty, header.cam_tz);

  last_seq_ = header.seq;
  return true;
}

bool WebotsBridgeCamera::tryReadArmorPoseOnce(CameraFrame& frame) {
  std::ifstream in(armor_pose_frame_path_, std::ios::binary);
  if (!in) return false;

  bridge::ArmorPoseFrameHeader header{};
  in.read(reinterpret_cast<char*>(&header), sizeof(header));
  if (!in || static_cast<size_t>(in.gcount()) != sizeof(header)) return false;
  if (std::memcmp(header.magic, bridge::kArmorPoseMagic, 8) != 0) return false;
  if (header.version != bridge::kArmorPoseProtocolVersion || header.seq <= last_seq_) return false;
  if (header.width == 0 || header.height == 0 ||
      header.armor_count > bridge::kMaxArmorPoses) return false;

  std::vector<bridge::ArmorPoseRecord> records(header.armor_count);
  if (!records.empty()) {
    const auto bytes = static_cast<std::streamsize>(records.size() * sizeof(records[0]));
    in.read(reinterpret_cast<char*>(records.data()), bytes);
    if (!in || in.gcount() != bytes) return false;
  }

  frame.input_mode = FrameInputMode::armor_pose;
  frame.seq = header.seq;
  frame.sim_time_s = header.sim_time_s;
  bridge::FrameHeader image_header{};
  cv::Mat synchronized_image;
  if (readImageFile(image_frame_path_, buffer_, image_header, synchronized_image) &&
      image_header.seq == header.seq) {
    frame.image = std::move(synchronized_image);
  } else {
    frame.image.release();
  }
  frame.intrinsics.width = static_cast<int>(header.width);
  frame.intrinsics.height = static_cast<int>(header.height);
  frame.intrinsics.fx = header.fx;
  frame.intrinsics.fy = header.fy;
  frame.intrinsics.cx = header.cx;
  frame.intrinsics.cy = header.cy;
  std::fill(std::begin(frame.intrinsics.distortion),
            std::end(frame.intrinsics.distortion), 0.0);
  frame.gimbal_yaw = header.gimbal_yaw;
  frame.gimbal_pitch = header.gimbal_pitch;
  frame.gimbal_yaw_vel = header.gimbal_yaw_vel;
  frame.gimbal_pitch_vel = header.gimbal_pitch_vel;
  frame.q_cam2world = Eigen::Quaterniond(
      header.cam_qw, header.cam_qx, header.cam_qy, header.cam_qz);
  if (frame.q_cam2world.norm() <= 1e-12) return false;
  frame.q_cam2world.normalize();
  frame.t_cam2world = Eigen::Vector3d(header.cam_tx, header.cam_ty, header.cam_tz);
  frame.direct_position_noise_std_m = header.position_noise_std_m;
  frame.direct_yaw_noise_std_rad = header.yaw_noise_std_rad;

  frame.direct_armors.clear();
  frame.direct_armors.reserve(records.size());
  for (const auto& record : records) {
    DirectArmorPose armor;
    armor.number.assign(record.number, strnlen(record.number, sizeof(record.number)));
    armor.type.assign(record.type, strnlen(record.type, sizeof(record.type)));
    armor.position_control = Eigen::Vector3d(record.x, record.y, record.z);
    armor.radial_yaw = record.radial_yaw;
    armor.confidence = record.confidence;
    armor.position_noise_std_m = record.position_noise_std_m;
    armor.yaw_noise_std_rad = record.yaw_noise_std_rad;
    armor.view_angle_rad = record.view_angle_rad;
    if (record.surface_orientation_valid != 0U) {
      armor.surface_orientation_control = Eigen::Quaterniond(
          record.surface_qw,
          record.surface_qx,
          record.surface_qy,
          record.surface_qz);
      const double orientation_norm = armor.surface_orientation_control.norm();
      if (std::isfinite(orientation_norm) && orientation_norm > 1e-12) {
        armor.surface_orientation_control.normalize();
        armor.surface_orientation_valid = true;
      }
    }
    if (armor.number.empty() || !armor.position_control.allFinite() ||
        !std::isfinite(armor.radial_yaw) || !std::isfinite(armor.confidence) ||
        !std::isfinite(armor.position_noise_std_m) ||
        !std::isfinite(armor.yaw_noise_std_rad) ||
        !std::isfinite(armor.view_angle_rad) ||
        armor.position_noise_std_m < 0.0 || armor.yaw_noise_std_rad < 0.0) {
      continue;
    }
    frame.direct_armors.push_back(std::move(armor));
  }

  last_seq_ = header.seq;
  return true;
}

}  // namespace hfut::io
