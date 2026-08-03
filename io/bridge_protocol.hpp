// Shared wire contract between the Webots sim bridge controllers and the
// ros-free hfut_auto_aim program. Both sides include THIS file; it is the
// single source of truth for the on-disk binary layout. No dependencies
// beyond <cstdint> so it can be dropped into either build unchanged.
//
// Transport: files under ${WEBOTS_ROS_FREE_BRIDGE_DIR} (default
// /tmp/hfut_auto_aim_webots). The writer always writes to "<name>.tmp" then
// rename(2)s over the target, so a reader either sees the whole previous file
// or the whole new one (POSIX atomic rename). Readers detect a new payload by
// the monotonically increasing `seq` in the header.
//
//   camera_frame.bin    sim -> auto_aim   magic "HFUTFRM1"  FrameHeader + pixels
//   armor_pose_frame.bin sim -> auto_aim  magic "HFUTARM1"  ArmorPoseFrameHeader + poses
//   gimbal_command.bin  auto_aim -> sim   magic "HFUTCMD1"  CommandPacket
//
// Angles in BOTH structs are RADIANS. (The legacy ROS GimbalCmd used degrees;
// the conversion is done once at the io boundary, not on the wire.)

#ifndef HFUT_AUTO_AIM_BRIDGE_PROTOCOL_HPP
#define HFUT_AUTO_AIM_BRIDGE_PROTOCOL_HPP

#include <cstdint>

namespace hfut::bridge {

inline constexpr char kFrameMagic[8] = {'H', 'F', 'U', 'T', 'F', 'R', 'M', '1'};
inline constexpr char kArmorPoseMagic[8] = {'H', 'F', 'U', 'T', 'A', 'R', 'M', '1'};
inline constexpr char kCommandMagic[8] = {'H', 'F', 'U', 'T', 'C', 'M', 'D', '1'};
inline constexpr uint32_t kProtocolVersion = 1;
inline constexpr uint32_t kArmorPoseProtocolVersion = 3;

inline constexpr const char* kDefaultBridgeDir = "/tmp/hfut_auto_aim_webots";
inline constexpr const char* kFrameFile = "camera_frame.bin";
inline constexpr const char* kArmorPoseFrameFile = "armor_pose_frame.bin";
inline constexpr const char* kCommandFile = "gimbal_command.bin";
inline constexpr uint32_t kMaxArmorPoses = 16;

// Pixel layout of the payload that follows FrameHeader.
enum class ImageEncoding : uint8_t { bgr8 = 0, rgb8 = 1, bgra8 = 2 };

// Mirrors rm_interfaces/msg/GimbalCmd mode semantics.
enum class GimbalMode : int8_t {
  blind_camera_result = -2,
  no_valid_measurement = -1,
  unknown = 0,
  normal_measurement = 1,
};

#pragma pack(push, 1)

// camera_frame.bin: header is immediately followed by `data_size` bytes of raw
// pixels (width*height*channels, row-major, no padding).
struct FrameHeader {
  char magic[8];        // kFrameMagic
  uint32_t version;     // kProtocolVersion
  uint32_t reserved0;   // keep 8-byte alignment for the doubles below
  uint64_t seq;         // monotonically increasing frame counter
  double sim_time_s;    // Webots simulation time, seconds

  uint32_t width;
  uint32_t height;
  uint8_t encoding;     // ImageEncoding
  uint8_t channels;     // 3 (bgr8/rgb8) or 4 (bgra8)
  uint16_t reserved1;
  uint32_t reserved2;

  // Pinhole intrinsics + plumb_bob distortion.
  double fx, fy, cx, cy;
  double distortion[5];

  // Gimbal feedback (replaces /joint_states). Radians, rad/s.
  double gimbal_yaw, gimbal_pitch;
  double gimbal_yaw_vel, gimbal_pitch_vel;

  // camera_optical_frame -> control frame pose. The control frame keeps
  // odom-aligned axes but its origin is the gimbal/shooter, matching the
  // selector and ballistic solver contract. Quaternion is (w,x,y,z);
  // translation in meters.
  double cam_qw, cam_qx, cam_qy, cam_qz;
  double cam_tx, cam_ty, cam_tz;

  uint64_t data_size;   // bytes of pixel payload following this header
};

// armor_pose_frame.bin: direct noisy armor measurements in the control frame.
// In direct mode camera_frame.bin may accompany this packet with the same seq;
// camera fields keep projection and diagnostics valid even without that image.
struct ArmorPoseFrameHeader {
  char magic[8];
  uint32_t version;
  uint32_t reserved0;
  uint64_t seq;
  double sim_time_s;

  uint32_t width;
  uint32_t height;
  uint32_t armor_count;
  uint32_t reserved1;

  double fx, fy, cx, cy;
  double gimbal_yaw, gimbal_pitch;
  double gimbal_yaw_vel, gimbal_pitch_vel;
  double cam_qw, cam_qx, cam_qy, cam_qz;
  double cam_tx, cam_ty, cam_tz;

  double position_noise_std_m;
  double yaw_noise_std_rad;
};

struct ArmorPoseRecord {
  char number[16];
  char type[8];
  double x, y, z;       // control-frame position, meters
  double radial_yaw;    // center-to-armor yaw in control-frame axes, radians
  double confidence;
  double position_noise_std_m;
  double yaw_noise_std_rad;
  double view_angle_rad;

  // Complete control-frame plate pose. The right-handed local armor frame is
  // X = outward surface normal, Y = plate width/left, Z = plate height/up.
  uint8_t surface_orientation_valid;
  uint8_t reserved0[7];
  double surface_qw, surface_qx, surface_qy, surface_qz;
};

// gimbal_command.bin: a single fixed-size record.
struct CommandPacket {
  char magic[8];        // kCommandMagic
  uint32_t version;     // kProtocolVersion
  uint32_t reserved0;
  uint64_t seq;         // command counter
  double sim_time_s;    // sim_time of the frame this command was computed from

  // Absolute setpoints (radians) and the diff/velocity/accel feed-forwards the
  // sim's mpc_state follower consumes.
  double yaw, yaw_diff, yaw_vel, yaw_acc;
  double pitch, pitch_diff, pitch_vel, pitch_acc;
  double distance;      // meters, physical range to target (0 if none)

  uint8_t fire_advice;  // 0/1
  int8_t mode;          // GimbalMode
  uint16_t reserved1;
  uint32_t reserved2;
};

#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 216, "FrameHeader layout changed");
static_assert(sizeof(ArmorPoseFrameHeader) == 184, "ArmorPoseFrameHeader layout changed");
static_assert(sizeof(ArmorPoseRecord) == 128, "ArmorPoseRecord layout changed");
static_assert(sizeof(CommandPacket) == 112, "CommandPacket layout changed");

}  // namespace hfut::bridge

#endif  // HFUT_AUTO_AIM_BRIDGE_PROTOCOL_HPP
