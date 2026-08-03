// TCP contract between the Windows Gestalt proxy and hfut_auto_aim in WSL2.
// All fields are little-endian; both supported hosts are x86-64. TCP messages
// are Envelope followed by payload_size bytes.
#ifndef HFUT_AUTO_AIM_GESTALT_PROTOCOL_HPP
#define HFUT_AUTO_AIM_GESTALT_PROTOCOL_HPP

#include <cstdint>

namespace hfut::gestalt {

inline constexpr char kEnvelopeMagic[8] = {'H', 'F', 'G', 'N', 'E', 'T', '1', '\0'};
inline constexpr uint32_t kProtocolVersion = 1;
inline constexpr uint16_t kDefaultPort = 47000;
inline constexpr uint64_t kMaxFramePayload = 64ULL * 1024ULL * 1024ULL;

enum class MessageType : uint32_t {
  frame = 1,
  command = 2,
};

// Raw formats match the game publisher. LZ4 variants carry the exact same
// bytes losslessly compressed by the Windows proxy.
enum class PixelFormat : uint32_t {
  bgra8 = 1,
  rgba8 = 2,
  a2b10g10r10 = 3,
  lz4_bgra8 = 101,
  lz4_rgba8 = 102,
  lz4_a2b10g10r10 = 103,
};

#pragma pack(push, 1)

struct Envelope {
  char magic[8];
  uint32_t version;
  uint32_t type;
  uint64_t payload_size;
  uint64_t seq;
};

// Followed immediately by pixel_bytes bytes. Camera pose is the exact final
// FSceneView pose that rendered the accompanying pixels, not AttributeMap
// telemetry sampled on a different clock.
struct FrameMetadata {
  uint64_t seq;
  double capture_time_s;
  double world_time_s;

  uint32_t width;
  uint32_t height;
  uint32_t row_bytes;
  uint32_t pixel_format;
  uint64_t pixel_bytes;

  double horizontal_fov_degrees;
  double camera_arm_length_cm;
  double camera_position_ue_cm[3];
  double camera_quaternion_ue_xyzw[4];

  uint32_t writer_process_id;
  uint32_t view_actor_unique_id;
  uint32_t takeover_target_unique_id;
  int32_t takeover_player_id;
  int32_t takeover_attribute_map_id;
  uint32_t takeover_epoch;
  uint32_t identity_flags;
};

#pragma pack(pop)

static_assert(sizeof(Envelope) == 32, "Gestalt Envelope layout changed");
static_assert(sizeof(FrameMetadata) == 148, "Gestalt FrameMetadata layout changed");

}  // namespace hfut::gestalt

#endif  // HFUT_AUTO_AIM_GESTALT_PROTOCOL_HPP
