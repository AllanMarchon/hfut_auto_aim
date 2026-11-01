#ifndef HFUT_AUTO_AIM_SENTRY_PACKET_HPP
#define HFUT_AUTO_AIM_SENTRY_PACKET_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace hfut::io {

struct __attribute__((packed)) SentryCommandPacket {
  std::uint8_t header = 0xFF;
  std::uint8_t fire = 0;
  float pitch = 0.0F;
  float yaw = 0.0F;
  float distance = 0.0F;
  std::int32_t target_id = -1;
  float pitch_velocity = 0.0F;
  float yaw_velocity = 0.0F;
  std::uint8_t reserved[36]{};
  std::uint8_t checksum = 0;
  std::uint8_t tail = 0x0D;
};

struct __attribute__((packed)) SentryFeedbackPacket {
  std::uint8_t header = 0xFF;
  std::uint8_t mode = 0;
  float roll = 0.0F;
  float yaw = 0.0F;
  float pitch = 0.0F;
  std::uint8_t reserved[48]{};
  std::uint8_t checksum = 0;
  std::uint8_t tail = 0x0D;
};

inline constexpr std::size_t kSentryPacketSize = 64;
inline constexpr char kSentryCommandProtocol[] = "sentry_64";
inline constexpr char kSentryFeedbackProtocol[] = "sentry_feedback_64";

static_assert(std::is_standard_layout_v<SentryCommandPacket>);
static_assert(std::is_standard_layout_v<SentryFeedbackPacket>);
static_assert(sizeof(SentryCommandPacket) == kSentryPacketSize);
static_assert(sizeof(SentryFeedbackPacket) == kSentryPacketSize);
static_assert(offsetof(SentryCommandPacket, pitch) == 2);
static_assert(offsetof(SentryCommandPacket, yaw) == 6);
static_assert(offsetof(SentryCommandPacket, distance) == 10);
static_assert(offsetof(SentryCommandPacket, target_id) == 14);
static_assert(offsetof(SentryCommandPacket, pitch_velocity) == 18);
static_assert(offsetof(SentryCommandPacket, yaw_velocity) == 22);
static_assert(offsetof(SentryCommandPacket, checksum) == 62);
static_assert(offsetof(SentryCommandPacket, tail) == 63);
static_assert(offsetof(SentryFeedbackPacket, roll) == 2);
static_assert(offsetof(SentryFeedbackPacket, yaw) == 6);
static_assert(offsetof(SentryFeedbackPacket, pitch) == 10);
static_assert(offsetof(SentryFeedbackPacket, checksum) == 62);
static_assert(offsetof(SentryFeedbackPacket, tail) == 63);

}  // namespace hfut::io

#endif  // HFUT_AUTO_AIM_SENTRY_PACKET_HPP
