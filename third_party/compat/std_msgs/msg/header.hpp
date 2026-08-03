// ROS-free shim for std_msgs::msg::Header.
//
// The ported detector only carries a Header to pass a frame timestamp +
// frame_id through; it never reads the fields inside the detector core. This
// minimal stand-in keeps the algorithm sources unmodified.
#ifndef HFUT_COMPAT_STD_MSGS_HEADER_HPP
#define HFUT_COMPAT_STD_MSGS_HEADER_HPP

#include <cstdint>
#include <string>

namespace std_msgs {
namespace msg {

struct Time {
  int32_t sec = 0;
  uint32_t nanosec = 0;
};

struct Header {
  Time stamp;
  std::string frame_id;
};

}  // namespace msg
}  // namespace std_msgs

#endif  // HFUT_COMPAT_STD_MSGS_HEADER_HPP
