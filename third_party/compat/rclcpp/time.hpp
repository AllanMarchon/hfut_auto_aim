// ROS-free shim for rclcpp::Time / rclcpp::Duration.
//
// The ported detector carries observation timestamps as rclcpp::Time but only
// uses: default ctor, ctor from int64 nanoseconds, .nanoseconds(), .seconds(),
// and (Time - Time).seconds(). This header provides exactly that with no ROS
// dependency, so the algorithm sources compile unmodified.
#ifndef HFUT_COMPAT_RCLCPP_TIME_HPP
#define HFUT_COMPAT_RCLCPP_TIME_HPP

#include <cstdint>

// Clock-type enum stand-in (only RCL_ROS_TIME is referenced).
#ifndef RCL_ROS_TIME
enum rcl_clock_type_t { RCL_CLOCK_UNINITIALIZED = 0, RCL_ROS_TIME = 1, RCL_SYSTEM_TIME = 2, RCL_STEADY_TIME = 3 };
#endif

namespace rclcpp {

class Duration {
 public:
  explicit Duration(int64_t ns) : ns_(ns) {}
  double seconds() const { return static_cast<double>(ns_) * 1e-9; }
  int64_t nanoseconds() const { return ns_; }

 private:
  int64_t ns_;
};

class Time {
 public:
  Time() = default;
  explicit Time(int64_t nanoseconds) : ns_(nanoseconds) {}
  Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t /*clock*/ = RCL_ROS_TIME)
      : ns_(static_cast<int64_t>(seconds) * 1000000000LL + nanoseconds) {}

  int64_t nanoseconds() const { return ns_; }
  double seconds() const { return static_cast<double>(ns_) * 1e-9; }

  Duration operator-(const Time& other) const { return Duration(ns_ - other.ns_); }
  bool operator==(const Time& other) const { return ns_ == other.ns_; }
  bool operator!=(const Time& other) const { return ns_ != other.ns_; }
  bool operator<(const Time& other) const { return ns_ < other.ns_; }
  bool operator>(const Time& other) const { return ns_ > other.ns_; }
  bool operator<=(const Time& other) const { return ns_ <= other.ns_; }
  bool operator>=(const Time& other) const { return ns_ >= other.ns_; }

 private:
  int64_t ns_ = 0;
};

}  // namespace rclcpp

#endif  // HFUT_COMPAT_RCLCPP_TIME_HPP
