// ROS-free shim for the slice of <rclcpp/rclcpp.hpp> the ported tracker/
// selector/controller use: rclcpp::Time/Duration (via time.hpp), a trivial
// Logger + get_logger, and the RCLCPP_* logging macros routed to stderr.
//
// Logging here is intentionally lightweight; the detector path uses the richer
// FYT logger. These macros keep the gimbal_pipeline sources unmodified.
#ifndef HFUT_COMPAT_RCLCPP_RCLCPP_HPP
#define HFUT_COMPAT_RCLCPP_RCLCPP_HPP

#include <cstdio>
#include <string>

#include <rclcpp/time.hpp>

namespace rclcpp {

class Logger {
 public:
  explicit Logger(std::string name = "") : name_(std::move(name)) {}
  const std::string& get_name() const { return name_; }

 private:
  std::string name_;
};

inline Logger get_logger(const std::string& name) { return Logger(name); }

}  // namespace rclcpp

// Minimal printf-style logging. The variadic args mirror RCLCPP_*'s
// (logger, "fmt", ...) signature. Throttled variant ignores the clock/period.
#define HFUT_LOG_IMPL(level, logger, ...)                          \
  do {                                                             \
    std::fprintf(stderr, "[" level "] [%s] ", (logger).get_name().c_str()); \
    std::fprintf(stderr, __VA_ARGS__);                             \
    std::fprintf(stderr, "\n");                                    \
  } while (0)

#define RCLCPP_DEBUG(logger, ...) HFUT_LOG_IMPL("DEBUG", logger, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) HFUT_LOG_IMPL("INFO", logger, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) HFUT_LOG_IMPL("WARN", logger, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) HFUT_LOG_IMPL("ERROR", logger, __VA_ARGS__)

#define RCLCPP_DEBUG_THROTTLE(logger, clock, period_ms, ...) \
  HFUT_LOG_IMPL("DEBUG", logger, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, period_ms, ...) \
  HFUT_LOG_IMPL("INFO", logger, __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, period_ms, ...) \
  HFUT_LOG_IMPL("WARN", logger, __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, period_ms, ...) \
  HFUT_LOG_IMPL("ERROR", logger, __VA_ARGS__)

#endif  // HFUT_COMPAT_RCLCPP_RCLCPP_HPP
