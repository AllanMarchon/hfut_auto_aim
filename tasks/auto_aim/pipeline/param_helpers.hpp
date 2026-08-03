// Shared inline param helpers lifted from gimbal_pipeline_node.cpp's anonymous
// namespace. They take ParameterHost& (was rclcpp::Node&); hasParameterOverride
// is now a member. Used by both pipeline_params.cpp and
// pipeline_controller_config.cpp.
#ifndef HFUT_PIPELINE_PARAM_HELPERS_HPP_
#define HFUT_PIPELINE_PARAM_HELPERS_HPP_

#include <cmath>
#include <initializer_list>
#include <string>
#include <unordered_set>

#include "parameter_host.hpp"

namespace hfut::pipeline {
namespace param_detail {

inline bool shouldWarnDeprecatedOnce(const std::string & deprecated_key) {
  static std::unordered_set<std::string> warned_keys;
  return warned_keys.insert(deprecated_key).second;
}

inline double readCompatDoubleParameter(
  ParameterHost & node,
  const std::string & canonical_key,
  const std::string & deprecated_key,
  double conflict_eps = 1e-9)
{
  const double canonical_value = node.get_parameter(canonical_key).as_double();
  const double deprecated_value = node.get_parameter(deprecated_key).as_double();
  const bool canonical_overridden = node.hasParameterOverride(canonical_key);
  const bool deprecated_overridden = node.hasParameterOverride(deprecated_key);

  if (deprecated_overridden && !canonical_overridden) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is deprecated; please use '%s'. Applying deprecated value: %.6f",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value);
    }
    return deprecated_value;
  }

  if (deprecated_overridden && canonical_overridden &&
    std::abs(canonical_value - deprecated_value) > conflict_eps)
  {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Both deprecated '%s' and canonical '%s' are set with different values "
        "(deprecated=%.6f, canonical=%.6f). Canonical value will be used.",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value,
        canonical_value);
    }
  }

  return canonical_value;
}

inline int readCompatIntParameter(
  ParameterHost & node,
  const std::string & canonical_key,
  const std::string & deprecated_key)
{
  const int canonical_value = node.get_parameter(canonical_key).as_int();
  const int deprecated_value = node.get_parameter(deprecated_key).as_int();
  const bool canonical_overridden = node.hasParameterOverride(canonical_key);
  const bool deprecated_overridden = node.hasParameterOverride(deprecated_key);

  if (deprecated_overridden && !canonical_overridden) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is deprecated; please use '%s'. Applying deprecated value: %d",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value);
    }
    return deprecated_value;
  }

  if (deprecated_overridden && canonical_overridden && canonical_value != deprecated_value) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Both deprecated '%s' and canonical '%s' are set with different values "
        "(deprecated=%d, canonical=%d). Canonical value will be used.",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value,
        canonical_value);
    }
  }

  return canonical_value;
}

inline bool readCompatBoolParameter(
  ParameterHost & node,
  const std::string & canonical_key,
  const std::string & deprecated_key)
{
  const bool canonical_value = node.get_parameter(canonical_key).as_bool();
  const bool deprecated_value = node.get_parameter(deprecated_key).as_bool();
  const bool canonical_overridden = node.hasParameterOverride(canonical_key);
  const bool deprecated_overridden = node.hasParameterOverride(deprecated_key);

  if (deprecated_overridden && !canonical_overridden) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is deprecated; please use '%s'. Applying deprecated value: %s",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value ? "true" : "false");
    }
    return deprecated_value;
  }

  if (deprecated_overridden && canonical_overridden && canonical_value != deprecated_value) {
    if (shouldWarnDeprecatedOnce(deprecated_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Both deprecated '%s' and canonical '%s' are set with different values "
        "(deprecated=%s, canonical=%s). Canonical value will be used.",
        deprecated_key.c_str(),
        canonical_key.c_str(),
        deprecated_value ? "true" : "false",
        canonical_value ? "true" : "false");
    }
  }

  return canonical_value;
}

inline double readUnifiedDoubleParameter(
  ParameterHost & node,
  const std::string & canonical_key,
  const std::initializer_list<std::string> & fallback_keys,
  double conflict_eps = 1e-9)
{
  const double canonical_value = node.get_parameter(canonical_key).as_double();
  const bool canonical_overridden = node.hasParameterOverride(canonical_key);

  if (canonical_overridden) {
    for (const auto & fallback_key : fallback_keys) {
      if (!node.hasParameterOverride(fallback_key)) {
        continue;
      }
      const double fallback_value = node.get_parameter(fallback_key).as_double();
      if (std::abs(canonical_value - fallback_value) > conflict_eps &&
        shouldWarnDeprecatedOnce(fallback_key))
      {
        RCLCPP_WARN(
          node.get_logger(),
          "Both '%s' and compatibility key '%s' are set with different values "
          "(canonical=%.6f, compatibility=%.6f). Canonical value will be used.",
          canonical_key.c_str(),
          fallback_key.c_str(),
          canonical_value,
          fallback_value);
      }
    }
    return canonical_value;
  }

  for (const auto & fallback_key : fallback_keys) {
    if (!node.hasParameterOverride(fallback_key)) {
      continue;
    }
    const double fallback_value = node.get_parameter(fallback_key).as_double();
    if (shouldWarnDeprecatedOnce(fallback_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is preferred; applying compatibility key '%s' value: %.6f",
        canonical_key.c_str(),
        fallback_key.c_str(),
        fallback_value);
    }
    return fallback_value;
  }

  return canonical_value;
}

inline int readUnifiedIntParameter(
  ParameterHost & node,
  const std::string & canonical_key,
  const std::initializer_list<std::string> & fallback_keys)
{
  const int canonical_value = node.get_parameter(canonical_key).as_int();
  const bool canonical_overridden = node.hasParameterOverride(canonical_key);

  if (canonical_overridden) {
    for (const auto & fallback_key : fallback_keys) {
      if (!node.hasParameterOverride(fallback_key)) {
        continue;
      }
      const int fallback_value = node.get_parameter(fallback_key).as_int();
      if (canonical_value != fallback_value && shouldWarnDeprecatedOnce(fallback_key)) {
        RCLCPP_WARN(
          node.get_logger(),
          "Both '%s' and compatibility key '%s' are set with different values "
          "(canonical=%d, compatibility=%d). Canonical value will be used.",
          canonical_key.c_str(),
          fallback_key.c_str(),
          canonical_value,
          fallback_value);
      }
    }
    return canonical_value;
  }

  for (const auto & fallback_key : fallback_keys) {
    if (!node.hasParameterOverride(fallback_key)) {
      continue;
    }
    const int fallback_value = node.get_parameter(fallback_key).as_int();
    if (shouldWarnDeprecatedOnce(fallback_key)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Parameter '%s' is preferred; applying compatibility key '%s' value: %d",
        canonical_key.c_str(),
        fallback_key.c_str(),
        fallback_value);
    }
    return fallback_value;
  }

  return canonical_value;
}

}  // namespace param_detail

using param_detail::readCompatBoolParameter;
using param_detail::readCompatIntParameter;
using param_detail::readCompatDoubleParameter;
using param_detail::readUnifiedDoubleParameter;
using param_detail::readUnifiedIntParameter;

// Free-function form used directly by lifted ctor code:
// hasParameterOverride(*this, key) -> host.hasParameterOverride(key).
inline bool hasParameterOverride(ParameterHost & host, const std::string & key) {
  return host.hasParameterOverride(key);
}

}  // namespace hfut::pipeline
#endif
