// ROS-free host for the rclcpp parameter + logging API surface the ported
// gimbal_pipeline node methods use. A Pipeline inherits ParameterHost so that
// declareXxxParameters() / applyTrackerParamsToConfig() / initGimbalStrategies()
// can be lifted from gimbal_pipeline_node.cpp almost verbatim.
//
// Parameters are loaded from a ROS-style YAML (`<node>: ros__parameters: ...`),
// flattened to dotted keys. declare_parameter(name, default) returns the YAML
// value if present, else registers and returns the default.
#ifndef HFUT_PIPELINE_PARAMETER_HOST_HPP
#define HFUT_PIPELINE_PARAMETER_HOST_HPP

#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>  // compat shim: Logger + RCLCPP_* macros

namespace hfut::pipeline {

class ParamValue {
 public:
  using Storage = std::variant<bool, int64_t, double, std::string,
                               std::vector<std::string>, std::vector<int64_t>,
                               std::vector<double>>;

  ParamValue() : v_(int64_t{0}) {}
  explicit ParamValue(Storage v) : v_(std::move(v)) {}

  double as_double() const {
    if (std::holds_alternative<double>(v_)) return std::get<double>(v_);
    if (std::holds_alternative<int64_t>(v_)) return static_cast<double>(std::get<int64_t>(v_));
    throw std::runtime_error("param not a double");
  }
  int64_t as_int() const {
    if (std::holds_alternative<int64_t>(v_)) return std::get<int64_t>(v_);
    if (std::holds_alternative<double>(v_)) return static_cast<int64_t>(std::get<double>(v_));
    throw std::runtime_error("param not an int");
  }
  bool as_bool() const { return std::get<bool>(v_); }
  std::string as_string() const { return std::get<std::string>(v_); }
  std::vector<std::string> as_string_array() const {
    return std::get<std::vector<std::string>>(v_);
  }
  std::vector<int64_t> as_integer_array() const {
    return std::get<std::vector<int64_t>>(v_);
  }
  std::vector<double> as_double_array() const {
    return std::get<std::vector<double>>(v_);
  }

 private:
  Storage v_;
};

class ParameterHost {
 public:
  virtual ~ParameterHost() = default;

  // Load `<node_name>: ros__parameters:` from a YAML file into the flat map.
  void loadParametersFromYaml(const std::string& path, const std::string& node_name);

  // Multi-file variant: deep-merges the `ros__parameters:` subtrees of every
  // file in order; keys from later files win on conflict. Lets the pipeline
  // config split into topic files with a master file on top.
  void loadParametersFromYaml(const std::vector<std::string>& paths,
                              const std::string& node_name);

  // declare_parameter(name, default): returns YAML value if present, else the
  // default (and registers it). Overloads cover the types the node uses.
  bool declare_parameter(const std::string& name, bool def);
  int64_t declare_parameter(const std::string& name, int def);
  double declare_parameter(const std::string& name, double def);
  std::string declare_parameter(const std::string& name, const char* def);
  std::string declare_parameter(const std::string& name, const std::string& def);
  std::vector<std::string> declare_parameter(const std::string& name,
                                             const std::vector<std::string>& def);
  std::vector<int64_t> declare_parameter(const std::string& name,
                                         const std::vector<int64_t>& def);
  std::vector<double> declare_parameter(const std::string& name,
                                        const std::vector<double>& def);

  ParamValue get_parameter(const std::string& name) const;
  bool has_parameter(const std::string& name) const;

  // True if the key was present in the loaded YAML (the ros-free analogue of a
  // ROS parameter override). Used by the lifted readCompat/readUnified helpers.
  bool hasParameterOverride(const std::string& name) const;

  // Logging API used by the lifted methods.
  rclcpp::Logger get_logger() const { return rclcpp::Logger("gimbal_pipeline"); }

 private:
  std::map<std::string, ParamValue> params_;
  std::shared_ptr<YAML::Node> raw_yaml_;  // ros__parameters subtree, if loaded
};

}  // namespace hfut::pipeline

#endif  // HFUT_PIPELINE_PARAMETER_HOST_HPP
