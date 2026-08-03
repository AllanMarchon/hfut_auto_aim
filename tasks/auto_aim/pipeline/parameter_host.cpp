#include "parameter_host.hpp"

namespace hfut::pipeline {

namespace {
// Recursive map merge: maps merge key-wise, anything else is replaced by the
// incoming (later-file) value.
void deepMerge(YAML::Node& dst, const YAML::Node& src) {
  if (!src.IsMap()) return;
  if (!dst.IsMap()) dst = YAML::Node(YAML::NodeType::Map);
  for (const auto& kv : src) {
    const std::string key = kv.first.as<std::string>();
    const YAML::Node& value = kv.second;
    if (value.IsMap() && dst[key] && dst[key].IsMap()) {
      YAML::Node child = dst[key];
      deepMerge(child, value);
      dst[key] = child;
    } else {
      dst[key] = YAML::Clone(value);
    }
  }
}
}  // namespace

void ParameterHost::loadParametersFromYaml(const std::string& path,
                                           const std::string& node_name) {
  loadParametersFromYaml(std::vector<std::string>{path}, node_name);
}

void ParameterHost::loadParametersFromYaml(const std::vector<std::string>& paths,
                                           const std::string& node_name) {
  YAML::Node merged;
  for (const auto& path : paths) {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node params = root[node_name] ? root[node_name]["ros__parameters"] : YAML::Node();
    if (!params) params = root["/**"] ? root["/**"]["ros__parameters"] : YAML::Node();
    if (!params) continue;
    if (!merged) merged = YAML::Node(YAML::NodeType::Map);
    deepMerge(merged, params);
  }
  if (!merged) return;
  // Store the merged ros__parameters subtree; declare_parameter re-reads each
  // key using the declared default's type for correct coercion.
  raw_yaml_ = std::make_shared<YAML::Node>(merged);
}

namespace {
// Walk dotted key into a flattened node.
YAML::Node lookup(const YAML::Node& root, const std::string& dotted) {
  YAML::Node cur = YAML::Clone(root);
  size_t start = 0;
  while (start < dotted.size()) {
    size_t dot = dotted.find('.', start);
    std::string key = dotted.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
    if (!cur || !cur.IsMap() || !cur[key]) return YAML::Node(YAML::NodeType::Undefined);
    cur = cur[key];
    if (dot == std::string::npos) break;
    start = dot + 1;
  }
  return cur;
}

// Resolve a declared parameter key against the raw yaml. The canonical
// "vehicle_tracker.*" section falls back to the legacy "norm4_v3.*" section so
// shared configs written for the experimental tracker naming keep working.
YAML::Node lookupWithLegacyAliases(const YAML::Node& root,
                                   const std::string& dotted) {
  YAML::Node n = lookup(root, dotted);
  if (n && n.IsDefined() && !n.IsNull()) return n;
  static constexpr std::string_view kCanonicalPrefix = "vehicle_tracker.";
  static constexpr std::string_view kLegacyPrefix = "norm4_v3.";
  if (dotted.compare(0, kCanonicalPrefix.size(), kCanonicalPrefix) == 0) {
    return lookup(
        root, std::string(kLegacyPrefix) + dotted.substr(kCanonicalPrefix.size()));
  }
  return n;
}
}  // namespace

#define HFUT_DECLARE_IMPL(TYPE, COERCE_EXPR)                                  \
  do {                                                                        \
    if (raw_yaml_) {                                                          \
      YAML::Node n = lookupWithLegacyAliases(*raw_yaml_, name);                               \
      if (n && n.IsDefined() && !n.IsNull()) {                               \
        TYPE v = (COERCE_EXPR);                                              \
        params_[name] = ParamValue(ParamValue::Storage{v});                 \
        return v;                                                            \
      }                                                                       \
    }                                                                         \
    params_[name] = ParamValue(ParamValue::Storage{def});                    \
    return def;                                                               \
  } while (0)

bool ParameterHost::declare_parameter(const std::string& name, bool def) {
  HFUT_DECLARE_IMPL(bool, n.as<bool>());
}
int64_t ParameterHost::declare_parameter(const std::string& name, int def) {
  int64_t d = def; (void)d;
  if (raw_yaml_) {
    YAML::Node n = lookupWithLegacyAliases(*raw_yaml_, name);
    if (n && n.IsDefined() && !n.IsNull()) {
      int64_t v = n.as<int64_t>();
      params_[name] = ParamValue(ParamValue::Storage{v});
      return v;
    }
  }
  int64_t v = def;
  params_[name] = ParamValue(ParamValue::Storage{v});
  return v;
}
double ParameterHost::declare_parameter(const std::string& name, double def) {
  HFUT_DECLARE_IMPL(double, n.as<double>());
}
std::string ParameterHost::declare_parameter(const std::string& name, const char* def) {
  return declare_parameter(name, std::string(def));
}
std::string ParameterHost::declare_parameter(const std::string& name, const std::string& def) {
  HFUT_DECLARE_IMPL(std::string, n.as<std::string>());
}

#undef HFUT_DECLARE_IMPL

std::vector<std::string> ParameterHost::declare_parameter(
    const std::string& name, const std::vector<std::string>& def) {
  if (raw_yaml_) {
    YAML::Node n = lookupWithLegacyAliases(*raw_yaml_, name);
    if (n && n.IsDefined() && n.IsSequence()) {
      std::vector<std::string> v;
      for (const auto& e : n) v.push_back(e.as<std::string>());
      params_[name] = ParamValue(ParamValue::Storage{v});
      return v;
    }
  }
  params_[name] = ParamValue(ParamValue::Storage{def});
  return def;
}
std::vector<int64_t> ParameterHost::declare_parameter(
    const std::string& name, const std::vector<int64_t>& def) {
  if (raw_yaml_) {
    YAML::Node n = lookupWithLegacyAliases(*raw_yaml_, name);
    if (n && n.IsDefined() && n.IsSequence()) {
      std::vector<int64_t> v;
      for (const auto& e : n) v.push_back(e.as<int64_t>());
      params_[name] = ParamValue(ParamValue::Storage{v});
      return v;
    }
  }
  params_[name] = ParamValue(ParamValue::Storage{def});
  return def;
}
std::vector<double> ParameterHost::declare_parameter(
    const std::string& name, const std::vector<double>& def) {
  if (raw_yaml_) {
    YAML::Node n = lookupWithLegacyAliases(*raw_yaml_, name);
    if (n && n.IsDefined() && n.IsSequence()) {
      std::vector<double> v;
      for (const auto& e : n) v.push_back(e.as<double>());
      params_[name] = ParamValue(ParamValue::Storage{v});
      return v;
    }
  }
  params_[name] = ParamValue(ParamValue::Storage{def});
  return def;
}

ParamValue ParameterHost::get_parameter(const std::string& name) const {
  auto it = params_.find(name);
  if (it == params_.end()) {
    throw std::runtime_error("get_parameter: undeclared parameter '" + name + "'");
  }
  return it->second;
}

bool ParameterHost::has_parameter(const std::string& name) const {
  return params_.find(name) != params_.end();
}

bool ParameterHost::hasParameterOverride(const std::string& name) const {
  if (!raw_yaml_) return false;
  YAML::Node n = lookupWithLegacyAliases(*raw_yaml_, name);
  return n && n.IsDefined() && !n.IsNull();
}

}  // namespace hfut::pipeline
