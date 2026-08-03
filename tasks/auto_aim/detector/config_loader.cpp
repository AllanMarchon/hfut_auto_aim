#include "config_loader.hpp"

namespace hfut::detector {

using namespace fyt::auto_aim;

namespace {

// Overlay helper: assign node[key] into `out` only if present.
template <typename T>
void opt(const YAML::Node& node, const char* key, T& out) {
  if (node && node[key]) out = node[key].as<T>();
}

// float needs an explicit bridge (YAML reads as double well enough).
void optf(const YAML::Node& node, const char* key, float& out) {
  if (node && node[key]) out = node[key].as<float>();
}

BackendType backendTypeFromStr(const std::string& s) {
  if (s == "openvino") return BackendType::OPENVINO;
  if (s == "tensorrt") return BackendType::TENSORRT;
  return BackendType::ONNX_RUNTIME;
}
Precision precisionFromStr(const std::string& s) {
  if (s == "fp16") return Precision::FP16;
  if (s == "int8") return Precision::INT8;
  return Precision::FP32;
}

void loadBackend(const YAML::Node& n, BackendConfig& c) {
  if (!n) return;
  std::string type, prec, fallback;
  opt(n, "type", type);
  opt(n, "precision", prec);
  opt(n, "fallback_type", fallback);
  if (!type.empty()) c.type = backendTypeFromStr(type);
  if (!prec.empty()) c.precision = precisionFromStr(prec);
  if (!fallback.empty()) c.fallback_type = backendTypeFromStr(fallback);
  opt(n, "device", c.device);
  opt(n, "model_path", c.model_path);
  opt(n, "engine_path", c.engine_path);
  opt(n, "openvino_model_xml", c.openvino_xml_path);
  opt(n, "openvino_model_bin", c.openvino_bin_path);
  opt(n, "input_name", c.input_name);
  opt(n, "output_names", c.output_names);
  opt(n, "warmup_iterations", c.warmup_iterations);
  opt(n, "num_threads", c.num_threads);
  opt(n, "use_pinned_memory", c.use_pinned_memory);
  opt(n, "allow_fallback", c.allow_fallback);
}

void loadPreprocess(const YAML::Node& n, PreprocessConfig& c) {
  if (!n) return;
  opt(n, "input_width", c.input_width);
  opt(n, "input_height", c.input_height);
  opt(n, "input_color", c.input_color);
  opt(n, "normalize", c.normalize);
  opt(n, "mean", c.mean);
  opt(n, "std", c.std);
  optf(n, "pad_value", c.pad_value);
}

void loadPostprocess(const YAML::Node& n, PostprocessConfig& c) {
  if (!n) return;
  opt(n, "strategy", c.strategy);
  opt(n, "num_classes", c.num_classes);
  opt(n, "num_keypoints", c.num_keypoints);
  opt(n, "keypoint_dims", c.keypoint_dims);
  opt(n, "bbox_offset", c.bbox_offset);
  opt(n, "class_offset", c.class_offset);
  opt(n, "keypoint_offset", c.keypoint_offset);
  optf(n, "conf_threshold", c.conf_threshold);
  optf(n, "nms_threshold", c.nms_threshold);
  opt(n, "max_detections", c.max_detections);
  opt(n, "class_agnostic_nms", c.class_agnostic_nms);
  opt(n, "keypoint_remap", c.keypoint_remap);
  opt(n, "keypoint_auto_reorder", c.keypoint_auto_reorder);
}

void loadQualityFilter(const YAML::Node& n, QualityFilterConfig& c) {
  if (!n) return;
  opt(n, "enabled", c.enabled);
  opt(n, "min_armor_ratio", c.min_armor_ratio);
  opt(n, "max_armor_ratio", c.max_armor_ratio);
  opt(n, "max_side_ratio", c.max_side_ratio);
  opt(n, "max_rectangular_error_deg", c.max_rectangular_error_deg);
  opt(n, "min_lightbar_length_px", c.min_lightbar_length_px);
  opt(n, "min_area_px", c.min_area_px);
  opt(n, "deduplicate_enabled", c.deduplicate_enabled);
  opt(n, "duplicate_iou_threshold", c.duplicate_iou_threshold);
  opt(n, "duplicate_keypoint_mean_dist_px", c.duplicate_keypoint_mean_dist_px);
}

void loadPose(const YAML::Node& n, PoseConfig& c) {
  if (!n) return;
  opt(n, "pnp_method", c.pnp_method);
  opt(n, "small_armor_width", c.small_armor_width);
  opt(n, "small_armor_height", c.small_armor_height);
  opt(n, "large_armor_width", c.large_armor_width);
  opt(n, "large_armor_height", c.large_armor_height);
  opt(n, "force_pnp_rotate_180", c.force_pnp_rotate_180);

  if (const YAML::Node r = n["refiner"]) opt(r, "mode", c.refiner.mode);

  if (const YAML::Node sy = n["single_yaw"]) {
    opt(sy, "max_iterations", c.single_yaw.max_iterations);
    opt(sy, "huber_delta", c.single_yaw.huber_delta);
    opt(sy, "pitch_deg_default", c.single_yaw.pitch_deg_default);
    opt(sy, "roll_deg_default", c.single_yaw.roll_deg_default);
    opt(sy, "outpost_pitch_sign", c.single_yaw.outpost_pitch_sign);
  }

  if (const YAML::Node s = n["sliding"]) {
    opt(s, "window_size", c.sliding.window_size);
    opt(s, "min_frames", c.sliding.min_frames);
    opt(s, "max_time_span_ms", c.sliding.max_time_span_ms);
    opt(s, "max_solver_time_ms", c.sliding.max_solver_time_ms);
    opt(s, "max_opt_iters", c.sliding.max_opt_iters);
    opt(s, "sigma_prior_xy", c.sliding.sigma_prior_xy);
    opt(s, "sigma_prior_z", c.sliding.sigma_prior_z);
    opt(s, "sigma_prior_yaw", c.sliding.sigma_prior_yaw);
    opt(s, "sigma_smooth_xy", c.sliding.sigma_smooth_xy);
    opt(s, "sigma_smooth_z", c.sliding.sigma_smooth_z);
    opt(s, "sigma_smooth_yaw", c.sliding.sigma_smooth_yaw);
    opt(s, "sigma_kp_min", c.sliding.sigma_kp_min);
    opt(s, "sigma_kp_scale", c.sliding.sigma_kp_scale);
    opt(s, "huber_delta", c.sliding.huber_delta);
  }

  if (const YAML::Node g = n["gate"]) {
    opt(g, "max_raw_reproj_error", c.gate.max_raw_reproj_error);
    opt(g, "max_reproj_error", c.gate.max_reproj_error);
    opt(g, "max_pose_delta_m", c.gate.max_pose_delta_m);
    opt(g, "max_yaw_delta_deg", c.gate.max_yaw_delta_deg);
    opt(g, "require_finite", c.gate.require_finite);
  }

  if (const YAML::Node d = n["depth_correction"]) {
    opt(d, "enabled", c.depth_correction.enabled);
    opt(d, "min_depth_delta_m", c.depth_correction.min_depth_delta_m);
    opt(d, "blend_alpha", c.depth_correction.blend_alpha);
    opt(d, "max_correction_m", c.depth_correction.max_correction_m);
    opt(d, "max_scale", c.depth_correction.max_scale);
    opt(d, "min_lightbar_length_px", c.depth_correction.min_lightbar_length_px);
  }
}

void loadTracker(const YAML::Node& n, TrackerConfig& c) {
  if (!n) return;
  opt(n, "strategy", c.strategy);
  opt(n, "iou_threshold", c.iou_threshold);
  opt(n, "max_missed", c.max_missed);
  opt(n, "min_hits", c.min_hits);
  opt(n, "max_center_dist_px", c.max_center_dist_px);
}

void loadRuntime(const YAML::Node& n, RuntimeConfig& c) {
  if (!n) return;
  std::string cfs, copy_policy;
  opt(n, "color_filter_source", cfs);
  opt(n, "copy_policy", copy_policy);
  if (cfs == "disabled") c.color_filter_source = ColorFilterSource::DISABLED;
  else if (cfs == "model") c.color_filter_source = ColorFilterSource::MODEL;
  if (copy_policy == "never_copy") c.copy_policy = CopyPolicy::NEVER_COPY;
  else if (copy_policy == "always_copy") c.copy_policy = CopyPolicy::ALWAYS_COPY;
  else if (copy_policy == "copy_on_write_debug") c.copy_policy = CopyPolicy::COPY_ON_WRITE_DEBUG;
  opt(n, "publish_empty", c.publish_empty);
  opt(n, "profile", c.profile);
}

void loadCornerRefine(const YAML::Node& n, CornerRefineConfig& c) {
  if (!n) return;
  opt(n, "enabled", c.enabled);
  opt(n, "method", c.method);
  opt(n, "time_budget_ms", c.time_budget_ms);
  opt(n, "roi_expand_ratio", c.roi_expand_ratio);
  opt(n, "min_bright_points", c.min_bright_points);
  opt(n, "pca_stability_threshold", c.pca_stability_threshold);
  opt(n, "max_aspect_ratio", c.max_aspect_ratio);
  opt(n, "min_aspect_ratio", c.min_aspect_ratio);
  opt(n, "max_corner_shift_px", c.max_corner_shift_px);
  opt(n, "max_mean_corner_shift_px", c.max_mean_corner_shift_px);
  opt(n, "max_refined_center_shift_px", c.max_refined_center_shift_px);
  opt(n, "min_refine_quality", c.min_refine_quality);
  opt(n, "preserve_perspective", c.preserve_perspective);
  opt(n, "max_edge_angle_delta_deg", c.max_edge_angle_delta_deg);
  opt(n, "max_area_ratio_delta", c.max_area_ratio_delta);
  opt(n, "max_length_ratio_delta", c.max_length_ratio_delta);
  opt(n, "full_roi_expand_ratio", c.full_roi_expand_ratio);
  opt(n, "binary_threshold", c.binary_threshold);
  opt(n, "min_contour_area_px", c.min_contour_area_px);
  opt(n, "min_lightbar_length_px", c.min_lightbar_length_px);
  opt(n, "min_lightbar_ratio", c.min_lightbar_ratio);
  opt(n, "max_lightbar_ratio", c.max_lightbar_ratio);
  opt(n, "max_lightbar_angle_error_deg", c.max_lightbar_angle_error_deg);
  opt(n, "max_rectangular_error_deg", c.max_rectangular_error_deg);
  opt(n, "max_side_ratio", c.max_side_ratio);
  opt(n, "max_lightbar_match_error_px", c.max_lightbar_match_error_px);
  opt(n, "max_pair_center_shift_px", c.max_pair_center_shift_px);
}

}  // namespace

fyt::auto_aim::DetectorConfig loadDetectorConfig(const YAML::Node& root) {
  // Accept either a bare config or one nested under `detector:`.
  YAML::Node n = (root && root["detector"]) ? root["detector"] : root;
  DetectorConfig c;  // struct defaults
  if (!n) return c;

  opt(n, "target_frame", c.target_frame);
  loadBackend(n["backend"], c.backend);
  loadPreprocess(n["preprocess"], c.preprocess);
  loadPostprocess(n["postprocess"], c.postprocess);
  if (const YAML::Node lm = n["label_map"]) opt(lm, "path", c.label_map.path);
  loadQualityFilter(n["quality_filter"], c.quality_filter);
  loadPose(n["pose"], c.pose);
  loadTracker(n["tracker"], c.tracker);
  loadRuntime(n["runtime"], c.runtime);
  loadCornerRefine(n["corner_refine"], c.corner_refine);
  return c;
}

fyt::auto_aim::DetectorConfig loadDetectorConfigFile(const std::string& path) {
  return loadDetectorConfig(YAML::LoadFile(path));
}

}  // namespace hfut::detector
