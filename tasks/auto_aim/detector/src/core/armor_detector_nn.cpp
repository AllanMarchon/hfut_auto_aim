#include "armor_detector_nn/core/armor_detector_nn.hpp"

#include <algorithm>
#include <stdexcept>

#include "rm_utils/logger/log.hpp"

#include "armor_detector_nn/backend/inference_backend_factory.hpp"
#include "armor_detector_nn/geometry/nms.hpp"
#include "armor_detector_nn/postprocess/decode_strategy_factory.hpp"
#include "armor_detector_nn/postprocess/detection_quality_filter.hpp"
#include "armor_detector_nn/core/corner_refine/roi_pca_corner_refiner.hpp"
#include "armor_detector_nn/core/tracker/internal_iou_tracker_strategy.hpp"

namespace fyt::auto_aim {

ArmorDetectorNN::ArmorDetectorNN(const DetectorConfig& config)
  : config_(config)
{
}

ArmorDetectorNN::~ArmorDetectorNN() = default;

bool ArmorDetectorNN::initialize() {
  // 1. Backend
  backend_ = InferenceBackendFactory::create(config_.backend);
  if (!backend_) {
    FYT_ERROR("armor_detector_nn", "Failed to create inference backend");
    return false;
  }

  try {
    backend_->warmup(config_.backend.warmup_iterations);
  } catch (const std::exception& e) {
    FYT_WARN("armor_detector_nn", "Warmup failed (non-fatal): {}", e.what());
  }

  // 2. Preprocessor
  preprocessor_ = std::make_unique<Preprocessor>(config_.preprocess);

  // 3. Decode strategy
  decode_strategy_ = DecodeStrategyFactory::create(config_.postprocess);
  if (!decode_strategy_) {
    FYT_ERROR("armor_detector_nn",
              "Unknown decode strategy: {}", config_.postprocess.strategy.c_str());
    return false;
  }

  // 4. Label map
  if (!config_.label_map.path.empty()) {
    label_map_ = std::make_unique<LabelMap>();
    try {
      label_map_->load(config_.label_map.path);
    } catch (const std::exception& e) {
      FYT_ERROR("armor_detector_nn", "Failed to load label map: {}", e.what());
      return false;
    }
  } else {
    FYT_ERROR("armor_detector_nn", "Label map path is empty");
    return false;
  }

  if (config_.tracker.strategy == "internal_iou") {
    tracker_ = std::make_unique<InternalIoUTrackerStrategy>(config_.tracker);
  } else if (!config_.tracker.strategy.empty() &&
             config_.tracker.strategy != "none") {
    FYT_WARN("armor_detector_nn", "Unknown tracker strategy '{}'; tracking disabled",
             config_.tracker.strategy.c_str());
  }

  // Optional NN-keypoint refinement: re-fit light bars in a local ROI around
  // each detection and replace the keypoints when the classical fit passes
  // geometric validation (sp_vision-style hybrid). Failure keeps the NN
  // keypoints unchanged, so this can only help or no-op.
  if (config_.corner_refine.enabled) {
    corner_refiner_ =
        std::make_unique<RoiPcaCornerRefiner>(config_.corner_refine);
    FYT_INFO("armor_detector_nn",
             "Corner refinement enabled (method={})",
             config_.corner_refine.method.c_str());
  }

  initialized_ = true;
  FYT_INFO("armor_detector_nn", "Core pipeline initialized with {} backend",
           backend_->info().backend_name.c_str());
  return true;
}

std::vector<FrameDetections> ArmorDetectorNN::detectBatch(
    const std::vector<cv::Mat>& images,
    const std::vector<std_msgs::msg::Header>& headers)
{
  std::vector<FrameDetections> results;

  if (!initialized_) {
    FYT_ERROR("armor_detector_nn", "detectBatch called before initialize()");
    return results;
  }

  for (size_t i = 0; i < images.size(); ++i) {
    FrameDetections fd;
    fd.header = headers[i];

    last_profile_ = ProfilerEntry{};

    // Preprocess
    {
      ScopedTimer t(last_profile_.preprocess_ms);

      auto t_start = std::chrono::steady_clock::now();

      auto pre = preprocessor_->process(images[i]);

      auto t_preprocess_end = std::chrono::steady_clock::now();
      FYT_DEBUG("armor_detector_nn", "Preprocessing completed in {:.2f} ms",
                std::chrono::duration<double, std::milli>(t_preprocess_end - t_start).count());

      // Infer
      {
        ScopedTimer t2(last_profile_.infer_ms);
        std::vector<TensorOutput> outputs;
        try {
          outputs = backend_->infer(pre.tensor);
        } catch (const std::exception& e) {
          FYT_ERROR("armor_detector_nn", "Inference failed: {}", e.what());
          results.push_back(std::move(fd));
          continue;
        }

        auto infer_end = std::chrono::steady_clock::now();
        FYT_DEBUG("armor_detector_nn", "Inference completed in {:.2f} ms",
                  std::chrono::duration<double, std::milli>(infer_end - t_preprocess_end).count());

        // Decode
        std::vector<RawDetection> raw;
        {
          ScopedTimer t3(last_profile_.decode_ms);
          raw = decode_strategy_->decode(outputs, pre.image_meta, config_.postprocess);
        }
        auto t_detect_end = std::chrono::steady_clock::now();
        FYT_DEBUG("armor_detector_nn", "Decoding completed in {:.2f} ms, {} raw detections",
                  std::chrono::duration<double, std::milli>(t_detect_end - infer_end).count(),
                  raw.size());

        last_profile_.raw_candidates = static_cast<int>(raw.size());
        last_profile_.after_conf = last_profile_.raw_candidates;

        // NMS
        std::vector<RawDetection> nms_result;
        {
          ScopedTimer t4(last_profile_.nms_ms);
          nms_result = applyNMS(raw, config_.postprocess.nms_threshold,
                                config_.postprocess.class_agnostic_nms);
        }
        auto t_nms_end = std::chrono::steady_clock::now();
        FYT_DEBUG("armor_detector_nn", "NMS completed in {:.2f} ms, {} detections remaining",
                  std::chrono::duration<double, std::milli>(t_nms_end - t_detect_end).count(),
                  nms_result.size());
          for (const auto& rd : nms_result) {
            FYT_DEBUG("armor_detector_nn", "NMS candidate: class_id={} confidence={}",
                      rd.class_id, rd.confidence);
          }
        last_profile_.after_nms = static_cast<int>(nms_result.size());

        // Label map
        for (const auto& rd : nms_result) {
          const auto* entry = label_map_->lookup(rd.class_id);
          if (!entry) continue;

          ArmorDetection ad;
          ad.model_label    = entry->model_label;
          ad.publish_number = entry->publish_number;
          ad.publish_type   = entry->publish_type;
          ad.color          = entry->color;
          ad.confidence     = rd.confidence;
          ad.bbox           = rd.bbox;
          ad.keypoints      = rd.keypoints;
          ad.center =
            (ad.keypoints[0] + ad.keypoints[1] + ad.keypoints[2] + ad.keypoints[3]) * 0.25F;
          ad.stamp = rclcpp::Time(
            headers[i].stamp.sec, headers[i].stamp.nanosec, RCL_ROS_TIME);
          if (config_.postprocess.keypoint_auto_reorder) {
            fd.detections.push_back(canonicalizeArmorDetectionGeometry(ad));
          } else {
            fd.detections.push_back(std::move(ad));
          }
        }

        // Color filter
        if (config_.runtime.color_filter_source == ColorFilterSource::MODEL) {
          fd.detections = label_map_->filterByColor(fd.detections, target_color_);
        }

        if (config_.quality_filter.enabled) {
          const size_t before_quality_filter = fd.detections.size();
          fd.detections = filterByGeometryQuality(fd.detections, config_.quality_filter);
          fd.detections = suppressDuplicateDetections(fd.detections, config_.quality_filter);
          FYT_DEBUG("armor_detector_nn",
                    "Quality filter completed: {} -> {} detections",
                    before_quality_filter, fd.detections.size());
        }

        // Clamp to max_detections
        if (static_cast<int>(fd.detections.size()) > config_.postprocess.max_detections) {
          std::partial_sort(
            fd.detections.begin(),
            fd.detections.begin() + config_.postprocess.max_detections,
            fd.detections.end(),
            [](const ArmorDetection& a, const ArmorDetection& b) {
              return a.confidence > b.confidence;
            });
          fd.detections.resize(config_.postprocess.max_detections);
        }

        if (tracker_) {
          const rclcpp::Time stamp(
            headers[i].stamp.sec, headers[i].stamp.nanosec, RCL_ROS_TIME);
          auto tracked = tracker_->associate(fd.detections, stamp);
          fd.detections.clear();
          fd.detections.reserve(tracked.size());
          for (auto& detection : tracked) {
            fd.detections.push_back(std::move(detection.det));
          }
        }

        // NN-keypoint refinement: re-fit light bars in a local ROI around
        // each detection (sp_vision-style hybrid). Only replace keypoints
        // when the classical fit passes geometric validation; otherwise the
        // NN keypoints are kept, so environments where light bars are not
        // resolvable (over/under-exposure, blur, occlusion) fall back
        // cleanly with zero behavior change.
        if (corner_refiner_) {
          auto quad_extent = [](const std::array<cv::Point2f, 4>& kp) {
            const double w = 0.5 * (cv::norm(kp[1] - kp[2]) +
                                    cv::norm(kp[0] - kp[3]));
            const double h = 0.5 * (cv::norm(kp[0] - kp[1]) +
                                    cv::norm(kp[2] - kp[3]));
            return w + h;
          };
          for (auto& det : fd.detections) {
            const double nn_extent = quad_extent(det.keypoints);
            const auto refined = corner_refiner_->refine(images[i], det);
            if (refined.ok) {
              const double ref_extent = quad_extent(refined.refined_keypoints);
              // Match the refined quad's extent to the NN's trained plate
              // extent (EMA per 2D track): keeps the refit's stable shape,
              // removes the light-bar-inset depth bias. See header comment.
              double scale = 1.0;
              if (ref_extent > 1.0 && nn_extent > 1.0) {
                const double inst = nn_extent / ref_extent;
                auto it = extent_ratio_ema_.find(det.track_id);
                if (it == extent_ratio_ema_.end()) {
                  it = extent_ratio_ema_.emplace(det.track_id, inst).first;
                } else {
                  it->second = 0.9 * it->second + 0.1 * inst;
                }
                scale = it->second;
              }
              const cv::Point2f rc =
                (refined.refined_keypoints[0] + refined.refined_keypoints[1] +
                 refined.refined_keypoints[2] + refined.refined_keypoints[3]) *
                0.25F;
              for (int k = 0; k < 4; ++k) {
                det.keypoints[k] =
                  rc + (refined.refined_keypoints[k] - rc) *
                           static_cast<float>(scale);
              }
              det.center =
                (det.keypoints[0] + det.keypoints[1] + det.keypoints[2] +
                 det.keypoints[3]) * 0.25F;
              ++corner_refine_ok_;
            } else {
              ++corner_refine_fail_;
            }
          }
          if ((corner_refine_ok_ + corner_refine_fail_) % 200 == 0) {
            FYT_DEBUG("armor_detector_nn",
                      "corner refine acceptance: {}/{}",
                      corner_refine_ok_,
                      corner_refine_ok_ + corner_refine_fail_);
          }
        }

        auto t_pose_end = std::chrono::steady_clock::now();
        FYT_DEBUG("armor_detector_nn", "Postprocess completed in {:.2f} ms, {} detections published",
                  std::chrono::duration<double, std::milli>(t_pose_end - t_nms_end).count(),
                  fd.detections.size());

        last_profile_.published = static_cast<int>(fd.detections.size());
      }
    }

    results.push_back(std::move(fd));
  }

  FYT_DEBUG("armor_detector_nn", "Batch detection completed: {} frames processed",
            results.size());
  return results;
}

BackendInfo ArmorDetectorNN::backendInfo() const {
  if (backend_) {
    return backend_->info();
  }
  BackendInfo bi;
  bi.backend_name = "none";
  return bi;
}

}  // namespace fyt::auto_aim
