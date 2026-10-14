#ifndef AUTO_BUFF__BUFF_DETECTOR_HPP
#define AUTO_BUFF__BUFF_DETECTOR_HPP

#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "buff_type.hpp"
#include "szu_rune_detector.hpp"
#include "tools/img_tools.hpp"
#include "yolo11_buff.hpp"
const int LOSE_MAX = 20;  // 丢失的阙值
namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect(cv::Mat & bgr_img, PowerRune_type rune_type);

  // 仅在串口实际放行一发后调用，建立短暂的防重复开火相位记录。
  void notify_fire(PowerRune_type rune_type);

  std::optional<PowerRune> detect_24(cv::Mat & bgr_img);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);

  std::optional<PowerRune> detect_debug(cv::Mat & bgr_img, cv::Point2f v);

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img);

  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);

  void handle_lose();

  std::optional<PowerRune> detect_sp25(cv::Mat & bgr_img, bool multi_candidate);

  std::optional<PowerRune> detect_szu(cv::Mat & bgr_img, PowerRune_type rune_type);

  FanBlade_type classify_szu_blade(int class_id, PowerRune_type rune_type) const;

  struct TargetGuardProfile
  {
    bool enabled = true;
    double cooldown_s = 1.0;
    double phase_tolerance_rad = 28.0 * CV_PI / 180.0;
  };

  struct TargetGuardState
  {
    struct FiredLeaf
    {
      double phase_rad = 0.0;
      double phase_velocity_rad_s = 0.0;
      std::chrono::steady_clock::time_point fired_time{};
    };

    bool phase_valid = false;
    double phase_rad = 0.0;
    double phase_velocity_rad_s = 0.0;
    std::chrono::steady_clock::time_point phase_time{};
    std::deque<FiredLeaf> fired_leaves;
  };

  static std::size_t guard_index(PowerRune_type rune_type);
  static double wrap_phase(double phase_rad);
  bool blocked_after_fire(
    PowerRune_type rune_type, double phase_rad,
    std::chrono::steady_clock::time_point now) const;
  void update_target_phase(
    PowerRune_type rune_type, double phase_rad,
    std::chrono::steady_clock::time_point now);

  void log_szu_debug(
    const char * stage, const SzuRuneDetector::DebugStats & stats, std::size_t raw_count,
    std::size_t target_count, std::size_t other_count);

  std::string config_path_;
  std::unique_ptr<YOLO11_BUFF> sp25_detector_;
  std::unique_ptr<SzuRuneDetector> szu_detector_;
  RuneDetectorBackend backend_{SP25};
  int szu_target_class_id_{0};
  int szu_small_target_class_id_{0};
  int szu_big_target_class_id_{0};
  bool szu_debug_log_{false};
  int szu_debug_log_every_n_{60};
  int szu_debug_frame_{0};
  std::array<TargetGuardProfile, 2> target_guard_profiles_{};
  std::array<TargetGuardState, 2> target_guard_states_{};
  // R 点几何检查默认只做诊断，不直接丢弃整帧，避免把检测保护变成检测瓶颈。
  bool szu_reject_r_geometry_{false};
  double szu_r_center_max_spread_px_{50.0};
  double szu_min_target_radius_px_{20.0};
  Track_status status_;
  int lose_;  // 丢失的次数
  std::optional<PowerRune> last_powerrune_ = std::nullopt;
};
}  // namespace auto_buff
#endif  // AUTO_BUFF__BUFF_DETECTOR_HPP
