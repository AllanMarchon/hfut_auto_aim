#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

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

  std::string config_path_;
  std::unique_ptr<YOLO11_BUFF> sp25_detector_;
  std::unique_ptr<SzuRuneDetector> szu_detector_;
  RuneDetectorBackend backend_{SP25};
  int szu_target_class_id_{0};
  int szu_small_target_class_id_{0};
  int szu_big_target_class_id_{0};
  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;
  std::optional<PowerRune> last_powerrune_ = std::nullopt;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP
