#ifndef AUTO_BUFF__RUNE_DETECTOR_HPP
#define AUTO_BUFF__RUNE_DETECTOR_HPP

#include "../rune.hpp"

namespace auto_buff
{
class RuneDetector
{
public:
  virtual ~RuneDetector() = default;
  virtual void set_enemy_red(bool enemy_red) = 0;
  virtual RuneElements detect(const cv::Mat & image) = 0;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_DETECTOR_HPP
