#ifndef AUTO_BUFF_V2__RUNE_DETECTOR_HPP
#define AUTO_BUFF_V2__RUNE_DETECTOR_HPP

#include "../rune.hpp"

namespace auto_buff_v2
{
class RuneDetector
{
public:
  virtual ~RuneDetector() = default;
  virtual void set_enemy_red(bool enemy_red) = 0;
  virtual RuneElements detect(const cv::Mat & image) = 0;
};
}  // namespace auto_buff_v2

#endif  // AUTO_BUFF_V2__RUNE_DETECTOR_HPP
