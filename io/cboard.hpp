#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <string>
#include <vector>

#include "io/command.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};

const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};

const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

}  // namespace io

#endif  // IO__CBOARD_HPP
