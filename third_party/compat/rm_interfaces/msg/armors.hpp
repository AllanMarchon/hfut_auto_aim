// ROS-free shim for rm_interfaces/msg/Armors.
#ifndef HFUT_COMPAT_RM_INTERFACES_ARMORS_HPP
#define HFUT_COMPAT_RM_INTERFACES_ARMORS_HPP

#include <memory>
#include <vector>

#include <std_msgs/msg/header.hpp>

#include "rm_interfaces/msg/armor.hpp"

namespace rm_interfaces {
namespace msg {

struct Armors {
  using SharedPtr = std::shared_ptr<Armors>;
  using ConstSharedPtr = std::shared_ptr<const Armors>;

  std_msgs::msg::Header header;
  std::vector<Armor> armors;
};

}  // namespace msg
}  // namespace rm_interfaces

#endif  // HFUT_COMPAT_RM_INTERFACES_ARMORS_HPP
