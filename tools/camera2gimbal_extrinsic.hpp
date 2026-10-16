#ifndef TOOLS__CAMERA2GIMBAL_EXTRINSIC_HPP
#define TOOLS__CAMERA2GIMBAL_EXTRINSIC_HPP

#include <Eigen/Dense>
#include <yaml-cpp/node/node.h>

namespace tools
{
struct Camera2GimbalExtrinsic
{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
};

// camera2gimbal_mode 默认为 matrix，可选 xyz_ypr。xyz 为云台系位置；ypr 为相对相机光轴默认
// 朝向的机械安装偏角（单位：弧度，旋转顺序：ZYX）。
Camera2GimbalExtrinsic load_camera2gimbal_extrinsic(const YAML::Node & yaml);
}  // namespace tools

#endif  // TOOLS__CAMERA2GIMBAL_EXTRINSIC_HPP
