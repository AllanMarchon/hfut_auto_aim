#ifndef AUTO_BUFF_V2__SP_RUNE_CONFIG_HPP
#define AUTO_BUFF_V2__SP_RUNE_CONFIG_HPP

#include <yaml-cpp/yaml.h>

namespace auto_buff_v2
{
struct SPRuneConfig
{
  int max_lost_frames = 6;
  double small_speed_tolerance = 0.17453292519943295;
  double small_yaw_acceleration_noise = 0.001;
  double big_yaw_acceleration_noise = 0.9;
  double big_rotation_angle_noise = 0.09;
  double big_rotation_speed_noise = 0.5;
  double big_phase_noise = 1.0;
  double observation_yaw = 0.01;
  double observation_pitch = 0.01;
  double observation_distance = 0.5;
  double observation_rotation_angle = 0.1;
  double initial_small_speed = 1.0471975511965976;
  double initial_big_speed = 1.1775;
  double initial_amplitude = 0.9125;
  double initial_frequency = 1.942;
  double initial_phase = 0.0;
  double initial_covariance = 10.0;
  double small_speed_covariance = 0.01;
  double big_speed_covariance = 100.0;
  double big_phase_covariance = 400.0;
  double divergence_factor = 1.5;

  static SPRuneConfig load(const YAML::Node & node);
};
}  // namespace auto_buff_v2

#endif
