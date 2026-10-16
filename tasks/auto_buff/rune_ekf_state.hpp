#ifndef AUTO_BUFF__RUNE_EKF_STATE_HPP
#define AUTO_BUFF__RUNE_EKF_STATE_HPP

#include <Eigen/Dense>

namespace auto_buff
{
enum class RuneEkfStateComponent
{
  center_x,
  center_y,
  center_z,
  rotation_speed,
  rotation_angle,
  face_yaw
};

class RuneEkfState
{
public:
  static constexpr Eigen::Index dimension = 6;
  using Vector = Eigen::Matrix<double, dimension, 1>;

  RuneEkfState() = default;
  explicit RuneEkfState(const Vector & values) : values_{values} {}

  const Vector & vector() const { return values_; }
  bool all_finite() const { return values_.allFinite(); }

  double center_x() const { return values_[index(RuneEkfStateComponent::center_x)]; }
  double center_y() const { return values_[index(RuneEkfStateComponent::center_y)]; }
  double center_z() const { return values_[index(RuneEkfStateComponent::center_z)]; }
  double rotation_speed() const
  {
    return values_[index(RuneEkfStateComponent::rotation_speed)];
  }
  double rotation_angle() const
  {
    return values_[index(RuneEkfStateComponent::rotation_angle)];
  }
  double face_yaw() const { return values_[index(RuneEkfStateComponent::face_yaw)]; }

  void set_center_x(double value) { values_[index(RuneEkfStateComponent::center_x)] = value; }
  void set_center_y(double value) { values_[index(RuneEkfStateComponent::center_y)] = value; }
  void set_center_z(double value) { values_[index(RuneEkfStateComponent::center_z)] = value; }
  void set_rotation_speed(double value)
  {
    values_[index(RuneEkfStateComponent::rotation_speed)] = value;
  }
  void set_rotation_angle(double value)
  {
    values_[index(RuneEkfStateComponent::rotation_angle)] = value;
  }
  void set_face_yaw(double value) { values_[index(RuneEkfStateComponent::face_yaw)] = value; }

  RuneEkfState predict(double seconds) const
  {
    auto predicted = *this;
    predicted.add_rotation_angle(rotation_speed() * seconds);
    return predicted;
  }

  void add_rotation_angle(double value) { set_rotation_angle(rotation_angle() + value); }
  void add_component(RuneEkfStateComponent component, double value)
  {
    values_[index(component)] += value;
  }

private:
  static constexpr Eigen::Index index(RuneEkfStateComponent component)
  {
    return static_cast<Eigen::Index>(component);
  }

  Vector values_ = Vector::Zero();
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_EKF_STATE_HPP
