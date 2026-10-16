#ifndef AUTO_BUFF__NJUST_RUNE_MODEL_HPP
#define AUTO_BUFF__NJUST_RUNE_MODEL_HPP

#include "rune_model.hpp"
#include "../rune_ekf_state.hpp"
#include "../rune_energy_fitter.hpp"

namespace auto_buff
{
class NjustRuneModel final : public RuneModel
{
public:
  struct Config
  {
    RunePredictionMethod prediction_method = RunePredictionMethod::weighted_least_squares;  // 小符
    BuffConfig::MotionFit small_motion_fit;
    BuffConfig::MotionFit big_motion_fit;
    double timeout_seconds = 1.5;
    double noise_x = 1e-5;
    double noise_y = 1e-5;
    double noise_z = 1e-5;
    double noise_rotation_speed = 1;
    double noise_rotation_angle = 1e-3;
    double noise_face_yaw = 1e-5;
    double noise_observation = 20;
    double gate_threshold = 13.816;
    double init_seed_mean_error = 10;
    double init_seed_max_error = 20;
    double init_center_gate = 30;
    double init_pitch_bound = 20;
    double diverge_face_angle = 45;

    static Config load(const YAML::Node & node);
  };

  NjustRuneModel(BuffConfig::Camera camera, Config config, bool big_rune);
  void set_q_gimbal2world(const Eigen::Quaterniond & q_gimbal2world) override;
  bool update(const RuneElements & elements, Timestamp timestamp) override;
  void reset() override;
  std::optional<RuneEstimate> state() const override;
  const std::string & diagnostic() const override { return diagnostic_; }
  std::vector<RuneReprojectedFeature> reprojected_features() const override;
  std::optional<cv::Point2f> reprojected_center() const override;

private:
  bool initialize(const RuneElements & elements, Timestamp timestamp,
                  const Eigen::Matrix3d & R_camera2world,
                  const Eigen::Vector3d & t_camera2world);
  void update_motion_fit(RuneEstimate & state, double elapsed_seconds);
  bool diverged() const;

  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_camera2gimbal_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_camera2gimbal_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_gimbal2world_ = Eigen::Quaterniond::Identity();
  bool big_rune_ = false;
  Config config_;
  std::string diagnostic_;
  std::optional<RuneEstimate> state_;
  RuneEnergyFitter fitter_;
  Eigen::Matrix<double, 6, 6> covariance_ = Eigen::Matrix<double, 6, 6>::Identity();
  RuneEkfState ekf_state_;
  std::array<Timestamp, 5> inactive_timeout_{};
  Timestamp last_inactive_corrected_{};
  Timestamp force_sine_until_{};
};
}  // namespace auto_buff

#endif
