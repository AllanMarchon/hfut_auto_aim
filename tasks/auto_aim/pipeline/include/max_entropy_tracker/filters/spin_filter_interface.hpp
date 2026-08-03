// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_SPIN_FILTER_INTERFACE_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_SPIN_FILTER_INTERFACE_HPP_

#include <utility>

#include <Eigen/Dense>

#include "max_entropy_tracker/filters/process_models/composite.hpp"

namespace fyt::auto_aim {

/// Shared read/write interface for spin-based filters used by trackers.
class SpinFilterInterface {
 public:
  virtual ~SpinFilterInterface() = default;

  virtual const Eigen::VectorXd &x() const = 0;
  virtual Eigen::VectorXd &x() = 0;
  virtual const Eigen::MatrixXd &P() const = 0;
  virtual Eigen::MatrixXd &P() = 0;

  virtual const DynamicStateIndex &state_idx() const = 0;

  virtual Eigen::Vector3d get_center_position() const = 0;
  virtual std::pair<double, double> get_radii() const = 0;
  virtual double get_dza() const = 0;
  virtual double get_yaw() const = 0;
  /// Raw whole-vehicle yaw state as held in the filter layout (unnormalized,
  /// no armor-phase composition). Use get_yaw() for the publishable heading.
  virtual double get_raw_yaw() const = 0;
  virtual int get_k() const = 0;

  virtual const Eigen::VectorXd &last_innov_xyz() const = 0;
  virtual double last_innov_yaw() const = 0;
  virtual double last_nis() const = 0;
  virtual int last_update_type() const = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_SPIN_FILTER_INTERFACE_HPP_
