#include "delayed_measurement_ekf.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace landmark_loc {
namespace delayed_ekf {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

DelayedMeasurementEkf::DelayedMeasurementEkf()
    : DelayedMeasurementEkf(Options()) {}

DelayedMeasurementEkf::DelayedMeasurementEkf(const Options& options)
    : options_(options) {}

bool DelayedMeasurementEkf::Initialize(double timestamp_sec,
                                       const Vector3& initial_state) {
  return Initialize(timestamp_sec, initial_state,
                    options_.default_initial_covariance);
}

bool DelayedMeasurementEkf::Initialize(double timestamp_sec,
                                       const Vector3& initial_state,
                                       const Vector3& initial_covariance) {
  initialized_ = true;
  controls_.clear();
  snapshots_.clear();
  snapshots_.emplace_back(timestamp_sec,
               Vector3(initial_state.x(), initial_state.y(),
                       NormalizeAngle(initial_state.z())),
               CovarianceFromDiagonal(initial_covariance));
  return true;
}

bool DelayedMeasurementEkf::AddControl(double timestamp_sec,
                                       const Vector3& control) {
  return AddControl(timestamp_sec, control, options_.default_control_covariance);
}

bool DelayedMeasurementEkf::AddControl(double timestamp_sec,
                                       const Vector3& control,
                                       const Vector3& control_covariance) {
  if (!initialized_) {
    return false;
  }

  const double last_time_sec = snapshots_.back().timestamp_sec;
  if (timestamp_sec <= last_time_sec + options_.timestamp_epsilon_sec) {
    return false;
  }

  const double dt_sec = timestamp_sec - last_time_sec;
  const PropagationResult propagated =
      Propagate(snapshots_.back().state, snapshots_.back().covariance, control,
                control_covariance, dt_sec);

  controls_.emplace_back(last_time_sec, timestamp_sec, control, control_covariance);
  snapshots_.emplace_back(timestamp_sec, propagated.state, propagated.covariance);
  TrimHistory();
  return true;
}

bool DelayedMeasurementEkf::AddMeasurement(
    double timestamp_sec, const Vector3& measurement,
    const Vector3& measurement_covariance) {
  if (!initialized_ || snapshots_.empty()) {
    return false;
  }

  if (timestamp_sec < snapshots_.front().timestamp_sec -
                           options_.timestamp_epsilon_sec ||
      timestamp_sec > snapshots_.back().timestamp_sec +
                           options_.timestamp_epsilon_sec) {
    return false;
  }

  auto upper = std::upper_bound(
      snapshots_.begin(), snapshots_.end(), timestamp_sec,
      [](double value, const Snapshot& snapshot) {
        return value < snapshot.timestamp_sec;
      });
  std::size_t snapshot_index = 0;
  if (upper == snapshots_.begin()) {
    snapshot_index = 0;
  } else {
    snapshot_index = static_cast<std::size_t>(upper - snapshots_.begin() - 1);
  }

  Vector3 predicted_state = snapshots_[snapshot_index].state;
  Matrix3 predicted_covariance = snapshots_[snapshot_index].covariance;
  if (!NearlyEqual(timestamp_sec, snapshots_[snapshot_index].timestamp_sec)) {
    if (snapshot_index >= controls_.size()) {
      return false;
    }
    const TimedControl& control = controls_[snapshot_index];
    const double partial_dt_sec = timestamp_sec - control.start_time_sec;
    const PropagationResult partial =
        Propagate(predicted_state, predicted_covariance, control.control,
                  control.covariance, partial_dt_sec);
    predicted_state = partial.state;
    predicted_covariance = partial.covariance;
  }

  const PropagationResult corrected =
      Correct(predicted_state, predicted_covariance, measurement,
              measurement_covariance);
  ReplayFromMeasurement(snapshot_index, timestamp_sec, corrected.state,
                        corrected.covariance);
  return true;
}

DelayedMeasurementEkf::Estimate DelayedMeasurementEkf::latest_estimate() const {
  if (!initialized_ || snapshots_.empty()) {
    return Estimate{};
  }
  const Snapshot& latest = snapshots_.back();
  return Estimate{latest.timestamp_sec, latest.state, latest.covariance};
}

DelayedMeasurementEkf::PropagationResult DelayedMeasurementEkf::Propagate(
    const Vector3& state, const Matrix3& covariance, const Vector3& control,
    const Vector3& control_covariance, double dt_sec) const {
  const double yaw = state.z();
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double vx = control.x();
  const double vy = control.y();
  const double yaw_rate = control.z();

  Vector3 predicted_state = state;
  predicted_state.x() += (cos_yaw * vx - sin_yaw * vy) * dt_sec;
  predicted_state.y() += (sin_yaw * vx + cos_yaw * vy) * dt_sec;
  predicted_state.z() = NormalizeAngle(yaw + yaw_rate * dt_sec);

  Matrix3 state_jacobian = Matrix3::Identity();
  state_jacobian(0, 2) = (-sin_yaw * vx - cos_yaw * vy) * dt_sec;
  state_jacobian(1, 2) = (cos_yaw * vx - sin_yaw * vy) * dt_sec;

  Matrix3 control_jacobian = Matrix3::Zero();
  control_jacobian(0, 0) = cos_yaw * dt_sec;
  control_jacobian(0, 1) = -sin_yaw * dt_sec;
  control_jacobian(1, 0) = sin_yaw * dt_sec;
  control_jacobian(1, 1) = cos_yaw * dt_sec;
  control_jacobian(2, 2) = dt_sec;

  const Matrix3 control_noise = CovarianceFromDiagonal(control_covariance);
  const Matrix3 predicted_covariance = ClampCovariance(
      state_jacobian * covariance * state_jacobian.transpose() +
      control_jacobian * control_noise * control_jacobian.transpose());
  return PropagationResult{predicted_state, predicted_covariance};
}

DelayedMeasurementEkf::PropagationResult DelayedMeasurementEkf::Correct(
    const Vector3& state, const Matrix3& covariance, const Vector3& measurement,
    const Vector3& measurement_covariance) const {
  Vector3 innovation = measurement - state;
  innovation.z() = NormalizeAngle(innovation.z());

  const Matrix3 measurement_noise = CovarianceFromDiagonal(measurement_covariance);
  const Matrix3 innovation_covariance = covariance + measurement_noise;
  const Matrix3 innovation_inverse = innovation_covariance.ldlt().solve(Matrix3::Identity());
  const Matrix3 kalman_gain = covariance * innovation_inverse;

  Vector3 corrected_state = state + kalman_gain * innovation;
  corrected_state.z() = NormalizeAngle(corrected_state.z());

  const Matrix3 identity = Matrix3::Identity();
  const Matrix3 corrected_covariance = ClampCovariance(
      (identity - kalman_gain) * covariance * (identity - kalman_gain).transpose() +
      kalman_gain * measurement_noise * kalman_gain.transpose());
  return PropagationResult{corrected_state, corrected_covariance};
}

void DelayedMeasurementEkf::ReplayFromMeasurement(
    std::size_t snapshot_index, double measurement_time_sec,
    const Vector3& corrected_state, const Matrix3& corrected_covariance) {
  Vector3 replay_state = corrected_state;
  Matrix3 replay_covariance = corrected_covariance;

  if (NearlyEqual(measurement_time_sec, snapshots_[snapshot_index].timestamp_sec)) {
    snapshots_[snapshot_index].state = replay_state;
    snapshots_[snapshot_index].covariance = replay_covariance;
    for (std::size_t control_index = snapshot_index;
         control_index < controls_.size(); ++control_index) {
      const TimedControl& control = controls_[control_index];
      const double dt_sec = control.end_time_sec - control.start_time_sec;
      const PropagationResult propagated =
          Propagate(replay_state, replay_covariance, control.control,
                    control.covariance, dt_sec);
      replay_state = propagated.state;
      replay_covariance = propagated.covariance;
      snapshots_[control_index + 1].state = replay_state;
      snapshots_[control_index + 1].covariance = replay_covariance;
    }
    return;
  }

  const TimedControl& partial_control = controls_[snapshot_index];
  const double remaining_dt_sec =
      partial_control.end_time_sec - measurement_time_sec;
  const PropagationResult remainder =
      Propagate(replay_state, replay_covariance, partial_control.control,
                partial_control.covariance, remaining_dt_sec);
  replay_state = remainder.state;
  replay_covariance = remainder.covariance;
  snapshots_[snapshot_index + 1].state = replay_state;
  snapshots_[snapshot_index + 1].covariance = replay_covariance;

  for (std::size_t control_index = snapshot_index + 1;
       control_index < controls_.size(); ++control_index) {
    const TimedControl& control = controls_[control_index];
    const double dt_sec = control.end_time_sec - control.start_time_sec;
    const PropagationResult propagated =
        Propagate(replay_state, replay_covariance, control.control,
                  control.covariance, dt_sec);
    replay_state = propagated.state;
    replay_covariance = propagated.covariance;
    snapshots_[control_index + 1].state = replay_state;
    snapshots_[control_index + 1].covariance = replay_covariance;
  }
}

void DelayedMeasurementEkf::TrimHistory() {
  while (controls_.size() > options_.max_history_size) {
    controls_.erase(controls_.begin());
    snapshots_.erase(snapshots_.begin());
  }
}

bool DelayedMeasurementEkf::NearlyEqual(double lhs, double rhs) const {
  return std::abs(lhs - rhs) <= options_.timestamp_epsilon_sec;
}

DelayedMeasurementEkf::Matrix3 DelayedMeasurementEkf::CovarianceFromDiagonal(
    const Vector3& diagonal) const {
  Matrix3 covariance = Matrix3::Zero();
  covariance.diagonal() = diagonal;
  return ClampCovariance(covariance);
}

DelayedMeasurementEkf::Matrix3 DelayedMeasurementEkf::ClampCovariance(
    const Matrix3& covariance) const {
  Matrix3 clamped = 0.5 * (covariance + covariance.transpose());
  for (int i = 0; i < clamped.rows(); ++i) {
    clamped(i, i) =
        std::max(clamped(i, i), options_.min_covariance_diagonal);
  }
  return clamped;
}

double DelayedMeasurementEkf::NormalizeAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

}  // namespace delayed_ekf
}  // namespace landmark_loc
