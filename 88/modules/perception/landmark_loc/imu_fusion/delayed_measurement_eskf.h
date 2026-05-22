#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace landmark_loc {
namespace delayed_measurement_eskf {

// Error-State Kalman Filter (ESKF) with delayed measurement support.
//
// State (5D): [px, py, vx, vy, yaw]
//   - px, py : position in world frame (metres)
//   - vx, vy : velocity in world frame (m/s)
//   - yaw    : heading angle (rad, normalised to [-pi, pi])
//
// Control (3D): [ax_body, ay_body, wz]
//   - ax_body, ay_body : linear acceleration in body frame (m/s²)
//                        from corrected_imu (gravity already zeroed)
//   - wz               : angular velocity around the vertical axis (rad/s)
//                        from corrected_imu
//
// Delayed measurement (3D): [mx, my, m_yaw]
//   - AMCL pose in world frame; may arrive late.
//   - The filter finds the historical snapshot nearest to the measurement
//     timestamp, applies the correction there, then replays all subsequent
//     IMU controls to recover a consistent latest estimate.
//
// Instant measurement (3D): [mx, my, m_yaw]
//   - RTK pose in world frame; assumed to arrive in real time (no delay).
//   - Applied directly to the latest snapshot — no history lookup or replay.
//   - Any subset of the three components can be masked by setting the
//     corresponding noise diagonal entry to a very large value (e.g. 1e9).

class DelayedMeasurementEskf {
 public:
  static constexpr int kStateDim = 5;
  static constexpr int kControlDim = 3;
  static constexpr int kMeasDim = 3;
  static constexpr int kZuptDim = 2;

  using StateVec = Eigen::Matrix<double, kStateDim, 1>;
  using StateMat = Eigen::Matrix<double, kStateDim, kStateDim>;
  using ControlVec = Eigen::Matrix<double, kControlDim, 1>;
  using MeasVec = Eigen::Matrix<double, kMeasDim, 1>;
  using MeasMat = Eigen::Matrix<double, kMeasDim, kMeasDim>;
  using ZuptVec = Eigen::Matrix<double, kZuptDim, 1>;
  using ZuptMat = Eigen::Matrix<double, kZuptDim, kZuptDim>;

  struct Estimate {
    double timestamp_sec = 0.0;
    StateVec state = StateVec::Zero();
    StateMat covariance = StateMat::Identity();
  };

  struct Options {
    std::size_t max_history_size = 4096;
    double timestamp_epsilon_sec = 1e-9;
    double min_covariance_diagonal = 1e-9;

    // Diagonal of initial state covariance P0.
    // Order: [px, py, vx, vy, yaw]
    StateVec default_initial_covariance =
        (StateVec() << 1e-2, 1e-2, 1e-1, 1e-1, 1e-3).finished();

    // Diagonal of IMU control noise Q.
    // Order: [ax_body, ay_body, wz]
    ControlVec default_control_covariance =
        (ControlVec() << 1e-2, 1e-2, 1e-4).finished();
  };

  DelayedMeasurementEskf();
  explicit DelayedMeasurementEskf(const Options& options);

  // Initialise with a known pose; velocity is assumed zero.
  // initial_state: [px, py, 0, 0, yaw]  (vx/vy are initialised to 0)
  bool Initialize(double timestamp_sec, const StateVec& initial_state);

  bool Initialize(double timestamp_sec,
                  const StateVec& initial_state,
                  const StateVec& initial_covariance_diag);

  // IMU measurement used as control input.
  // control: [ax_body, ay_body, wz] from corrected_imu
  bool AddControl(double timestamp_sec, const ControlVec& control);
  bool AddControl(double timestamp_sec,
                  const ControlVec& control,
                  const ControlVec& control_covariance_diag);

  // AMCL pose measurement — may arrive late.
  // Finds the historical snapshot at timestamp_sec, corrects it, then replays
  // all later IMU controls.
  // measurement: [mx, my, m_yaw]
  // measurement_covariance_diag: diagonal of 3×3 noise matrix R
  bool AddMeasurement(double timestamp_sec,
                      const MeasVec& measurement,
                      const MeasVec& measurement_covariance_diag);

  // RTK pose measurement — corrects the latest state in place.
  // This interface has no independent timestamp semantics: it always applies
  // at the latest snapshot time, so no history lookup or replay is needed.
  // measurement: [mx, my, m_yaw]
  // measurement_covariance_diag: diagonal of 3×3 noise matrix R
  bool AddInstantMeasurement(const MeasVec& measurement,
                             const MeasVec& measurement_covariance_diag);

  // Zero-velocity update — corrects the latest state assuming the vehicle is
  // stationary.  Observes [vx, vy] = [0, 0].
  // velocity_covariance_diag: diagonal of 2×2 noise matrix R for [vx, vy]
  bool AddZeroVelocityUpdate(const ZuptVec& velocity_covariance_diag);

  bool initialized() const { return initialized_; }
  Estimate latest_estimate() const;

 private:
  struct TimedControl {
    double start_time_sec = 0.0;
    double end_time_sec = 0.0;
    ControlVec control = ControlVec::Zero();
    ControlVec covariance_diag = ControlVec::Ones();

    TimedControl() = default;
    TimedControl(double t0, double t1,
                 const ControlVec& u, const ControlVec& cov)
        : start_time_sec(t0), end_time_sec(t1),
          control(u), covariance_diag(cov) {}
  };

  struct Snapshot {
    double timestamp_sec = 0.0;
    StateVec state = StateVec::Zero();
    StateMat covariance = StateMat::Identity();

    Snapshot() = default;
    Snapshot(double t, const StateVec& x, const StateMat& P)
        : timestamp_sec(t), state(x), covariance(P) {}
  };

  struct PropagationResult {
    StateVec state = StateVec::Zero();
    StateMat covariance = StateMat::Identity();
  };

  // ESKF predict step: propagates nominal state + error covariance.
  PropagationResult Propagate(const StateVec& state,
                              const StateMat& covariance,
                              const ControlVec& control,
                              const ControlVec& control_covariance_diag,
                              double dt_sec) const;

  // ESKF correct step: applies AMCL measurement to update state.
  PropagationResult Correct(const StateVec& state,
                            const StateMat& covariance,
                            const MeasVec& measurement,
                            const MeasVec& measurement_covariance_diag) const;

  // ESKF correct step: zero-velocity observation on [vx, vy].
  PropagationResult CorrectZeroVelocity(
      const StateVec& state,
      const StateMat& covariance,
      const ZuptVec& velocity_covariance_diag) const;

  void ReplayFromMeasurement(std::size_t snapshot_index,
                             double measurement_time_sec,
                             const StateVec& corrected_state,
                             const StateMat& corrected_covariance);
  void TrimHistory();
  bool NearlyEqual(double lhs, double rhs) const;

  StateMat CovarianceFromDiagonal(const StateVec& diagonal) const;
  MeasMat MeasCovarianceFromDiagonal(const MeasVec& diagonal) const;
  Eigen::Matrix<double, kControlDim, kControlDim>
      ControlCovarianceFromDiagonal(const ControlVec& diagonal) const;

  StateMat ClampCovariance(const StateMat& covariance) const;
  static double NormalizeAngle(double angle_rad);

  Options options_;
  bool initialized_ = false;
  std::vector<TimedControl> controls_;
  std::vector<Snapshot> snapshots_;
};

}  // namespace delayed_measurement_eskf
}  // namespace landmark_loc
