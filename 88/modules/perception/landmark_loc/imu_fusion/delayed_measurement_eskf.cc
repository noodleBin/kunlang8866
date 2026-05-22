#include "delayed_measurement_eskf.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

#include <Eigen/Cholesky>

#include "cyber/common/log.h"

namespace landmark_loc {
namespace delayed_measurement_eskf {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

DelayedMeasurementEskf::DelayedMeasurementEskf()
    : DelayedMeasurementEskf(Options()) {}

DelayedMeasurementEskf::DelayedMeasurementEskf(const Options& options)
    : options_(options) {}

bool DelayedMeasurementEskf::Initialize(double timestamp_sec,
                                        const StateVec& initial_state) {
  return Initialize(timestamp_sec, initial_state,
                    options_.default_initial_covariance);
}

bool DelayedMeasurementEskf::Initialize(
    double timestamp_sec,
    const StateVec& initial_state,
    const StateVec& initial_covariance_diag) {
  initialized_ = true;
  controls_.clear();
  snapshots_.clear();

  StateVec normalised = initial_state;
  normalised(4) = NormalizeAngle(initial_state(4));

  snapshots_.emplace_back(timestamp_sec, normalised,
                          CovarianceFromDiagonal(initial_covariance_diag));
  ADEBUG << "[ESKF] Initialize t=" << std::fixed << std::setprecision(3)
         << timestamp_sec << " state=[" << normalised.transpose() << "]";
  return true;
}

bool DelayedMeasurementEskf::AddControl(double timestamp_sec,
                                        const ControlVec& control) {
  return AddControl(timestamp_sec, control,
                    options_.default_control_covariance);
}

bool DelayedMeasurementEskf::AddControl(double timestamp_sec,
                                        const ControlVec& control,
                                        const ControlVec& control_covariance_diag) {
  if (!initialized_) {
    return false;
  }

  const double last_time_sec = snapshots_.back().timestamp_sec;
  if (timestamp_sec <= last_time_sec + options_.timestamp_epsilon_sec) {
    LOG(WARNING) << "[ESKF] AddControl rejected: t=" << std::fixed
                 << std::setprecision(6) << timestamp_sec
                 << " <= last=" << last_time_sec;
    return false;
  }

  const double dt_sec = timestamp_sec - last_time_sec;
  const PropagationResult propagated =
      Propagate(snapshots_.back().state, snapshots_.back().covariance,
                control, control_covariance_diag, dt_sec);

  controls_.emplace_back(last_time_sec, timestamp_sec,
                         control, control_covariance_diag);
  snapshots_.emplace_back(timestamp_sec, propagated.state, propagated.covariance);
  TrimHistory();
  ADEBUG << "[ESKF] AddControl t=" << std::fixed << std::setprecision(3)
         << timestamp_sec << " dt=" << std::setprecision(6) << dt_sec
         << " u=[" << control.transpose() << "]"
         << " -> state=[" << propagated.state.transpose() << "]";
  return true;
}

bool DelayedMeasurementEskf::AddMeasurement(
    double timestamp_sec,
    const MeasVec& measurement,
    const MeasVec& measurement_covariance_diag) {
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

  StateVec predicted_state = snapshots_[snapshot_index].state;
  StateMat predicted_covariance = snapshots_[snapshot_index].covariance;

  // Partial propagation when measurement falls between two snapshots.
  if (!NearlyEqual(timestamp_sec, snapshots_[snapshot_index].timestamp_sec)) {
    if (snapshot_index >= controls_.size()) {
      return false;
    }
    const TimedControl& ctrl = controls_[snapshot_index];
    const double partial_dt_sec = timestamp_sec - ctrl.start_time_sec;
    const PropagationResult partial =
        Propagate(predicted_state, predicted_covariance,
                  ctrl.control, ctrl.covariance_diag, partial_dt_sec);
    predicted_state = partial.state;
    predicted_covariance = partial.covariance;
  }

  const PropagationResult corrected =
      Correct(predicted_state, predicted_covariance,
              measurement, measurement_covariance_diag);
  ReplayFromMeasurement(snapshot_index, timestamp_sec,
                        corrected.state, corrected.covariance);
  return true;
}

bool DelayedMeasurementEskf::AddInstantMeasurement(
    const MeasVec& measurement,
    const MeasVec& measurement_covariance_diag) {
  if (!initialized_ || snapshots_.empty()) {
    return false;
  }

  // Apply correction in-place to the latest snapshot.
  Snapshot& latest = snapshots_.back();
  ADEBUG << "[ESKF] InstantMeas z=[" << measurement.transpose()
         << "] R_diag=[" << measurement_covariance_diag.transpose()
         << "] pre_state=[" << latest.state.transpose() << "]";
  const PropagationResult corrected =
      Correct(latest.state, latest.covariance,
              measurement, measurement_covariance_diag);
  latest.state = corrected.state;
  latest.covariance = corrected.covariance;
  ADEBUG << "[ESKF] InstantMeas post_state=[" << corrected.state.transpose()
         << "] P_diag=[" << corrected.covariance.diagonal().transpose() << "]";
  return true;
}

bool DelayedMeasurementEskf::AddZeroVelocityUpdate(
    const ZuptVec& velocity_covariance_diag) {
  if (!initialized_ || snapshots_.empty()) {
    return false;
  }

  Snapshot& latest = snapshots_.back();
  ADEBUG << "[ESKF] ZUPT pre_vel=[" << latest.state(2) << ", " << latest.state(3) << "]";
  const PropagationResult corrected =
      CorrectZeroVelocity(latest.state, latest.covariance,
                          velocity_covariance_diag);
  latest.state = corrected.state;
  latest.covariance = corrected.covariance;
  ADEBUG << "[ESKF] ZUPT post_vel=[" << corrected.state(2) << ", " << corrected.state(3)
         << "] post_state=[" << corrected.state.transpose() << "]";
  return true;
}

DelayedMeasurementEskf::Estimate
DelayedMeasurementEskf::latest_estimate() const {
  if (!initialized_ || snapshots_.empty()) {
    return Estimate{};
  }
  const Snapshot& latest = snapshots_.back();
  return Estimate{latest.timestamp_sec, latest.state, latest.covariance};
}

// ---------------------------------------------------------------------------
// ESKF predict step
// ---------------------------------------------------------------------------
// Propagates the 5-DOF nominal state using body-frame IMU measurements and
// updates the error-state covariance via the linearised state transition.
//
// Nominal dynamics (2D ground robot):
//   ax_w = cos(θ)*ax_b - sin(θ)*ay_b
//   ay_w = sin(θ)*ax_b + cos(θ)*ay_b
//
//   px' = px + vx*dt + 0.5*ax_w*dt²
//   py' = py + vy*dt + 0.5*ay_w*dt²
//   vx' = vx + ax_w*dt
//   vy' = vy + ay_w*dt
//   θ'  = θ  + wz*dt
//
// State Jacobian F = ∂f/∂x  (5×5):
//
//        px  py  vx  vy   θ
//   px [  1   0   dt   0   ½(-s·ax_b - c·ay_b)·dt² ]
//   py [  0   1    0  dt   ½( c·ax_b - s·ay_b)·dt² ]
//   vx [  0   0    1   0    (-s·ax_b - c·ay_b)·dt  ]
//   vy [  0   0    0   1    ( c·ax_b - s·ay_b)·dt  ]
//   θ  [  0   0    0   0    1                       ]
//
// Control Jacobian G = ∂f/∂u  (5×3):
//
//        ax_b         ay_b         wz
//   px [  ½c·dt²    -½s·dt²        0 ]
//   py [  ½s·dt²     ½c·dt²        0 ]
//   vx [  c·dt       -s·dt         0 ]
//   vy [  s·dt        c·dt         0 ]
//   θ  [  0            0          dt ]
// ---------------------------------------------------------------------------
DelayedMeasurementEskf::PropagationResult DelayedMeasurementEskf::Propagate(
    const StateVec& state,
    const StateMat& covariance,
    const ControlVec& control,
    const ControlVec& control_covariance_diag,
    double dt_sec) const {
  const double yaw = state(4);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double ax_b = control(0);
  const double ay_b = control(1);
  const double wz   = control(2);

  // World-frame acceleration
  const double ax_w = c * ax_b - s * ay_b;
  const double ay_w = s * ax_b + c * ay_b;
  const double dt2 = dt_sec * dt_sec;

  // Propagate nominal state
  StateVec next_state;
  next_state(0) = state(0) + state(2) * dt_sec + 0.5 * ax_w * dt2;
  next_state(1) = state(1) + state(3) * dt_sec + 0.5 * ay_w * dt2;
  next_state(2) = state(2) + ax_w * dt_sec;
  next_state(3) = state(3) + ay_w * dt_sec;
  next_state(4) = NormalizeAngle(yaw + wz * dt_sec);

  // State Jacobian F (5×5)
  StateMat F = StateMat::Identity();
  F(0, 2) = dt_sec;
  F(1, 3) = dt_sec;
  F(0, 4) = 0.5 * (-s * ax_b - c * ay_b) * dt2;
  F(1, 4) = 0.5 * ( c * ax_b - s * ay_b) * dt2;
  F(2, 4) =       (-s * ax_b - c * ay_b) * dt_sec;
  F(3, 4) =       ( c * ax_b - s * ay_b) * dt_sec;

  // Control Jacobian G (5×3)
  using ControlMat = Eigen::Matrix<double, kStateDim, kControlDim>;
  ControlMat G = ControlMat::Zero();
  G(0, 0) =  0.5 * c * dt2;
  G(0, 1) = -0.5 * s * dt2;
  G(1, 0) =  0.5 * s * dt2;
  G(1, 1) =  0.5 * c * dt2;
  G(2, 0) =  c * dt_sec;
  G(2, 1) = -s * dt_sec;
  G(3, 0) =  s * dt_sec;
  G(3, 1) =  c * dt_sec;
  G(4, 2) =  dt_sec;

  const auto Q = ControlCovarianceFromDiagonal(control_covariance_diag);
  const StateMat next_covariance = ClampCovariance(
      F * covariance * F.transpose() + G * Q * G.transpose());

  return PropagationResult{next_state, next_covariance};
}

// ---------------------------------------------------------------------------
// ESKF correct step
// ---------------------------------------------------------------------------
// Measurement model: z = H·x + v,  H (3×5) observes [px, py, θ].
//
//       px  py  vx  vy  θ
//   z0 [  1   0   0   0  0 ]   // px
//   z1 [  0   1   0   0  0 ]   // py
//   z2 [  0   0   0   0  1 ]   // yaw
//
// Error-state correction (Joseph form for numerical stability):
//   ν  = z - H·x̂
//   S  = H·P·Hᵀ + R
//   K  = P·Hᵀ·S⁻¹
//   x̂' = x̂ + K·ν     (inject error-state correction into nominal state)
//   P' = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ
// ---------------------------------------------------------------------------
DelayedMeasurementEskf::PropagationResult DelayedMeasurementEskf::Correct(
    const StateVec& state,
    const StateMat& covariance,
    const MeasVec& measurement,
    const MeasVec& measurement_covariance_diag) const {
  // H (3×5)
  using HMat = Eigen::Matrix<double, kMeasDim, kStateDim>;
  HMat H = HMat::Zero();
  H(0, 0) = 1.0;  // px
  H(1, 1) = 1.0;  // py
  H(2, 4) = 1.0;  // yaw

  // Predicted measurement from nominal state
  MeasVec predicted_meas;
  predicted_meas(0) = state(0);
  predicted_meas(1) = state(1);
  predicted_meas(2) = state(4);

  MeasVec innovation = measurement - predicted_meas;
  innovation(2) = NormalizeAngle(innovation(2));

  ADEBUG << "[ESKF] Correct innovation=[" << innovation.transpose() << "]";

  const MeasMat R = MeasCovarianceFromDiagonal(measurement_covariance_diag);
  const MeasMat S = H * covariance * H.transpose() + R;
  const MeasMat S_inv = S.ldlt().solve(MeasMat::Identity());

  // Kalman gain K (5×3)
  using KMat = Eigen::Matrix<double, kStateDim, kMeasDim>;
  const KMat K = covariance * H.transpose() * S_inv;
  ADEBUG << "[ESKF] Correct K_diag=[" << K(0,0) << ", " << K(1,1) << ", " << K(4,2) << "]"
         << " S_diag=[" << S.diagonal().transpose() << "]";

  // Inject correction into nominal state
  StateVec corrected_state = state + K * innovation;
  corrected_state(4) = NormalizeAngle(corrected_state(4));

  // Joseph form covariance update
  const StateMat IKH = StateMat::Identity() - K * H;
  const StateMat corrected_covariance = ClampCovariance(
      IKH * covariance * IKH.transpose() + K * R * K.transpose());

  return PropagationResult{corrected_state, corrected_covariance};
}

// ---------------------------------------------------------------------------
// ESKF zero-velocity correct step
// ---------------------------------------------------------------------------
// Measurement model: z = [0, 0],  H (2×5) observes [vx, vy].
//
//       px  py  vx  vy  θ
//   z0 [  0   0   1   0  0 ]   // vx
//   z1 [  0   0   0   1  0 ]   // vy
// ---------------------------------------------------------------------------
DelayedMeasurementEskf::PropagationResult
DelayedMeasurementEskf::CorrectZeroVelocity(
    const StateVec& state,
    const StateMat& covariance,
    const ZuptVec& velocity_covariance_diag) const {
  using HMat = Eigen::Matrix<double, kZuptDim, kStateDim>;
  HMat H = HMat::Zero();
  H(0, 2) = 1.0;  // vx
  H(1, 3) = 1.0;  // vy

  // Innovation: z - H·x̂ = [0,0] - [vx,vy]
  ZuptVec innovation;
  innovation(0) = -state(2);
  innovation(1) = -state(3);

  ZuptMat R = ZuptMat::Zero();
  R.diagonal() = velocity_covariance_diag;
  for (int i = 0; i < R.rows(); ++i) {
    R(i, i) = std::max(R(i, i), options_.min_covariance_diagonal);
  }

  const ZuptMat S = H * covariance * H.transpose() + R;
  const ZuptMat S_inv = S.ldlt().solve(ZuptMat::Identity());

  using KMat = Eigen::Matrix<double, kStateDim, kZuptDim>;
  const KMat K = covariance * H.transpose() * S_inv;

  StateVec corrected_state = state + K * innovation;
  corrected_state(4) = NormalizeAngle(corrected_state(4));

  const StateMat IKH = StateMat::Identity() - K * H;
  const StateMat corrected_covariance = ClampCovariance(
      IKH * covariance * IKH.transpose() + K * R * K.transpose());

  return PropagationResult{corrected_state, corrected_covariance};
}

void DelayedMeasurementEskf::ReplayFromMeasurement(
    std::size_t snapshot_index,
    double measurement_time_sec,
    const StateVec& corrected_state,
    const StateMat& corrected_covariance) {
  StateVec replay_state = corrected_state;
  StateMat replay_covariance = corrected_covariance;

  if (NearlyEqual(measurement_time_sec,
                  snapshots_[snapshot_index].timestamp_sec)) {
    snapshots_[snapshot_index].state = replay_state;
    snapshots_[snapshot_index].covariance = replay_covariance;
    for (std::size_t i = snapshot_index; i < controls_.size(); ++i) {
      const TimedControl& ctrl = controls_[i];
      const double dt_sec = ctrl.end_time_sec - ctrl.start_time_sec;
      const PropagationResult propagated =
          Propagate(replay_state, replay_covariance,
                    ctrl.control, ctrl.covariance_diag, dt_sec);
      replay_state = propagated.state;
      replay_covariance = propagated.covariance;
      snapshots_[i + 1].state = replay_state;
      snapshots_[i + 1].covariance = replay_covariance;
    }
    return;
  }

  // Measurement fell between snapshot_index and snapshot_index+1.
  const TimedControl& partial_ctrl = controls_[snapshot_index];
  const double remaining_dt =
      partial_ctrl.end_time_sec - measurement_time_sec;
  const PropagationResult remainder =
      Propagate(replay_state, replay_covariance,
                partial_ctrl.control, partial_ctrl.covariance_diag,
                remaining_dt);
  replay_state = remainder.state;
  replay_covariance = remainder.covariance;
  snapshots_[snapshot_index + 1].state = replay_state;
  snapshots_[snapshot_index + 1].covariance = replay_covariance;

  for (std::size_t i = snapshot_index + 1; i < controls_.size(); ++i) {
    const TimedControl& ctrl = controls_[i];
    const double dt_sec = ctrl.end_time_sec - ctrl.start_time_sec;
    const PropagationResult propagated =
        Propagate(replay_state, replay_covariance,
                  ctrl.control, ctrl.covariance_diag, dt_sec);
    replay_state = propagated.state;
    replay_covariance = propagated.covariance;
    snapshots_[i + 1].state = replay_state;
    snapshots_[i + 1].covariance = replay_covariance;
  }
}

void DelayedMeasurementEskf::TrimHistory() {
  while (controls_.size() > options_.max_history_size) {
    controls_.erase(controls_.begin());
    snapshots_.erase(snapshots_.begin());
  }
}

bool DelayedMeasurementEskf::NearlyEqual(double lhs, double rhs) const {
  return std::abs(lhs - rhs) <= options_.timestamp_epsilon_sec;
}

DelayedMeasurementEskf::StateMat
DelayedMeasurementEskf::CovarianceFromDiagonal(
    const StateVec& diagonal) const {
  StateMat cov = StateMat::Zero();
  cov.diagonal() = diagonal;
  return ClampCovariance(cov);
}

DelayedMeasurementEskf::MeasMat
DelayedMeasurementEskf::MeasCovarianceFromDiagonal(
    const MeasVec& diagonal) const {
  MeasMat cov = MeasMat::Zero();
  cov.diagonal() = diagonal;
  // Clamp diagonals to avoid singular matrices.
  for (int i = 0; i < cov.rows(); ++i) {
    cov(i, i) = std::max(cov(i, i), options_.min_covariance_diagonal);
  }
  return cov;
}

Eigen::Matrix<double, DelayedMeasurementEskf::kControlDim,
              DelayedMeasurementEskf::kControlDim>
DelayedMeasurementEskf::ControlCovarianceFromDiagonal(
    const ControlVec& diagonal) const {
  using ControlMat =
      Eigen::Matrix<double, kControlDim, kControlDim>;
  ControlMat cov = ControlMat::Zero();
  cov.diagonal() = diagonal;
  for (int i = 0; i < cov.rows(); ++i) {
    cov(i, i) = std::max(cov(i, i), options_.min_covariance_diagonal);
  }
  return cov;
}

DelayedMeasurementEskf::StateMat DelayedMeasurementEskf::ClampCovariance(
    const StateMat& covariance) const {
  StateMat clamped = 0.5 * (covariance + covariance.transpose());
  for (int i = 0; i < clamped.rows(); ++i) {
    clamped(i, i) = std::max(clamped(i, i), options_.min_covariance_diagonal);
  }
  return clamped;
}

double DelayedMeasurementEskf::NormalizeAngle(double angle_rad) {
  while (angle_rad >  kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

}  // namespace delayed_measurement_eskf
}  // namespace landmark_loc
