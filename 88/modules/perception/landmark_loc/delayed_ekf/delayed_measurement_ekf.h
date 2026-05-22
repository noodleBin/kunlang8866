#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace landmark_loc {
namespace delayed_ekf {

class DelayedMeasurementEkf {
 public:
  using Vector3 = Eigen::Vector3d;
  using Matrix3 = Eigen::Matrix3d;

  struct Estimate {
    double timestamp_sec = 0.0;
    Vector3 state = Vector3::Zero();
    Matrix3 covariance = Matrix3::Identity();
  };

  struct Options {
    bool use_delayed_ekf = false;
    std::size_t max_history_size = 2048;
    double timestamp_epsilon_sec = 1e-9;
    double min_covariance_diagonal = 1e-9;
    Vector3 default_initial_covariance = Vector3(1e-4, 1e-4, 7.6e-5);
    Vector3 default_control_covariance = Vector3(1e-2, 1e-2, 1e-4);
  };

  DelayedMeasurementEkf();
  explicit DelayedMeasurementEkf(const Options& options);

  bool Initialize(double timestamp_sec, const Vector3& initial_state);

  bool Initialize(double timestamp_sec,
                  const Vector3& initial_state,
                  const Vector3& initial_covariance);

  // The control sample is treated as the piecewise-constant velocity applied
  // over the interval [last_control_time, timestamp_sec].
  bool AddControl(double timestamp_sec,
                  const Vector3& control);

  bool AddControl(double timestamp_sec,
                  const Vector3& control,
                  const Vector3& control_covariance);

  // The measurement can arrive late. The filter updates the historical state at
  // timestamp_sec and then replays all later controls to recover the latest
  // estimate.
  bool AddMeasurement(double timestamp_sec,
                      const Vector3& measurement,
                      const Vector3& measurement_covariance);

  bool initialized() const { return initialized_; }
  Estimate latest_estimate() const;

 private:
  struct TimedControl {
    double start_time_sec = 0.0;
    double end_time_sec = 0.0;
    Vector3 control = Vector3::Zero();
    Vector3 covariance = Vector3::Ones();

    TimedControl() = default;
    TimedControl(double start_time_sec_in, double end_time_sec_in,
                 const Vector3& control_in, const Vector3& covariance_in)
        : start_time_sec(start_time_sec_in),
          end_time_sec(end_time_sec_in),
          control(control_in),
          covariance(covariance_in) {}
  };

  struct Snapshot {
    double timestamp_sec = 0.0;
    Vector3 state = Vector3::Zero();
    Matrix3 covariance = Matrix3::Identity();

    Snapshot() = default;
    Snapshot(double timestamp_sec_in, const Vector3& state_in,
             const Matrix3& covariance_in)
        : timestamp_sec(timestamp_sec_in),
          state(state_in),
          covariance(covariance_in) {}
  };

  struct PropagationResult {
    Vector3 state = Vector3::Zero();
    Matrix3 covariance = Matrix3::Identity();
  };

  PropagationResult Propagate(const Vector3& state,
                              const Matrix3& covariance,
                              const Vector3& control,
                              const Vector3& control_covariance,
                              double dt_sec) const;

  PropagationResult Correct(const Vector3& state,
                            const Matrix3& covariance,
                            const Vector3& measurement,
                            const Vector3& measurement_covariance) const;

  void ReplayFromMeasurement(std::size_t snapshot_index,
                             double measurement_time_sec,
                             const Vector3& corrected_state,
                             const Matrix3& corrected_covariance);
  void TrimHistory();
  bool NearlyEqual(double lhs, double rhs) const;
  Matrix3 CovarianceFromDiagonal(const Vector3& diagonal) const;
  Matrix3 ClampCovariance(const Matrix3& covariance) const;
  static double NormalizeAngle(double angle_rad);

  Options options_;
  bool initialized_ = false;
  std::vector<TimedControl> controls_;
  std::vector<Snapshot> snapshots_;
};

}  // namespace delayed_ekf
}  // namespace landmark_loc
