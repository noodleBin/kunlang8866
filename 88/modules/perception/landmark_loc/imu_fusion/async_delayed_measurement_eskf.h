#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

#include "delayed_measurement_eskf.h"

namespace landmark_loc {
namespace delayed_measurement_eskf {

// Thread-safe, asynchronous wrapper around DelayedMeasurementEskf.
//
// All public methods are non-blocking: they enqueue an event and return
// immediately. A dedicated worker thread processes the queue in order,
// keeping the filter state consistent at all times.
//
// Usage pattern:
//   AsyncDelayedMeasurementEskf filter;
//   filter.Initialize(t0, initial_state);
//   // IMU callback (high rate):
//   filter.AddControl(t_imu, imu_control);
//   // AMCL callback (low rate, may be delayed):
//   uint64_t seq = filter.AddMeasurement(t_amcl, amcl_pose, amcl_cov);
//   filter.WaitUntilProcessed(seq);   // optional: block until fused
//   auto estimate = filter.latest_estimate();

class AsyncDelayedMeasurementEskf {
 public:
  using StateVec = DelayedMeasurementEskf::StateVec;
  using StateMat = DelayedMeasurementEskf::StateMat;
  using ControlVec = DelayedMeasurementEskf::ControlVec;
  using MeasVec = DelayedMeasurementEskf::MeasVec;
  using ZuptVec = DelayedMeasurementEskf::ZuptVec;
  using Estimate = DelayedMeasurementEskf::Estimate;
  using Options = DelayedMeasurementEskf::Options;

  explicit AsyncDelayedMeasurementEskf(const Options& options = Options());
  ~AsyncDelayedMeasurementEskf();

  AsyncDelayedMeasurementEskf(const AsyncDelayedMeasurementEskf&) = delete;
  AsyncDelayedMeasurementEskf& operator=(
      const AsyncDelayedMeasurementEskf&) = delete;

  void Initialize(double timestamp_sec, const StateVec& initial_state);
  void Initialize(double timestamp_sec,
                  const StateVec& initial_state,
                  const StateVec& initial_covariance_diag);

  // Returns the sequence number assigned to this event (use with
  // WaitUntilProcessed if you need the result synchronously).
  std::uint64_t AddControl(double timestamp_sec, const ControlVec& control);
  std::uint64_t AddControl(double timestamp_sec,
                           const ControlVec& control,
                           const ControlVec& control_covariance_diag);

  // AMCL pose — delayed, triggers history replay.
  std::uint64_t AddMeasurement(double timestamp_sec,
                               const MeasVec& measurement,
                               const MeasVec& measurement_covariance_diag);

  // RTK pose — instant, applied directly to the latest state.
  std::uint64_t AddInstantMeasurement(
      const MeasVec& measurement,
      const MeasVec& measurement_covariance_diag);

  // Zero-velocity update — instant, observes [vx, vy] = [0, 0].
  std::uint64_t AddZeroVelocityUpdate(
      const ZuptVec& velocity_covariance_diag);

  Estimate latest_estimate() const;
  bool initialized() const;
  std::uint64_t latest_processed_sequence() const;

  // Block until the event with the given sequence number has been processed.
  void WaitUntilProcessed(std::uint64_t sequence) const;

 private:
  enum class EventType {
    kInitializeDefault,
    kInitializeExplicit,
    kControl,
    kMeasurement,        // delayed (AMCL) — triggers replay
    kInstantMeasurement, // real-time (RTK) — no replay
    kZeroVelocityUpdate, // ZUPT — observes [vx, vy] = [0, 0]
  };

  struct Event {
    std::uint64_t sequence = 0;
    EventType type = EventType::kControl;
    double timestamp_sec = 0.0;
    StateVec state_value = StateVec::Zero();    // for Initialize events
    ControlVec control_value = ControlVec::Zero();
    MeasVec meas_value = MeasVec::Zero();
    StateVec state_covariance = StateVec::Ones();
    ControlVec control_covariance = ControlVec::Ones();
    MeasVec meas_covariance = MeasVec::Ones();
    ZuptVec zupt_covariance = ZuptVec::Ones();
  };

  std::uint64_t Enqueue(Event&& event);
  void WorkerLoop();

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  mutable std::condition_variable processed_cv_;
  std::deque<Event> queue_;
  Options options_;
  DelayedMeasurementEskf eskf_;
  Estimate latest_estimate_;
  std::thread worker_;
  bool stop_requested_ = false;
  bool latest_initialized_ = false;
  std::uint64_t next_sequence_ = 1;
  std::uint64_t latest_processed_sequence_ = 0;
};

}  // namespace delayed_measurement_eskf
}  // namespace landmark_loc
