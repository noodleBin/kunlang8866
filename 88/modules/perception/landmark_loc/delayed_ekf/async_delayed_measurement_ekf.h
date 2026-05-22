#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

#include "delayed_measurement_ekf.h"

namespace landmark_loc {
namespace delayed_ekf {

class AsyncDelayedMeasurementEkf {
 public:
  using Vector3 = DelayedMeasurementEkf::Vector3;
  using Matrix3 = DelayedMeasurementEkf::Matrix3;
  using Estimate = DelayedMeasurementEkf::Estimate;
  using Options = DelayedMeasurementEkf::Options;

  explicit AsyncDelayedMeasurementEkf(const Options& options = Options());
  ~AsyncDelayedMeasurementEkf();

  AsyncDelayedMeasurementEkf(const AsyncDelayedMeasurementEkf&) = delete;
  AsyncDelayedMeasurementEkf& operator=(const AsyncDelayedMeasurementEkf&) =
      delete;

  void Initialize(double timestamp_sec, const Vector3& initial_state);
  void Initialize(double timestamp_sec, const Vector3& initial_state,
                  const Vector3& initial_covariance);
  void AddControl(double timestamp_sec, const Vector3& control);
  void AddControl(double timestamp_sec, const Vector3& control,
                  const Vector3& control_covariance);
  void AddMeasurement(double timestamp_sec, const Vector3& measurement,
                      const Vector3& measurement_covariance);

  Estimate latest_estimate() const;
  bool initialized() const;
  std::uint64_t latest_processed_sequence() const;
  void WaitUntilProcessed(std::uint64_t sequence) const;

 private:
  enum class EventType {
    kInitializeDefault,
    kInitializeExplicit,
    kControl,
    kMeasurement,
  };

  struct Event {
    std::uint64_t sequence = 0;
    EventType type = EventType::kControl;
    double timestamp_sec = 0.0;
    Vector3 value = Vector3::Zero();
    Vector3 covariance = Vector3::Ones();
  };

  void Enqueue(Event&& event);
  void WorkerLoop();
  void PublishLatestEstimateLocked();

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  mutable std::condition_variable processed_cv_;
  std::deque<Event> queue_;
  Options options_;
  DelayedMeasurementEkf ekf_;
  Estimate latest_estimate_;
  std::thread worker_;
  bool stop_requested_ = false;
  bool latest_initialized_ = false;
  std::uint64_t next_sequence_ = 1;
  std::uint64_t latest_processed_sequence_ = 0;
};

}  // namespace delayed_ekf
}  // namespace landmark_loc
