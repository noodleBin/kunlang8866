#include "async_delayed_measurement_ekf.h"

#include <utility>

namespace landmark_loc {
namespace delayed_ekf {

AsyncDelayedMeasurementEkf::AsyncDelayedMeasurementEkf(const Options& options)
    : options_(options),
      ekf_(options),
      worker_(&AsyncDelayedMeasurementEkf::WorkerLoop, this) {}

AsyncDelayedMeasurementEkf::~AsyncDelayedMeasurementEkf() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
  }
  cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AsyncDelayedMeasurementEkf::Initialize(double timestamp_sec,
                                            const Vector3& initial_state) {
  Event event;
  event.type = EventType::kInitializeDefault;
  event.timestamp_sec = timestamp_sec;
  event.value = initial_state;
  Enqueue(std::move(event));
}

void AsyncDelayedMeasurementEkf::Initialize(double timestamp_sec,
                                            const Vector3& initial_state,
                                            const Vector3& initial_covariance) {
  Event event;
  event.type = EventType::kInitializeExplicit;
  event.timestamp_sec = timestamp_sec;
  event.value = initial_state;
  event.covariance = initial_covariance;
  Enqueue(std::move(event));
}

void AsyncDelayedMeasurementEkf::AddControl(double timestamp_sec,
                                            const Vector3& control) {
  Event event;
  event.type = EventType::kControl;
  event.timestamp_sec = timestamp_sec;
  event.value = control;
  event.covariance = options_.default_control_covariance;
  Enqueue(std::move(event));
}

void AsyncDelayedMeasurementEkf::AddControl(double timestamp_sec,
                                            const Vector3& control,
                                            const Vector3& control_covariance) {
  Event event;
  event.type = EventType::kControl;
  event.timestamp_sec = timestamp_sec;
  event.value = control;
  event.covariance = control_covariance;
  Enqueue(std::move(event));
}

void AsyncDelayedMeasurementEkf::AddMeasurement(
    double timestamp_sec, const Vector3& measurement,
    const Vector3& measurement_covariance) {
  Event event;
  event.type = EventType::kMeasurement;
  event.timestamp_sec = timestamp_sec;
  event.value = measurement;
  event.covariance = measurement_covariance;
  Enqueue(std::move(event));
}

AsyncDelayedMeasurementEkf::Estimate
AsyncDelayedMeasurementEkf::latest_estimate() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_estimate_;
}

bool AsyncDelayedMeasurementEkf::initialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_initialized_;
}

std::uint64_t AsyncDelayedMeasurementEkf::latest_processed_sequence() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_processed_sequence_;
}

void AsyncDelayedMeasurementEkf::WaitUntilProcessed(std::uint64_t sequence) const {
  std::unique_lock<std::mutex> lock(mutex_);
  processed_cv_.wait(lock, [this, sequence]() {
    return latest_processed_sequence_ >= sequence;
  });
}

void AsyncDelayedMeasurementEkf::Enqueue(Event&& event) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    event.sequence = next_sequence_++;
    queue_.push_back(std::move(event));
  }
  cv_.notify_one();
}

void AsyncDelayedMeasurementEkf::WorkerLoop() {
  while (true) {
    Event event;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return stop_requested_ || !queue_.empty(); });
      if (stop_requested_ && queue_.empty()) {
        return;
      }
      event = std::move(queue_.front());
      queue_.pop_front();
    }

    switch (event.type) {
      case EventType::kInitializeDefault:
        ekf_.Initialize(event.timestamp_sec, event.value);
        break;
      case EventType::kInitializeExplicit:
        ekf_.Initialize(event.timestamp_sec, event.value, event.covariance);
        break;
      case EventType::kControl:
        ekf_.AddControl(event.timestamp_sec, event.value, event.covariance);
        break;
      case EventType::kMeasurement:
        ekf_.AddMeasurement(event.timestamp_sec, event.value, event.covariance);
        break;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_initialized_ = ekf_.initialized();
      if (latest_initialized_) {
        latest_estimate_ = ekf_.latest_estimate();
      }
      latest_processed_sequence_ = event.sequence;
    }
    processed_cv_.notify_all();
  }
}

}  // namespace delayed_ekf
}  // namespace landmark_loc
