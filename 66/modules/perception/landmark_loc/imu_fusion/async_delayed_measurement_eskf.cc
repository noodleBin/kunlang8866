#include "async_delayed_measurement_eskf.h"

#include <utility>

namespace landmark_loc {
namespace delayed_measurement_eskf {

AsyncDelayedMeasurementEskf::AsyncDelayedMeasurementEskf(
    const Options& options)
    : options_(options),
      eskf_(options),
      worker_(&AsyncDelayedMeasurementEskf::WorkerLoop, this) {}

AsyncDelayedMeasurementEskf::~AsyncDelayedMeasurementEskf() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
  }
  cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AsyncDelayedMeasurementEskf::Initialize(double timestamp_sec,
                                             const StateVec& initial_state) {
  Event event;
  event.type = EventType::kInitializeDefault;
  event.timestamp_sec = timestamp_sec;
  event.state_value = initial_state;
  Enqueue(std::move(event));
}

void AsyncDelayedMeasurementEskf::Initialize(
    double timestamp_sec,
    const StateVec& initial_state,
    const StateVec& initial_covariance_diag) {
  Event event;
  event.type = EventType::kInitializeExplicit;
  event.timestamp_sec = timestamp_sec;
  event.state_value = initial_state;
  event.state_covariance = initial_covariance_diag;
  Enqueue(std::move(event));
}

std::uint64_t AsyncDelayedMeasurementEskf::AddControl(
    double timestamp_sec, const ControlVec& control) {
  Event event;
  event.type = EventType::kControl;
  event.timestamp_sec = timestamp_sec;
  event.control_value = control;
  event.control_covariance = options_.default_control_covariance;
  return Enqueue(std::move(event));
}

std::uint64_t AsyncDelayedMeasurementEskf::AddControl(
    double timestamp_sec,
    const ControlVec& control,
    const ControlVec& control_covariance_diag) {
  Event event;
  event.type = EventType::kControl;
  event.timestamp_sec = timestamp_sec;
  event.control_value = control;
  event.control_covariance = control_covariance_diag;
  return Enqueue(std::move(event));
}

std::uint64_t AsyncDelayedMeasurementEskf::AddMeasurement(
    double timestamp_sec,
    const MeasVec& measurement,
    const MeasVec& measurement_covariance_diag) {
  Event event;
  event.type = EventType::kMeasurement;
  event.timestamp_sec = timestamp_sec;
  event.meas_value = measurement;
  event.meas_covariance = measurement_covariance_diag;
  return Enqueue(std::move(event));
}

std::uint64_t AsyncDelayedMeasurementEskf::AddInstantMeasurement(
    const MeasVec& measurement,
    const MeasVec& measurement_covariance_diag) {
  Event event;
  event.type = EventType::kInstantMeasurement;
  event.meas_value = measurement;
  event.meas_covariance = measurement_covariance_diag;
  return Enqueue(std::move(event));
}

std::uint64_t AsyncDelayedMeasurementEskf::AddZeroVelocityUpdate(
    const ZuptVec& velocity_covariance_diag) {
  Event event;
  event.type = EventType::kZeroVelocityUpdate;
  event.zupt_covariance = velocity_covariance_diag;
  return Enqueue(std::move(event));
}

AsyncDelayedMeasurementEskf::Estimate
AsyncDelayedMeasurementEskf::latest_estimate() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_estimate_;
}

bool AsyncDelayedMeasurementEskf::initialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_initialized_;
}

std::uint64_t AsyncDelayedMeasurementEskf::latest_processed_sequence() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_processed_sequence_;
}

void AsyncDelayedMeasurementEskf::WaitUntilProcessed(
    std::uint64_t sequence) const {
  std::unique_lock<std::mutex> lock(mutex_);
  processed_cv_.wait(lock, [this, sequence]() {
    return latest_processed_sequence_ >= sequence;
  });
}

std::uint64_t AsyncDelayedMeasurementEskf::Enqueue(Event&& event) {
  std::uint64_t seq;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    seq = next_sequence_++;
    event.sequence = seq;
    queue_.push_back(std::move(event));
  }
  cv_.notify_one();
  return seq;
}

void AsyncDelayedMeasurementEskf::WorkerLoop() {
  while (true) {
    Event event;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() {
        return stop_requested_ || !queue_.empty();
      });
      if (stop_requested_ && queue_.empty()) {
        return;
      }
      event = std::move(queue_.front());
      queue_.pop_front();
    }

    switch (event.type) {
      case EventType::kInitializeDefault:
        eskf_.Initialize(event.timestamp_sec, event.state_value);
        break;
      case EventType::kInitializeExplicit:
        eskf_.Initialize(event.timestamp_sec, event.state_value,
                         event.state_covariance);
        break;
      case EventType::kControl:
        eskf_.AddControl(event.timestamp_sec, event.control_value,
                         event.control_covariance);
        break;
      case EventType::kMeasurement:
        eskf_.AddMeasurement(event.timestamp_sec, event.meas_value,
                             event.meas_covariance);
        break;
      case EventType::kInstantMeasurement:
        eskf_.AddInstantMeasurement(event.meas_value, event.meas_covariance);
        break;
      case EventType::kZeroVelocityUpdate:
        eskf_.AddZeroVelocityUpdate(event.zupt_covariance);
        break;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_initialized_ = eskf_.initialized();
      if (latest_initialized_) {
        latest_estimate_ = eskf_.latest_estimate();
      }
      latest_processed_sequence_ = event.sequence;
    }
    processed_cv_.notify_all();
  }
}

}  // namespace delayed_measurement_eskf
}  // namespace landmark_loc
