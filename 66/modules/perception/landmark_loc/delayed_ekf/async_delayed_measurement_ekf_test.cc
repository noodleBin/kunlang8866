#include "async_delayed_measurement_ekf.h"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

namespace landmark_loc {
namespace delayed_ekf {
namespace {

using Vector3 = AsyncDelayedMeasurementEkf::Vector3;
TEST(AsyncDelayedMeasurementEkfTest, AcceptsConcurrentInputsAndConverges) {
  AsyncDelayedMeasurementEkf ekf;
  ekf.Initialize(0.0, Vector3::Zero(), Vector3(10.0, 10.0, 10.0));

  std::thread control_thread([&ekf]() {
    const Vector3 control_covariance(1e-4, 1e-4, 1e-4);
    for (int i = 1; i <= 30; ++i) {
      ekf.AddControl(0.1 * i, Vector3(1.0, 0.0, 0.0), control_covariance);
    }
  });

  std::thread measurement_thread([&ekf]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ekf.AddMeasurement(1.5, Vector3(1.2, 0.0, 0.0),
                       Vector3(1e-4, 1e-4, 1e-4));
  });

  control_thread.join();
  measurement_thread.join();
  ekf.WaitUntilProcessed(32);

  const auto estimate = ekf.latest_estimate();
  EXPECT_NEAR(estimate.timestamp_sec, 3.0, 1e-9);
  EXPECT_NEAR(estimate.state.x(), 2.7, 2e-2);
  EXPECT_NEAR(estimate.state.y(), 0.0, 1e-6);
}

TEST(AsyncDelayedMeasurementEkfTest, UsesDefaultControlCovarianceWhenOmitted) {
  AsyncDelayedMeasurementEkf ekf;
  const AsyncDelayedMeasurementEkf::Options options;
  ekf.Initialize(0.0, Vector3::Zero());

  ekf.AddControl(1.0, Vector3(1.0, 0.0, 0.0));
  ekf.WaitUntilProcessed(2);

  const auto estimate = ekf.latest_estimate();
  EXPECT_NEAR(estimate.timestamp_sec, 1.0, 1e-9);
  EXPECT_NEAR(estimate.state.x(), 1.0, 1e-6);
  EXPECT_NEAR(estimate.state.y(), 0.0, 1e-6);
  EXPECT_NEAR(estimate.state.z(), 0.0, 1e-6);
  EXPECT_NEAR(estimate.covariance(0, 0),
              options.default_initial_covariance(0) +
                  options.default_control_covariance(0),
              1e-6);
  EXPECT_NEAR(estimate.covariance(2, 2),
              options.default_initial_covariance(2) +
                  options.default_control_covariance(2),
              1e-6);
}

}  // namespace
}  // namespace delayed_ekf
}  // namespace landmark_loc
