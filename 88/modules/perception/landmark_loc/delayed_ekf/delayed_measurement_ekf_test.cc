#include "delayed_measurement_ekf.h"

#include <cmath>

#include <gtest/gtest.h>

namespace landmark_loc {
namespace delayed_ekf {
namespace {

using Vector3 = DelayedMeasurementEkf::Vector3;
constexpr double kHalfPi = 1.57079632679489661923;

TEST(DelayedMeasurementEkfTest, PropagatesBodyFrameControlsToWorldState) {
  DelayedMeasurementEkf ekf;
  ASSERT_TRUE(ekf.Initialize(0.0, Vector3(0.0, 0.0, kHalfPi)));

  ASSERT_TRUE(ekf.AddControl(1.0, Vector3(1.0, 0.0, 0.0),
                             Vector3(1e-4, 1e-4, 1e-4)));

  const auto estimate = ekf.latest_estimate();
  EXPECT_NEAR(estimate.timestamp_sec, 1.0, 1e-9);
  EXPECT_NEAR(estimate.state.x(), 0.0, 1e-6);
  EXPECT_NEAR(estimate.state.y(), 1.0, 1e-6);
  EXPECT_NEAR(estimate.state.z(), kHalfPi, 1e-6);
  EXPECT_GT(estimate.covariance(0, 0), 1e-4);
  EXPECT_NEAR(estimate.covariance(2, 2), 1.76e-4, 1e-6);
}

TEST(DelayedMeasurementEkfTest, UsesDefaultControlCovarianceWhenOmitted) {
  DelayedMeasurementEkf ekf;
  const DelayedMeasurementEkf::Options options;
  ASSERT_TRUE(ekf.Initialize(0.0, Vector3::Zero()));

  ASSERT_TRUE(ekf.AddControl(1.0, Vector3(1.0, 0.0, 0.0)));

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

TEST(DelayedMeasurementEkfTest, ReplaysControlsAfterDelayedMeasurement) {
  DelayedMeasurementEkf ekf;
  ASSERT_TRUE(ekf.Initialize(0.0, Vector3::Zero(), Vector3(10.0, 10.0, 10.0)));

  const Vector3 control_covariance(1e-4, 1e-4, 1e-4);
  ASSERT_TRUE(ekf.AddControl(1.0, Vector3(1.0, 0.0, 0.0), control_covariance));
  ASSERT_TRUE(ekf.AddControl(2.0, Vector3(1.0, 0.0, 0.0), control_covariance));
  ASSERT_TRUE(ekf.AddControl(3.0, Vector3(1.0, 0.0, 0.0), control_covariance));

  ASSERT_TRUE(ekf.AddMeasurement(1.5, Vector3(1.2, 0.0, 0.0),
                                 Vector3(1e-4, 1e-4, 1e-4)));

  const auto estimate = ekf.latest_estimate();
  EXPECT_NEAR(estimate.timestamp_sec, 3.0, 1e-9);
  EXPECT_NEAR(estimate.state.x(), 2.7, 2e-2);
  EXPECT_NEAR(estimate.state.y(), 0.0, 1e-6);
  EXPECT_NEAR(estimate.state.z(), 0.0, 1e-6);
  EXPECT_LT(estimate.covariance(0, 0), 1e-2);
}

}  // namespace
}  // namespace delayed_ekf
}  // namespace landmark_loc
