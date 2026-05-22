#include "modules/perception/lidar_tracking/io/lidar_predictor_free_move.h"
#include "modules/prediction/common/prediction_util.h"

namespace century {
namespace perception {
namespace lidar {
LidarPredictorFreeMove::LidarPredictorFreeMove(const century::perception::PerceptionObstacle& obstacle,
                                               const double yaw_rate_active_perception) {
  points_.clear();
  Eigen::Vector2d position (obstacle.position().x(), obstacle.position().y());
  Eigen::Vector2d velocity (obstacle.velocity().x(), obstacle.velocity().y());
  Eigen::Vector2d acc (obstacle.acceleration().x(), obstacle.acceleration().y());
  double theta = obstacle.theta();
  double start_time = 0.0; //obstacle.timestamp();
  yaw_rate_active_perception_ = yaw_rate_active_perception;
  if(std::fabs(yaw_rate_active_perception_) < 1e-6) {
    DrawFreeMoveTrajectoryPoints(position, velocity, acc, theta, start_time, total_time_, period_);
  } else {
    //obstacle.acceleration().z() means yaw rate
    DrawYawRateTrajectoryPoints(position, velocity, acc, obstacle.acceleration().z() ,start_time, total_time_, period_);
  }
}


void LidarPredictorFreeMove::DrawYawRateTrajectoryPoints(
  const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
  const Eigen::Vector2d& acc, const double omiga,
  const double start_time, const double total_time, const double period) {
  if(abs(velocity(0)) < 1e-6 && abs(velocity(1)) < 1e-6) {
    points_.clear();
    return;
  }
  Eigen::Matrix<double, 8, 1> state;
  state.setZero();
  state(0) = 0.0;
  state(1) = 0.0;
  state(2) = velocity(0);
  state(3) = velocity(1);
  state(4) = common::math::Clamp(acc(0), linear_min_acc_, linear_max_acc_);
  state(5) = common::math::Clamp(acc(1), linear_min_acc_, linear_max_acc_);
  state(6) = atan2(velocity(1), velocity(0));
  state(7) = omiga;

  Eigen::Matrix<double, 8, 8> transition;
  transition.setIdentity();
  transition(0, 2) = period;
  transition(0, 4) = 0.5 * period * period;
  transition(1, 3) = period;
  transition(1, 5) = 0.5 * period * period;
  transition(2, 4) = period;
  transition(3, 5) = period;
  transition(6, 7) = period;   

  ::century::prediction::predictor_util::GenerateFreeMoveTrajectoryPoints(
      &state, transition, start_time, total_time, period, yaw_rate_active_perception_, &points_);

  for (size_t i = 0; i < points_.size(); ++i) {
    ::century::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(points_.operator[](i)));
  }
}

void LidarPredictorFreeMove::DrawFreeMoveTrajectoryPoints(
  const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
  const Eigen::Vector2d& acc, const double theta, const double start_time,
  const double total_time, const double period) {

  
  if((std::abs(velocity(0)) < 1e-6) && (std::abs(velocity(1)) < 1e-6)) {
    points_.clear();
    return;
  }
  Eigen::Matrix<double, 6, 1> state;
  state.setZero();
  state(0, 0) = 0.0;
  state(1, 0) = 0.0;
  state(2, 0) = velocity(0);
  state(3, 0) = velocity(1);
  state(4, 0) = common::math::Clamp(acc(0), linear_min_acc_, linear_max_acc_);
  state(5, 0) = common::math::Clamp(acc(1), linear_min_acc_, linear_max_acc_);

  Eigen::Matrix<double, 6, 6> transition;
  transition.setIdentity();
  transition(0, 2) = period;
  transition(0, 4) = 0.5 * period * period;
  transition(1, 3) = period;
  transition(1, 5) = 0.5 * period * period;
  transition(2, 4) = period;
  transition(3, 5) = period;

  size_t num = static_cast<size_t>(total_time / period);
  ::century::prediction::predictor_util::GenerateFreeMoveTrajectoryPoints(
      &state, transition, theta, start_time, num, period, &points_);

  for (size_t i = 0; i < points_.size(); ++i) {
    ::century::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(points_.operator[](i)));
  }
}
std::shared_ptr<century::prediction::Trajectory> LidarPredictorFreeMove::GeneratorTrajectory() {
  if (points_.empty()) {
    return nullptr;
  }
  std::shared_ptr<century::prediction::Trajectory> out_put = std::make_shared<century::prediction::Trajectory>();
  for(auto& point : points_) {
    out_put->add_trajectory_point()->CopyFrom(point);
  }
  const double full_confidence = 1.0;
  out_put->set_probability(full_confidence);
  return out_put;

}
}
}
}

