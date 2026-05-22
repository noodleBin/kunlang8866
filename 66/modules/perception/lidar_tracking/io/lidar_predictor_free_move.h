#pragma once
#include "modules/prediction/predictor/free_move/free_move_predictor.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
namespace century {
namespace perception {
namespace lidar {

class LidarPredictorFreeMove {
public:
  LidarPredictorFreeMove() = default;
  explicit LidarPredictorFreeMove(const century::perception::PerceptionObstacle& obstacle, 
                                  const double yaw_rate_active_perception);
  std::shared_ptr<century::prediction::Trajectory> GeneratorTrajectory();
  virtual ~LidarPredictorFreeMove() = default;
private:
void DrawYawRateTrajectoryPoints(
  const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
  const Eigen::Vector2d& acc, const double omiga,
  const double start_time, const double total_time, const double period);
void DrawFreeMoveTrajectoryPoints(
  const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
  const Eigen::Vector2d& acc, const double theta, const double start_time,
  const double total_time, const double period);
  std::vector<century::common::TrajectoryPoint> points_;
  const double period_ = 0.1;
  const double total_time_ = 5.0;
  const double linear_max_acc_ = 4.0;
  const double linear_min_acc_ = -4.0;
  double yaw_rate_active_perception_ = 0.0;
};
}
}
}