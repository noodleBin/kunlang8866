#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <list>
#include <memory>
#include <iostream>
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/perception/lidar_tracking/io/lidar_frame_generator.h"
#include "modules/perception/lidar_tracking/io/lidar_predictor_free_move.h"
#include "modules/perception/lidar_tracking/io/geojson_convert.h"
#include "cyber/common/file.h"
namespace century {
namespace perception {
namespace lidar {
class PredictionFrameGenerator {
  using PerceptionObstacleptr = std::shared_ptr<PerceptionObstacles>;
  using PredictionObstaclePtr = std::shared_ptr<PredictionObstacles>;
  using LidarPredictorFreeMovePtr = std::shared_ptr<LidarPredictorFreeMove>;
  public:
    PredictionFrameGenerator() = default;
    explicit PredictionFrameGenerator(const PerceptionObstacleptr msg, 
                                      const double yaw_rate_active_perception,
                                      GeoJsonConvert* traj_converter = nullptr) {
      ACHECK(msg != nullptr);
      perception_msg_ = msg;
      yaw_rate_active_perception_ = yaw_rate_active_perception;
      traj_converter_ = traj_converter;
    } 
     
    virtual ~PredictionFrameGenerator() = default;
    PredictionObstaclePtr RollOut();

  private:
  PerceptionObstacleptr perception_msg_;
  double yaw_rate_active_perception_ = 0.0;
  GeoJsonConvert* traj_converter_ = nullptr;
  const double prediction_duration = 5.0;
 
};

}
}
}
