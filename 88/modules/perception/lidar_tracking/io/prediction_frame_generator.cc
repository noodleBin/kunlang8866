#include "modules/perception/lidar_tracking/io/prediction_frame_generator.h"
namespace century {
namespace perception {
namespace lidar {
PredictionObstaclePtr PredictionFrameGenerator::RollOut() {
  auto prediction_msg = std::make_shared<PredictionObstacles>();
  for(int i=0; i<perception_msg_->perception_obstacle_size(); i++) {
    // fill in trajactory
    LidarPredictorFreeMove predictor(perception_msg_->perception_obstacle(i), yaw_rate_active_perception_);
    if(perception_msg_->mutable_perception_obstacle(i)->has_acceleration()) {
      perception_msg_->mutable_perception_obstacle(i)->mutable_acceleration()->set_z(0.0);  // acceleration_z is not used in prediction
    }
    auto traj = predictor.GeneratorTrajectory();
    auto cur_prediction =  prediction_msg->add_prediction_obstacle();
    auto cur_perception = new century::perception::PerceptionObstacle(perception_msg_->perception_obstacle(i));
    cur_prediction->set_allocated_perception_obstacle(cur_perception);
    std::vector<Point2D> line;
    if(traj) {
      cur_prediction->add_trajectory()->CopyFrom(*traj);
      for(int i=0; i< traj->trajectory_point_size(); ++i ) {
        line.push_back(Point2D(traj->trajectory_point(i).path_point().x(), 
                                traj->trajectory_point(i).path_point().y()));
      }
    }

    if(traj_converter_) {
      traj_converter_->AddLine(line, std::to_string(cur_perception->id()) +"_" 
                                    + std::to_string(cur_perception->type()) +"_"
                                    + std::to_string(cur_perception->timestamp()));
    }
    cur_prediction->set_is_static(!traj);
    cur_prediction->set_predicted_period(prediction_duration);
  }
  static uint32_t seq_num = 0; 
  prediction_msg->mutable_header()->set_timestamp_sec(perception_msg_->header().timestamp_sec());
  prediction_msg->mutable_header()->set_module_name("prediction");
  prediction_msg->set_is_loaded(false);
  prediction_msg->set_start_timestamp(perception_msg_->header().timestamp_sec());
  prediction_msg->set_end_timestamp(perception_msg_->header().timestamp_sec()+ prediction_duration);
  prediction_msg->mutable_header()->set_sequence_num(seq_num++);
  prediction_msg->set_perception_error_code(century::common::ErrorCode::OK);
  return prediction_msg;

}
}
}
}