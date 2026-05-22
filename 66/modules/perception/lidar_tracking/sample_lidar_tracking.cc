
#include <iostream>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <chrono>
#include <thread>
#include "modules/perception/lidar_tracking/sample_lidar_tracking.h"
#include "modules/perception/lidar_tracking/io/lidar_frame_generator.h"
#include "modules/perception/lidar_tracking/tracker/common/convert.h"
#include "modules/perception/lidar_tracking/io/lidar_predictor_free_move.h"
#include "modules/perception/lidar_tracking/io/geojson_convert.h"
#include "modules/perception/lidar_tracking/proto/lidar_tracking_node_config.pb.h"
#include "modules/perception/lidar_tracking/io/duplicated_object_filter.h"
#include "modules/perception/lidar_tracking/io/dump_perception.h"
#include "modules/perception/lidar_tracking/io/prediction_frame_generator.h"

using namespace century::perception::lidar;

int main(int argc, char** argv) {

  std::string config_file = "/century/modules/perception/lidar_tracking/conf/lidar_tracking_node_config.pb.txt";
  if(argc == 2) {
    config_file = argv[1];
  }
  century::cyber::Init(argv[0]);
  LidarTrackingNodeConfig node_config;
  if (!century::cyber::common::GetProtoFromFile(config_file, &node_config)) {
    AERROR << "Get LidarTrackingNodeConfig file failed: " << config_file << std::endl;
    return false;
  }
  FLAGS_minloglevel = google::INFO; 
  FLAGS_alsologtostderr = true; 
  AINFO << node_config.DebugString() << std::endl;

  auto talker_node = century::cyber::CreateNode(argv[0]);

  const std::string perception_channel_name = node_config.input_perception_detection_channel_name();
  const std::string localization_channel_name =  node_config.input_localization_channel_name();
  const std::string perception_output_channel_name = node_config.output_perception_channel_name();
  const std::string prediction_output_channel_name = node_config.output_prediction_channel_name();
  const bool enable_predictor_dump = node_config.enable_dump_predictor();
  const bool local_ecef_frame = node_config.local_ecef_frame();
  const double duplicated_distance_thres = node_config.duplicated_distance_thres();
  const uint32_t frame_wait_seconds = node_config.frame_wait_seconds();
  const std::string log_path = node_config.log_path();
  const double yaw_rate_active_perception = node_config.yaw_rate_active_perception();
  const double using_point_cloud_polygon_thres = node_config.using_point_cloud_polygon_thres();
  bool dump_perception_input = node_config.dump_perception_input();
  bool dump_prediction_output = node_config.dump_prediction_output();
  const double min_length_threshold = node_config.min_length_threshold();
  const double height_diff_threshold = node_config.height_diff_threshold();
  perception_module_name = node_config.perception_module_name();
  localization_module_name = node_config.localization_module_name();
  const double convariance_thres = node_config.convariance_thres();
  const double mean_shift_thres = node_config.mean_shift_thres();
  const bool use_detector_pose = node_config.use_detector_pose();
 
  if(!log_path.empty()) {
    FLAGS_log_dir = log_path;  
  }
  // writer
  auto perception_writer = talker_node->CreateWriter<PerceptionObstacles>(perception_output_channel_name);

  auto prediction_writer = talker_node->CreateWriter<PredictionObstacles>(prediction_output_channel_name);

  if(!perception_writer || !prediction_writer) {
    AERROR << "Create writer failed." << std::endl;
    return false;
  }
  
  // reader

  PerceptionObstacleReaderPtr perception_reader_ = talker_node->CreateReader<PerceptionObstacles>(
                                                    perception_channel_name,
                                                    ReceiveMessageCallback<PerceptionObstacleptr>);
  LocalizationReaderPtr localization_reader_;
  if (!use_detector_pose) {
    localization_reader_ = talker_node->CreateReader<LocalizationEstimate>(
                                                    localization_channel_name,
                                                    ReceiveMessageCallback<LocalizationPtr>);
    if (!localization_reader_) {
      return false;
    }
  }

  if(!perception_reader_) {
    AERROR << "Create reader failed." << std::endl;
    return false;
  }
  Eigen::Affine3d extrinsic = Eigen::Affine3d::Identity(); // to do .. set extrinsic
  LidarFrameGenerator lidar_frame_generator(extrinsic, duplicated_distance_thres, 
                                            using_point_cloud_polygon_thres, min_length_threshold, 
                                            height_diff_threshold, local_ecef_frame, use_detector_pose);
  
  century::perception::lidar::LidarTrackingComponent component;
  component.Init();

  GeoJsonConvert* traj_converter = nullptr;

  ProducerConsumerQueue<PerceptionObstacleptr>::GetInstance()->Init(frame_wait_seconds);
  VelocityFilter::GetInstance()->Init(convariance_thres, mean_shift_thres);
  while(1) {
    auto  cur_perception_data = ProducerConsumerQueue<PerceptionObstacleptr>::GetInstance()->Consume();
    
    if(cur_perception_data.empty()) {
      AINFO<<"prediction data is empty" << std::endl;
      break;
    }
    if(dump_perception_input) {
      for(const auto& cur_perception : cur_perception_data) {
        century::perception::DumpPerceptionObstacles(*cur_perception, "input",cur_perception->header().timestamp_sec());
      }
    }
    lidar_frame_generator.SplicePerceptionData(cur_perception_data);
    if (!use_detector_pose) {
      std::list<LocalizationPtr> fetch_dr;
      WriteOnceReadAllQueue<LocalizationPtr>::GetInstance()->ReadAll(fetch_dr);
      if(fetch_dr.empty()) {
        continue;
      }
      lidar_frame_generator.SpliceLocalizationEstimate(fetch_dr);
    }

    while(1) {
      auto lidar_frame = lidar_frame_generator.GenerateFrame();
      if(!lidar_frame) {
        break;
      }

      if(lidar_frame->lidar_frame_->segmented_objects.empty()) {
        AINFO << "empty lidar frame generated:" << lidar_frame->timestamp_ << std::endl;
        PerceptionObstacleptr perception_msg = std::make_shared<PerceptionObstacles>();
        perception_msg->mutable_header()->set_timestamp_sec(lidar_frame->timestamp_);
        PredictionFrameGenerator preception_generator(perception_msg, 0.0);
        auto prediction_msg = preception_generator.RollOut();
        perception_writer->Write(perception_msg);
        prediction_writer->Write(prediction_msg); 
        continue;
      }
   
      AINFO << "lidar frame generated:" << lidar_frame->timestamp_ << std::endl;
      if(enable_predictor_dump) {
        traj_converter = new GeoJsonConvert(lidar_frame->timestamp_, "trajectory");
        traj_converter->AddPoint(lidar_frame->lidar_frame_->lidar2world_pose.translation()(0),
                                 lidar_frame->lidar_frame_->lidar2world_pose.translation()(1),
                                 "vehicle_" + std::to_string(lidar_frame->timestamp_));
      }
      
      if(component.Process(lidar_frame)) {
        auto output_frame = component.GetSensorFrameMessage(); 
        SensorFrameMessagePostProcess(output_frame, using_point_cloud_polygon_thres);
        PerceptionObstacleptr perception_msg = std::make_shared<PerceptionObstacles>();
        perception_msg->mutable_header()->set_timestamp_sec(output_frame->timestamp_);
        if(ConvertSensorFrameMessage2Obstacles(output_frame, perception_msg.get())) {
            
          PredictionFrameGenerator preception_generator(perception_msg, yaw_rate_active_perception, traj_converter);
          auto prediction_msg = preception_generator.RollOut();
          if(dump_prediction_output) {
            century::perception::DumpPerceptionObstacles(*perception_msg, "output", prediction_msg->header().timestamp_sec());
          }
          
          perception_writer->Write(perception_msg);
          prediction_writer->Write(prediction_msg); 
         
        }
       
      }  
      if(traj_converter) {
        traj_converter->DumpTraj();
      }  
    }
  }

  perception_writer.reset();
  prediction_writer.reset();
  if (!use_detector_pose) {
    localization_reader_.reset();
  }
  perception_reader_.reset();
  talker_node.reset();
 
  //century::cyber::WaitForShutdown();
 
  return 0;
}