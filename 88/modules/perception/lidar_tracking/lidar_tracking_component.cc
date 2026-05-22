/******************************************************************************
 * Copyright 2023 The century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/lidar_tracking/lidar_tracking_component.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/canbus/proto/chassis.pb.h"

//#include "cyber/profiler/profiler.h"
#include "cyber/time/clock.h"

namespace century {
namespace perception {
namespace lidar {

using century::cyber::Clock;

bool LidarTrackingComponent::Init() {
  
  LidarTrackingComponentConfig comp_config;
  if (!century::cyber::common::GetProtoFromFile("/century/modules/perception/lidar_tracking/conf/lidar_tracking_config.pb.txt", &comp_config)) {
    AERROR << "Get LidarTrackingComponentConfig file failed";
    return false;
  }
  AINFO << "Lidar Tracking Component Configs: " << comp_config.DebugString();

  MultiTargetTrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = "/century/modules/perception/lidar_tracking/data/tracking";
  if(use_fase_tracker_) {
    multi_target_tracker_ = new TrackerEngine();
    tracker_init_options.config_file = "track_config.yaml";
  } else {
    multi_target_tracker_ = new MlfEngine();
    tracker_init_options.config_file = "mlf_engine.pb.txt";
  }
 
  // multi target tracking init

  ACHECK(multi_target_tracker_->Init(tracker_init_options));

  // // fused classifier init
  // auto fusion_classifier_param = comp_config.fusion_classifier_param();
  // std::string fusion_classifier_name = fusion_classifier_param.name();
  // fusion_classifier_ =
  //     BaseClassifierRegisterer::GetInstanceByName(fusion_classifier_name);
  // CHECK_NOTNULL(fusion_classifier_);

  // ClassifierInitOptions fusion_classifier_init_options;
  // fusion_classifier_init_options.config_path =
  //     fusion_classifier_param.config_path();
  // fusion_classifier_init_options.config_file =
  //     fusion_classifier_param.config_file();
  // ACHECK(fusion_classifier_->Init(fusion_classifier_init_options));
  return true;
}

std::shared_ptr<SensorFrameMessage> LidarTrackingComponent::GetSensorFrameMessage() const {
  return out_message_;
}

bool LidarTrackingComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  //PERF_FUNCTION()
  AINFO << std::setprecision(16)
        << "Enter LidarTracking component, message timestamp: "
        << message->timestamp_
        << " current timestamp: " << Clock::NowInSeconds() << std::endl;

 // auto out_message = std::make_shared<SensorFrameMessage>();
 
  if (InternalProc(message)) {
    return true;
  }

  AERROR << "Send lidar tracking output message failed!";
  return false;
}

bool LidarTrackingComponent::InternalProc( const std::shared_ptr<const LidarFrameMessage>& in_message) {
  AINFO << "Lidar Tracking: Start InternalProc" << std::endl;
  out_message_.reset(new SensorFrameMessage());
  out_message_->timestamp_ = in_message->timestamp_;
  out_message_->lidar_timestamp_ = in_message->lidar_timestamp_;
  out_message_->seq_num_ = in_message->seq_num_;
  out_message_->process_stage_ = onboard::ProcessStage::LIDAR_RECOGNITION;
  out_message_->sensor_id_ = in_message->lidar_frame_->sensor_info.name;

  if (in_message->error_code_ != century::common::ErrorCode::OK) {
    out_message_->error_code_ = in_message->error_code_;
    AERROR << "Lidar tracking receive message with error code, skip it.";
    return true;
  }

  auto& lidar_frame = in_message->lidar_frame_;
  AINFO << "message object size: " << lidar_frame.get()->segmented_objects.size()<< std::endl;
  // multi target tracker
  //PERF_BLOCK("multi_target_tracker")
 
  MultiTargetTrackerOptions tracker_options;
 
  if (!multi_target_tracker_->Track(tracker_options, lidar_frame.get())) {
    AINFO << "Lidar tracking, multi_target_tracker_ Track error.";
    return false;
  }
  //PERF_BLOCK_END

  // // fused classifer
  // PERF_BLOCK("fusion_classifier")
  // ClassifierOptions fusion_classifier_options;
  // if (!fusion_classifier_->Classify(fusion_classifier_options,
  //                                   lidar_frame.get())) {
  //   AERROR << "Lidar tracking, fusion_classifier_ Classify error.";
  //   return false;
  // }
  // PERF_BLOCK_END

  // get out_message
  out_message_->hdmap_ = lidar_frame->hdmap_struct;
  auto& frame = out_message_->frame_;
  frame = base::FramePool::Instance().Get();
  frame->sensor_info = lidar_frame->sensor_info;
  frame->timestamp = in_message->timestamp_;
  frame->objects = lidar_frame->tracked_objects;
  frame->sensor2world_pose = lidar_frame->lidar2world_pose;
  frame->lidar_frame_supplement.on_use = true;
  frame->lidar_frame_supplement.cloud_ptr = lidar_frame->cloud;

  const double end_timestamp = Clock::NowInSeconds();
  const double end_latency = (end_timestamp - in_message->timestamp_) * 1e3;
  AINFO << std::setprecision(16)
        << "FRAME_STATISTICS:LidarTracking:End:msg_time["
        << in_message->timestamp_ << "]:cur_time[" << end_timestamp
        << "]:cur_latency[" << end_latency << "]";

  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace century
