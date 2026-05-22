/******************************************************************************
 * Copyright 2018 The century Authors. All Rights Reserved.
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
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_tracker.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_shape_filter.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_motion_filter.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_type_filter.h"

namespace century {
namespace perception {
namespace lidar {
namespace {

using CreateFunction = std::function<MlfBaseFilter*()>;
std::unordered_map<std::string, CreateFunction> filter_functor = {
  {"MlfShapeFilter", []() { return new MlfShapeFilter(); }},
  {"MlfMotionFilter", []() { return new MlfMotionFilter(); }},
  {"MlfTypeFilter", []() { return new MlfTypeFilter(); }}
};
MlfBaseFilter* CreateObject(const std::string& className) {
  auto it = filter_functor.find(className);
  if (it != filter_functor.end()) {
      return it->second(); // 调用对应的创建函数
  }
  return nullptr;
}
}

bool MlfTracker::Init(const MlfTrackerInitOptions options) {
  std::string config_file = "mlf_tracker.conf";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MlfTrackerConfig config;
  ACHECK(century::cyber::common::GetProtoFromFile(config_file, &config));


  for (int i = 0; i < config.filter_name_size(); ++i) {
    const auto& name = config.filter_name(i);
    MlfBaseFilter* filter = CreateObject(name);
    MlfFilterInitOptions filter_init_options;
   
    //MlfBaseFilterRegisterer::GetInstanceByName(name);
    ACHECK(filter);
   
    filter_init_options.config_path = options.config_path;
    ACHECK(filter->Init(filter_init_options));
    filters_.push_back(filter);
    AINFO << "MlfTracker add filter: " << filter->Name();
  }
  is_front_critical_ = config.is_front_critical();
  return true;
}

void MlfTracker::InitializeTrack(MlfTrackDataPtr new_track_data,
                                 TrackedObjectPtr new_object) {
  new_track_data->Reset(new_object, GetNextTrackId());
  new_track_data->is_current_state_predicted_ = false;
  new_track_data->is_front_critical_track_ = is_front_critical_;
}

void MlfTracker::UpdateTrackDataWithObject(MlfTrackDataPtr track_data,
                                           TrackedObjectPtr new_object) {
  // 1. state filter and store belief in new_object
  for (auto& filter : filters_) {
    filter->UpdateWithObject(filter_options_, track_data, new_object);
  }
  // 2. push new_obect to track_data
  track_data->PushTrackedObjectToTrack(new_object);
  if (new_object->updated_stable_direction) {
    if (new_object->output_direction != Eigen::Vector3d::Zero()) {
      track_data->stable_direction_  = new_object->stable_direction;
    }
  } 
  track_data->is_current_state_predicted_ = false;
  track_data->is_front_critical_track_ = is_front_critical_;
      
}

void MlfTracker::UpdateTrackDataWithoutObject(double timestamp,
                                              MlfTrackDataPtr track_data) {
  for (auto& filter : filters_) {
    filter->UpdateWithoutObject(filter_options_, timestamp, track_data);
  }
  AINFO << "Leave scene Track id: " << track_data->track_id_ << std::endl;
  track_data->is_current_state_predicted_ = true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace century
