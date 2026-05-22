/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/mcloud/proto/super_traffic_light.pb.h"
#include "modules/mcloud/proto/mcloud_info.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"
#include "modules/planning/proto/lane_borrow_response.pb.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"
#include "modules/planning/proto/stackers_info.pb.h"
#include "modules/planning/proto/v2x_info.pb.h"
#include "modules/planning/proto/planning_aeb.pb.h"
#include "modules/planning/proto/blocking_area_response.pb.h"
#include "modules/planning/proto/temporary_parking_request.pb.h"
#include "modules/planning/proto/barrier.pb.h"
#include "modules/fas_aeb_backend/proto/fas_aeb_backend.pb.h"
#include "modules/dreamview/proto/background_music.pb.h"

namespace century {
namespace planning {

/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<perception::PerceptionObstacles> perception_aeb_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  std::shared_ptr<mcloud::SuperTrafficLight> super_traffic_light;
  std::shared_ptr<routing::RoutingResponse> routing;
  std::shared_ptr<relative_map::MapMsg> relative_map;
  std::shared_ptr<PadMessage> pad_msg;
  std::shared_ptr<storytelling::Stories> stories;
  std::shared_ptr<mcloud::McloudInfo> cloud_info;
  std::shared_ptr<planning::BorrowResponse> borrow_response;
  std::shared_ptr<planning::PassStackerResponse> pass_stacker_response;
  std::shared_ptr<planning::StackersInfo> stackers_info;
  std::shared_ptr<planning::V2xInfo> v2x_info;
  std::shared_ptr<planning::BlockingAreaResponse> blocking_area_response;
  std::shared_ptr<planning::TemporaryParkingRequest> temporary_parking_request;
  std::shared_ptr<planning::TemporaryParkingRequest> multi_path_temp_stop_request;
  std::shared_ptr<planning::Barrier> barrier;
  std::shared_ptr<fas_aeb_backend::FasAebInfo> fas_aeb_result;
  std::shared_ptr<dreamview::BackgroundMusic> background_music;
};

}  // namespace planning
}  // namespace century
