/******************************************************************************
 * Copyright 2019 The Century Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"

#include "modules/planning/scenarios/park/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/park/valet_parking/stage_parking.h"

#include <float.h>

namespace century {
namespace planning {
namespace scenario {
namespace valet_parking {

using century::common::VehicleState;
using century::common::math::Vec2d;
using century::hdmap::ParkingSpaceInfoConstPtr;
using century::hdmap::Path;
using century::hdmap::PathOverlap;

century::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    ValetParkingScenario::s_stage_factory_;

void ValetParkingScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
}

void ValetParkingScenario::RegisterStages() {
  if (s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::VALET_PARKING_APPROACHING_PARKING_SPOT,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageApproachingParkingSpot(config, injector);
      });
  s_stage_factory_.Register(
      ScenarioConfig::VALET_PARKING_PARKING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageParking(config, injector);
      });
}

std::unique_ptr<Stage> ValetParkingScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool ValetParkingScenario::GetScenarioConfig() {
  if (!config_.has_valet_parking_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.valet_parking_config());
  return true;
}

// check whether it is necessary to switch to the parking space scenario
bool ValetParkingScenario::IsTransferable(const Frame& frame,
                                          const double parking_start_range) {
  // TODO(all) Implement available parking spot detection by preception results
  std::string target_parking_spot_id;
  if (frame.local_view().routing->routing_request().has_parking_info() &&
      frame.local_view()
          .routing->routing_request()
          .parking_info()
          .has_parking_space_id()) {
    target_parking_spot_id = frame.local_view()
                                 .routing->routing_request()
                                 .parking_info()
                                 .parking_space_id();
  } else {
    ADEBUG << "No parking space id from routing";
    return false;
  }

  // not set target parking space
  if (target_parking_spot_id.empty()) {
    return false;
  }

  // reference line
  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  PathOverlap parking_space_overlap;
  const auto& vehicle_state = frame.vehicle_state();

  // check whether the target parking space is on the nearby_path
  if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id,
                                     &parking_space_overlap)) {
    ADEBUG << "No such parking spot found after searching all path forward "
              "possible"
           << target_parking_spot_id;
    return false;
  }

  // check whether the distance between the target parking space and
  // the vehicle current position is smaller than parking_start_range
  if (!CheckDistanceToParkingSpot(frame, vehicle_state, nearby_path,
                                  parking_start_range, parking_space_overlap)) {
    ADEBUG << "target parking spot found, but too far, distance larger than "
              "pre-defined distance"
           << target_parking_spot_id;
    return false;
  }

  return true;
}

// check whether the parking space is on the path
bool ValetParkingScenario::SearchTargetParkingSpotOnPath(
    const Path& nearby_path, const std::string& target_parking_id,
    PathOverlap* parking_space_overlap) {
  // parking spaces list in the path
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  for (const auto& parking_overlap : parking_space_overlaps) {
    // find target parking space
    if (parking_overlap.object_id == target_parking_id) {
      *parking_space_overlap = parking_overlap;
      return true;
    }
  }
  return false;
}

// check whether the distance between the parking space and the vehicle position
// is smaller than parking_start_range
bool ValetParkingScenario::CheckDistanceToParkingSpot(
    const Frame& frame,
    const VehicleState& vehicle_state, const Path& nearby_path,
    const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  // hdmap
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  id.set_id(parking_space_overlap.object_id);
  // get target parking space information
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);

  double parking_space_s = DBL_MAX;
  for (auto corner_point : target_parking_spot_ptr->polygon().points()) {
    double tmp_s = 0.0, tmp_l = 0.0;
    nearby_path.GetNearestPoint(corner_point, &tmp_s, &tmp_l);
    parking_space_s = std::fmin(parking_space_s, tmp_s);
  }

  // vehicle s
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);

  if (std::fabs(parking_space_s - vehicle_point_s) <
      parking_start_range) {
    return true;
  } else {
    return false;
  }
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace century
