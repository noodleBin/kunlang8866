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

/**
 * @file
 **/
#include "modules/planning/scenarios/dead_end/deadend_turnaround/stage_turning.h"

namespace century {
namespace planning {
namespace scenario {
namespace deadend_turnaround {

Stage::StageStatus StageTurning::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "StageTurning planning error";
    return StageStatus::ERROR;
  }

  const common::PointENU& target_point = frame->local_view()
                                             .routing->routing_request()
                                             .dead_end_info()
                                             .target_point();
  const common::VehicleState& car_position = frame->vehicle_state();
  // transfer dead_end to lane follow, should enhance transfer logic
  bool reached_target_point = JudgeReachTargetPoint(car_position, target_point);
  bool reached_destination =
      frame->mutable_open_space_info()->destination_reached();
  ADEBUG << "reached_target_point: " << reached_target_point;
  ADEBUG << "reached_destination: " << reached_destination;
  if (reached_target_point || reached_destination) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    frame->mutable_open_space_info()->set_destination_reached(true);
    ADEBUG << "StageTurning set_destination_reached.";
    return FinishStage();
  }

  return StageStatus::RUNNING;
}

bool StageTurning::JudgeReachTargetPoint(
    const common::VehicleState& car_position,
    const common::PointENU& target_point) {
  double distance_to_vehicle =
      std::sqrt((car_position.x() - target_point.x()) *
                    (car_position.x() - target_point.x()) +
                (car_position.y() - target_point.y()) *
                    (car_position.y() - target_point.y()));

  ADEBUG << "distance_to_vehicle: " << distance_to_vehicle;
  ADEBUG << "FLAGS_open_space_threshold_distance_for_destination: "
         << FLAGS_open_space_threshold_distance_for_destination;
  return distance_to_vehicle <
         FLAGS_open_space_threshold_distance_for_destination;
}

Stage::StageStatus StageTurning::FinishStage() { return Stage::FINISHED; }

}  // namespace deadend_turnaround
}  // namespace scenario
}  // namespace planning
}  // namespace century
