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

#include "modules/planning/scenarios/park_and_go/stage_adjust.h"

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

namespace century {
namespace planning {
namespace scenario {
namespace park_and_go {

using century::common::TrajectoryPoint;

// process function
Stage::StageStatus ParkAndGoStageAdjust::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Adjust";
  CHECK_NOTNULL(frame);

  // scenario configure
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  // plan
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStageAdjust planning error";
    return StageStatus::ERROR;
  }
  // check if it is possible to crusing
  const bool is_ready_to_cruise = scenario::util::CheckADCReadyToCruise(
      injector_->vehicle_state(), frame, scenario_config_);

  bool is_end_of_trajectory = false;
  // previous frame
  const auto& history_frame = injector_->frame_history()->Latest();
  if (history_frame) {
    // trajectory
    const auto& trajectory_points =
        history_frame->current_frame_planned_trajectory().trajectory_point();
    // check if it is reached end point
    if (!trajectory_points.empty()) {
      is_end_of_trajectory =
          (trajectory_points.rbegin()->relative_time() < 0.0);
    }
  }

  // not finished park and go
  if (!is_ready_to_cruise && !is_end_of_trajectory) {
    return StageStatus::RUNNING;
  }
  return FinishStage();
}

// stage finish check
Stage::StageStatus ParkAndGoStageAdjust::FinishStage() {
  // vehicle state
  const auto vehicle_status = injector_->vehicle_state();
  ADEBUG << vehicle_status->steering_percentage();
  // check if the wheels have returned to the correct position
  if (std::fabs(vehicle_status->steering_percentage()) <
      scenario_config_.max_steering_percentage_when_cruise()) {
    // transit to stage cruise
    next_stage_ = ScenarioConfig::PARK_AND_GO_CRUISE;
  } else {
    // reset 'parking and go' adc_init_position
    ResetInitPostion();
    // transit to stage pre_cruise
    next_stage_ = ScenarioConfig::PARK_AND_GO_PRE_CRUISE;
  }
  return Stage::FINISHED;
}

// reset 'parking and go' adc_init_position
void ParkAndGoStageAdjust::ResetInitPostion() {
  auto* park_and_go_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_park_and_go();
  park_and_go_status->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  park_and_go_status->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  park_and_go_status->mutable_adc_init_position()->set_z(0.0);
  park_and_go_status->set_adc_init_heading(
      injector_->vehicle_state()->heading());
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace century
