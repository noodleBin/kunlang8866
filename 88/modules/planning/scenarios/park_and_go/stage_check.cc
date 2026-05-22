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

#include "modules/planning/scenarios/park_and_go/stage_check.h"

namespace century {
namespace planning {
namespace scenario {
namespace park_and_go {

using century::common::TrajectoryPoint;

// process function
Stage::StageStatus ParkAndGoStageCheck::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Check";
  CHECK_NOTNULL(frame);

  // configure
  scenario_config_.CopyFrom(GetContext()->scenario_config);
  // set adc_init_position
  ADCInitStatus();
  // set is_on_open_space_trajectory
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  // plan
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStageCheck planning error";
    return StageStatus::ERROR;
  }

  // check if it is possible to start cruising
  bool ready_to_cruise = scenario::util::CheckADCReadyToCruise(
      injector_->vehicle_state(), frame, scenario_config_);
  return FinishStage(ready_to_cruise);
}

// finish stage check
Stage::StageStatus ParkAndGoStageCheck::FinishStage(const bool success) {
  if (success) {
    // transit to stage cruise
    next_stage_ = ScenarioConfig::PARK_AND_GO_CRUISE;
  } else {
    // transit to stage adjust
    next_stage_ = ScenarioConfig::PARK_AND_GO_ADJUST;
  }
  // update in_check_stage
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_park_and_go()
      ->set_in_check_stage(false);
  return Stage::FINISHED;
}

// set 'parking and go' adc_init_position
void ParkAndGoStageCheck::ADCInitStatus() {
  auto* park_and_go_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_park_and_go();
  park_and_go_status->Clear();
  // set adc_init_position
  park_and_go_status->mutable_adc_init_position()->set_x(
      injector_->vehicle_state()->x());
  park_and_go_status->mutable_adc_init_position()->set_y(
      injector_->vehicle_state()->y());
  park_and_go_status->mutable_adc_init_position()->set_z(0.0);
  park_and_go_status->set_adc_init_heading(
      injector_->vehicle_state()->heading());
  park_and_go_status->set_in_check_stage(true);
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace century
