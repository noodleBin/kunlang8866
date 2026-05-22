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

#include "modules/planning/scenarios/traffic_light/protected/stage_adjust.h"

#include <string>

#include "cyber/common/log.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace century {
namespace planning {
namespace scenario {
namespace traffic_light {

namespace {
constexpr double kStopSpeed = 0.05;  // m/s
}  // namespace

using century::common::TrajectoryPoint;
using century::hdmap::PathOverlap;
using century::perception::TrafficLight;

Stage::StageStatus TrafficLightProtectedStageAdjust::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);
  scenario_config_.CopyFrom(GetContext()->scenario_config);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  bool plan_ok = ExecuteTaskOnOpenSpace(frame);
  if (!plan_ok) {
    AERROR << "TrafficLigth Adjust planning error";
  }
  // end state is the same with origin approach ,
  // approach is consider to enter adjust
  const auto& reference_line_info = frame->reference_line_info().front();

  // to get from frame
  std::string traffic_id = frame->GetCurrentTrafficLightId();
  PathOverlap* current_traffic_light_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info, traffic_id,
                                                ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    AERROR << "!current_traffic_light_overlap"
           << "traffic_id is " << traffic_id;
    return FinishScenario();
  }
  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_to_stop_line =
      current_traffic_light_overlap->start_s - adc_front_edge_s;
  perception::TrafficLight traffic_light;
  frame->GetSignal(traffic_id, &traffic_light);

  if (distance_adc_to_stop_line >
      scenario_config_.start_traffic_light_scenario_distance()) {
    ADEBUG << "dist too far,the value is "
           << scenario_config_.start_traffic_light_scenario_distance();
    return FinishScenario();
  }

  const auto& signal_color = traffic_light.color();
  AINFO << "traffic_light_overlap_id[" << traffic_id << "] start_s["
        << current_traffic_light_overlap->start_s
        << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
        << "] color[" << signal_color << "]";
  // || parking finisher ,return FinishScenario
  if (TrafficLight::GREEN == signal_color) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightProtectedStageAdjust::FinishStage() {
  if (std::fabs(injector_->vehicle_state()->steering_percentage()) >
      FLAGS_teb_return_angle_limit) {
    double adc_speed = std::fabs(
        injector_->vehicle_state()->vehicle_state().linear_velocity());
    if (adc_speed > kStopSpeed) {
      return Stage::RUNNING;
    } else {
      AINFO << "need send empty traj for steering_percentage < limit";
      return StageStatus::ERROR;
    }
  }

  auto* traffic_light = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_traffic_light();
  traffic_light->clear_done_traffic_light_overlap_id();
  for (const auto& traffic_light_overlap_id :
       GetContext()->current_traffic_light_overlap_ids) {
    traffic_light->add_done_traffic_light_overlap_id(traffic_light_overlap_id);
  }

  next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace century
