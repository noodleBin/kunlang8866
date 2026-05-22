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

#include "modules/planning/scenarios/traffic_light/protected/stage_approach.h"

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
constexpr double KStopSpeedThreshold = 0.1;               // m/s
constexpr int KMatchTime = 5;                             // m/s
constexpr double KAdjustYawThreshold = 8.0 * M_PI / 180;  // rad
constexpr double kStoplinePoseLatOffset = -0.3;
}  // namespace

using century::common::TrajectoryPoint;
using century::common::math::Polygon2d;
using century::common::math::Vec2d;
using century::hdmap::PathOverlap;
using century::perception::TrafficLight;

Stage::StageStatus TrafficLightProtectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);
  scenario_config_.CopyFrom(GetContext()->scenario_config);
  ExecuteTaskOnReferenceLine(planning_init_point, frame);

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

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
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  const double distance_adc_to_stop_line =
      current_traffic_light_overlap->start_s - adc_front_edge_s;
  // check distance to stop line
  if (distance_adc_to_stop_line > scenario_config_.max_valid_stop_distance()) {
    return Stage::RUNNING;
  }
  perception::TrafficLight traffic_light;
  frame->GetSignal(traffic_id, &traffic_light);

  // Update stopline_sl into planning-context.
  common::SLPoint stopline_sl;
  const double front_to_axis =
      common::VehicleConfigHelper::GetConfig().vehicle_param().length() -
      common::VehicleConfigHelper::GetConfig()
          .vehicle_param()
          .back_edge_to_center();
  stopline_sl.set_s(current_traffic_light_overlap->start_s - front_to_axis);
  stopline_sl.set_l(kStoplinePoseLatOffset);
  common::math::Vec2d stopline_xy;
  reference_line_info.reference_line().SLToXY(stopline_sl, &stopline_xy);
  auto* adjust_pose = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_traffic_light()
                          ->mutable_adc_adjust_end_pose();
  adjust_pose->set_x(stopline_xy.x());
  adjust_pose->set_y(stopline_xy.y());

  // pose for adjust
  auto* traffic_light_pose = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_traffic_light()
                                 ->mutable_traffic_light_pose();

  const auto adc_speed =
      injector_->vehicle_state()->vehicle_state().linear_velocity();

  const auto& signal_color = traffic_light.color();
  AINFO << "traffic_light_overlap_id[" << traffic_id << "] start_s["
        << current_traffic_light_overlap->start_s
        << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
        << "] color[" << signal_color << "]";
  if (current_traffic_light_overlap->end_s <= adc_back_edge_s) {
    return FinishStage();
  }
  // pose for adjust
  if (((frame->GetSignalCenterFromId(traffic_id, traffic_light_pose))) &&
      std::fabs(adc_speed) < KStopSpeedThreshold) {
    // test position
    //  traffic_light_pose->set_x(625278);
    //  traffic_light_pose->set_y(3420337);

    ++match_count_;
    const double target_yaw =
        std::atan2(traffic_light_pose->y() - injector_->vehicle_state()->y(),
                   traffic_light_pose->x() - injector_->vehicle_state()->x());

    const double yaw_error = common::math::NormalizeAngle(
        target_yaw - injector_->vehicle_state()->heading());
    if (match_count_ > KMatchTime &&
        std::fabs(yaw_error) > KAdjustYawThreshold) {
      SetAdjustStageInfo();
      return Stage::FINISHED;
    }
  } else {
    match_count_ = 0;
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightProtectedStageApproach::FinishStage() {
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

void TrafficLightProtectedStageApproach::SetAdjustStageInfo() {
  next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_ADJUST;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace century
