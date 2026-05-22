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

#include "modules/planning/scenarios/yield_sign/stage_approach.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace century {
namespace planning {
namespace scenario {
namespace yield_sign {

namespace {

constexpr double kepsilon = 1e-6;
constexpr double kMinSTBoundaryT = 6.0;
constexpr double kIgnoreMaxSTMinT = 0.1;
constexpr double kIgnoreMinSTMinS = 15.0;
constexpr double kPassStopLineBuffer = 0.3;  // unit: m

}  // namespace

using century::common::TrajectoryPoint;
using century::cyber::Clock;
using century::hdmap::PathOverlap;

Stage::StageStatus YieldSignStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);
  if (!ExecuteTaskOnReferenceLine(planning_init_point, frame)) {
    AERROR << "YieldSignStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  if (GetContext()->current_yield_sign_overlap_ids.empty()) {
    return FinishScenario();
  }

  for (const auto& yield_sign_overlap_id :
       GetContext()->current_yield_sign_overlap_ids) {
    // get overlap along reference line
    PathOverlap* current_yield_sign_overlap =
        scenario::util::GetOverlapOnReferenceLine(
            reference_line_info, yield_sign_overlap_id,
            ReferenceLineInfo::YIELD_SIGN);
    if (!current_yield_sign_overlap) {
      continue;
    }

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_yield_sign_overlap->start_s, false);
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance_adc_pass_stop_sign =
        adc_front_edge_s - current_yield_sign_overlap->start_s;
    if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
      // passed stop line
      return FinishStage();
    }

    const double distance_adc_to_stop_line =
        current_yield_sign_overlap->start_s - adc_front_edge_s;
    ADEBUG << "yield_sign_overlap_id[" << yield_sign_overlap_id << "] start_s["
           << current_yield_sign_overlap->start_s
           << "] distance_adc_to_stop_line[" << distance_adc_to_stop_line
           << "]";
    bool yield_sign_done = false;
    if (distance_adc_to_stop_line <
        scenario_config_.max_valid_stop_distance()) {
      // close enough, check yield_sign clear
      yield_sign_done = CheckYieldDone(frame);

      // lwt20230830: note origin code

      // const auto& path_decision = reference_line_info.path_decision();
      // for (const auto* obstacle : path_decision.obstacles().Items()) {
      //   const std::string& obstacle_id = obstacle->Id();
      //   std::string obstacle_type_name =
      //       PerceptionObstacle_Type_Name(obstacle->Perception().type());
      //   ADEBUG << "yield_sign[" << yield_sign_overlap_id << "] obstacle_id["
      //          << obstacle_id << "] type[" << obstacle_type_name << "]";
      //   if (obstacle->IsVirtual() ||
      //       obstacle->reference_line_st_boundary().IsEmpty() ||
      //       obstacle->reference_line_st_boundary().min_t() > kMinSTBoundaryT)
      //       {
      //     continue;
      //   }

      //   double obstacle_traveled_s =
      //       obstacle->reference_line_st_boundary().bottom_left_point().s() -
      //       obstacle->reference_line_st_boundary().bottom_right_point().s();
      //   ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
      //          << obstacle->reference_line_st_boundary().min_t()
      //          << "] obstacle_st_min_s["
      //          << obstacle->reference_line_st_boundary().min_s()
      //          << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

      //   // ignore the obstacle which is already on reference line and moving
      //   // along the direction of ADC

      //   if (obstacle_traveled_s < kepsilon &&
      //       obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT
      //       && obstacle->reference_line_st_boundary().min_s() >
      //       kIgnoreMinSTMinS) {
      //     continue;
      //   }
      //   injector_->planning_context()
      //       ->mutable_planning_status()
      //       ->mutable_yield_sign()
      //       ->add_wait_for_obstacle_id(obstacle->Id());

      //   yield_sign_done = false;
      // }
    }

    if (yield_sign_done) {
      return FinishStage();
    }
  }

  return Stage::RUNNING;
}

Stage::StageStatus YieldSignStageApproach::FinishStage() {
  // update PlanningContext
  auto* yield_sign_status = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_yield_sign();

  yield_sign_status->mutable_done_yield_sign_overlap_id()->Clear();
  for (const auto& yield_sign_overlap_id :
       GetContext()->current_yield_sign_overlap_ids) {
    yield_sign_status->add_done_yield_sign_overlap_id(yield_sign_overlap_id);
  }
  yield_sign_status->clear_wait_for_obstacle_id();

  GetContext()->creep_start_time = Clock::NowInSeconds();

  next_stage_ = ScenarioConfig::YIELD_SIGN_CREEP;
  return Stage::FINISHED;
}

bool YieldSignStageApproach::CheckYieldDone(const Frame* frame) {
  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& path_decision = reference_line_info.path_decision();
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    const std::string& obstacle_id = obstacle->Id();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle->Perception().type());
    ADEBUG << "yield_sign["
           << "] obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]";
    if (obstacle->IsVirtual() ||
        obstacle->reference_line_st_boundary().IsEmpty() ||
        obstacle->reference_line_st_boundary().min_t() > kMinSTBoundaryT) {
      continue;
    }

    double obstacle_traveled_s =
        obstacle->reference_line_st_boundary().bottom_left_point().s() -
        obstacle->reference_line_st_boundary().bottom_right_point().s();
    ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
           << obstacle->reference_line_st_boundary().min_t()
           << "] obstacle_st_min_s["
           << obstacle->reference_line_st_boundary().min_s()
           << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

    // ignore the obstacle which is already on reference line and moving
    // along the direction of ADC

    if (obstacle_traveled_s < kepsilon &&
        obstacle->reference_line_st_boundary().min_t() < kIgnoreMaxSTMinT &&
        obstacle->reference_line_st_boundary().min_s() > kIgnoreMinSTMinS) {
      continue;
    }
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->add_wait_for_obstacle_id(obstacle->Id());

    return false;
  }
  return true;
}

}  // namespace yield_sign
}  // namespace scenario
}  // namespace planning
}  // namespace century
