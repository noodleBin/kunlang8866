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

#include "modules/planning/scenarios/scenario_manager.h"

#include <limits>
#include <string>
#include <vector>

#include "modules/map/proto/map_lane.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/dead_end/deadend_turnaround/deadend_turnaround_scenario.h"
#include "modules/planning/scenarios/emergency/emergency_pull_over/emergency_pull_over_scenario.h"
#include "modules/planning/scenarios/emergency/emergency_stop/emergency_stop_scenario.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/learning_model/learning_model_sample_scenario.h"
#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/park_and_go/park_and_go_scenario.h"
#include "modules/planning/scenarios/rescue/rescue_scenario.h"
#include "modules/planning/scenarios/rescue_teb/rescue_teb_scenario.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/scenarios/traffic_light/protected/traffic_light_protected_scenario.h"
#include "modules/planning/scenarios/traffic_light/protected_teb/traffic_light_protected_teb_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/scenarios/uturn_teb/uturn_teb_scenario.h"
#include "modules/planning/scenarios/yield_sign/yield_sign_scenario.h"

namespace century {
namespace planning {
namespace scenario {
namespace {
constexpr double kStoplinePoseLatOffset = -0.3;
constexpr double kDistanceNeartTrafficlight = 35.0;
constexpr double kDistanceToSignalOverlap = 20.0;
constexpr double KEsilon = 1e-5;
constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
}  // namespace
using century::hdmap::HDMapUtil;
using century::hdmap::PathOverlap;
using century::common::math::Vec2d;
using century::common::math::Box2d;
using century::common::math::Polygon2d;

ScenarioManager::ScenarioManager(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

bool ScenarioManager::Init(const PlanningConfig& planning_config) {
  planning_config_.CopyFrom(planning_config);
  RegisterScenarios();
  default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;
  current_scenario_ = CreateScenario(default_scenario_type_);
  return true;
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioConfig::ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;

  switch (scenario_type) {
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
      ptr.reset(
          new scenario::bare_intersection::BareIntersectionUnprotectedScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::EMERGENCY_PULL_OVER:
      ptr.reset(new emergency_pull_over::EmergencyPullOverScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::EMERGENCY_STOP:
      ptr.reset(new emergency_stop::EmergencyStopScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::LANE_FOLLOW:
      ptr.reset(new lane_follow::LaneFollowScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::LEARNING_MODEL_SAMPLE:
      ptr.reset(new scenario::LearningModelSampleScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::PARK_AND_GO:
      ptr.reset(new scenario::park_and_go::ParkAndGoScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::PULL_OVER:
      ptr.reset(new scenario::pull_over::PullOverScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      ptr.reset(new scenario::stop_sign::StopSignUnprotectedScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
      if (FLAGS_enable_scenario_traffic_teb_instead) {
        ptr.reset(
            new scenario::traffic_light_teb::TrafficLightProtectedTebScenario(
                config_map_[scenario_type], &scenario_context_, injector_));
      } else {
        ptr.reset(new scenario::traffic_light::TrafficLightProtectedScenario(
            config_map_[scenario_type], &scenario_context_, injector_));
      }
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedLeftTurnScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedRightTurnScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::VALET_PARKING:
      ptr.reset(new scenario::valet_parking::ValetParkingScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::YIELD_SIGN:
      ptr.reset(new scenario::yield_sign::YieldSignScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::DEADEND_TURNAROUND:
      ptr.reset(new scenario::deadend_turnaround::DeadEndTurnAroundScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::RESCUE:
      ptr.reset(new scenario::rescue::RescueScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::RESCUE_TEB:
      ptr.reset(new scenario::rescue::RescueTebScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioConfig::UTURN_TEB:
      ptr.reset(new scenario::uturn::UturnTebScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}

void ScenarioManager::RegisterScenarios() {
  // lane_follow
  if (planning_config_.learning_mode() == PlanningConfig::HYBRID ||
      planning_config_.learning_mode() == PlanningConfig::HYBRID_TEST) {
    // HYBRID or HYBRID_TEST
    ACHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_hybrid_config_file,
                                &config_map_[ScenarioConfig::LANE_FOLLOW]));
  } else {
    ACHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                                &config_map_[ScenarioConfig::LANE_FOLLOW]));
  }

  // bare_intersection
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_bare_intersection_unprotected_config_file,
      &config_map_[ScenarioConfig::BARE_INTERSECTION_UNPROTECTED]));

  // emergency_pull_over
  ACHECK(
      Scenario::LoadConfig(FLAGS_scenario_emergency_pull_over_config_file,
                           &config_map_[ScenarioConfig::EMERGENCY_PULL_OVER]));

  // emergency_stop
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_emergency_stop_config_file,
                              &config_map_[ScenarioConfig::EMERGENCY_STOP]));

  // learning model
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_learning_model_sample_config_file,
      &config_map_[ScenarioConfig::LEARNING_MODEL_SAMPLE]));

  // park_and_go
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_park_and_go_config_file,
                              &config_map_[ScenarioConfig::PARK_AND_GO]));

  // pull_over
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_pull_over_config_file,
                              &config_map_[ScenarioConfig::PULL_OVER]));

  // stop_sign
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_stop_sign_unprotected_config_file,
      &config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]));

  // traffic_light
  if (FLAGS_enable_scenario_traffic_teb_instead) {
    ACHECK(Scenario::LoadConfig(
        FLAGS_scenario_traffic_light_protected_teb_config_file,
        &config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]));
  } else {
    ACHECK(Scenario::LoadConfig(
        FLAGS_scenario_traffic_light_protected_config_file,
        &config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]));
  }

  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_left_turn_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]));
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_right_turn_config_file,
      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]));

  // valet parking
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
                              &config_map_[ScenarioConfig::VALET_PARKING]));

  // yield_sign
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_yield_sign_config_file,
                              &config_map_[ScenarioConfig::YIELD_SIGN]));
  // turn around
  ACHECK(
      Scenario::LoadConfig(FLAGS_scenario_deadend_turnaround_config_file,
                           &config_map_[ScenarioConfig::DEADEND_TURNAROUND]));

  // rescue
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_rescue_config_file,
                              &config_map_[ScenarioConfig::RESCUE]));

  // rescue teb
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_rescue_teb_config_file,
                              &config_map_[ScenarioConfig::RESCUE_TEB]));

  // uturn teb
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_uturn_teb_config_file,
                              &config_map_[ScenarioConfig::UTURN_TEB]));
}

ScenarioConfig::ScenarioType ScenarioManager::SelectPullOverScenario(
    const Frame& frame) {
  // const auto& scenario_config =
  //     config_map_[ScenarioConfig::PULL_OVER].pull_over_config();

  // const auto& routing = frame.local_view().routing;
  // const auto& routing_end =
  // *(routing->routing_request().waypoint().rbegin());

  // common::SLPoint dest_sl;
  // const auto& reference_line_info = frame.reference_line_info().front();
  // const auto& reference_line = reference_line_info.reference_line();
  // reference_line.XYToSL(routing_end.pose(), &dest_sl);
  // const double adc_front_edge_s =
  // reference_line_info.AdcSlBoundary().end_s();

  // const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;

  // bool pull_over_scenario =
  //     (frame.reference_line_info().size() == 1 &&  // NO, while changing lane
  //      adc_distance_to_dest >=
  //          scenario_config.pull_over_min_distance_buffer() &&
  //      adc_distance_to_dest <=
  //          scenario_config.start_pull_over_scenario_distance());

  // // too close to destination + not found pull-over position
  // if (pull_over_scenario) {
  //   const auto& pull_over_status =
  //       injector_->planning_context()->planning_status().pull_over();
  //   if (adc_distance_to_dest < scenario_config.max_distance_stop_search() &&
  //       !pull_over_status.has_position()) {
  //     pull_over_scenario = false;
  //   }
  // }

  // // check around junction
  // if (pull_over_scenario) {
  //   static constexpr double kDistanceToAvoidJunction = 8.0;  // meter
  //   for (const auto& overlap : first_encountered_overlap_map_) {
  //     if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
  //         overlap.first == ReferenceLineInfo::SIGNAL ||
  //         overlap.first == ReferenceLineInfo::STOP_SIGN ||
  //         overlap.first == ReferenceLineInfo::YIELD_SIGN) {
  //       const double distance_to = overlap.second.start_s - dest_sl.s();
  //       const double distance_passed = dest_sl.s() - overlap.second.end_s;
  //       if ((distance_to > 0.0 && distance_to < kDistanceToAvoidJunction) ||
  //           (distance_passed > 0.0 &&
  //            distance_passed < kDistanceToAvoidJunction)) {
  //         pull_over_scenario = false;
  //         break;
  //       }
  //     }
  //   }
  // }

  // // check rightmost driving lane along pull-over path
  // if (pull_over_scenario) {
  //   double check_s = adc_front_edge_s;
  //   static constexpr double kDistanceUnit = 5.0;
  //   while (check_s < dest_sl.s()) {
  //     check_s += kDistanceUnit;

  //     std::vector<hdmap::LaneInfoConstPtr> lanes;
  //     reference_line.GetLaneFromS(check_s, &lanes);
  //     if (lanes.empty()) {
  //       ADEBUG << "check_s[" << check_s << "] can't find a lane";
  //       continue;
  //     }
  //     const hdmap::LaneInfoConstPtr lane = lanes[0];
  //     const std::string lane_id = lane->lane().id().id();
  //     ADEBUG << "check_s[" << check_s << "] lane[" << lane_id << "]";

  //     // check neighbor lanes type: NONE/CITY_DRIVING/BIKING/SIDEWALK/PARKING
  //     bool rightmost_driving_lane = true;
  //     for (const auto& neighbor_lane_id :
  //          lane->lane().right_neighbor_forward_lane_id()) {
  //       const auto hdmap_ptr = HDMapUtil::BaseMapPtr();
  //       CHECK_NOTNULL(hdmap_ptr);
  //       const auto neighbor_lane = hdmap_ptr->GetLaneById(neighbor_lane_id);
  //       if (neighbor_lane == nullptr) {
  //         ADEBUG << "Failed to find neighbor lane[" << neighbor_lane_id.id()
  //                << "]";
  //         continue;
  //       }
  //       const auto& lane_type = neighbor_lane->lane().type();
  //       if (lane_type == hdmap::Lane::CITY_DRIVING) {
  //         ADEBUG << "lane[" << lane_id << "]'s right neighbor forward lane["
  //                << neighbor_lane_id.id() << "] type["
  //                << Lane_LaneType_Name(lane_type) << "] can't pull over";
  //         rightmost_driving_lane = false;
  //         break;
  //       }
  //     }
  //     if (!rightmost_driving_lane) {
  //       pull_over_scenario = false;
  //       break;
  //     }
  //   }
  // }

  // switch (current_scenario_->scenario_type()) {
  //   case ScenarioConfig::LANE_FOLLOW:
  //     if (pull_over_scenario) {
  //       return ScenarioConfig::PULL_OVER;
  //     }
  //     break;
  //   case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
  //   case ScenarioConfig::EMERGENCY_PULL_OVER:
  //   case ScenarioConfig::PARK_AND_GO:
  //   case ScenarioConfig::PULL_OVER:
  //   case ScenarioConfig::STOP_SIGN_PROTECTED:
  //   case ScenarioConfig::STOP_SIGN_UNPROTECTED:
  //   case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
  //   case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
  //   case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
  //   case ScenarioConfig::VALET_PARKING:
  //   case ScenarioConfig::DEADEND_TURNAROUND:
  //   case ScenarioConfig::YIELD_SIGN:
  //     if (current_scenario_->GetStatus() !=
  //         Scenario::ScenarioStatus::STATUS_DONE) {
  //       return current_scenario_->scenario_type();
  //     }
  //     break;
  //   default:
  //     break;
  // }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectPadMsgScenario(
    const Frame& frame) {
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();
  switch (pad_msg_driving_action) {
    case DrivingAction::PULL_OVER:
      if (FLAGS_enable_scenario_emergency_pull_over) {
        return ScenarioConfig::EMERGENCY_PULL_OVER;
      }
      break;
    case DrivingAction::STOP:
      if (FLAGS_enable_scenario_emergency_stop) {
        return ScenarioConfig::EMERGENCY_STOP;
      }
      break;
    case DrivingAction::RESUME_CRUISE:
      if (current_scenario_->scenario_type() ==
              ScenarioConfig::EMERGENCY_PULL_OVER ||
          current_scenario_->scenario_type() ==
              ScenarioConfig::EMERGENCY_STOP) {
        return ScenarioConfig::PARK_AND_GO;
      }
      break;
    default:
      break;
  }
  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectInterceptionScenario(
    const Frame& frame) {
  ScenarioConfig::ScenarioType scenario_type = default_scenario_type_;

  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  hdmap::PathOverlap* pnc_junction_overlap = nullptr;
  ReferenceLineInfo::OverlapType overlap_type;

  CheckOverLap(frame, &traffic_sign_overlap, &pnc_junction_overlap,
               &overlap_type);

  // pick a closer one between consecutive bare_intersection and traffic_sign
  if (traffic_sign_overlap && pnc_junction_overlap) {
    static constexpr double kJunctionDelta = 10.0;
    double s_diff = std::fabs(traffic_sign_overlap->start_s -
                              pnc_junction_overlap->start_s);
    if (s_diff >= kJunctionDelta) {
      if (pnc_junction_overlap->start_s > traffic_sign_overlap->start_s) {
        pnc_junction_overlap = nullptr;
      } else {
        traffic_sign_overlap = nullptr;
      }
    }
  }

  if (traffic_sign_overlap) {
    switch (overlap_type) {
      case ReferenceLineInfo::STOP_SIGN:
        if (FLAGS_enable_scenario_stop_sign) {
          hdmap::StopSignInfoConstPtr stop_sign_ptr =
              HDMapUtil::BaseMap().GetStopSignById(
                  hdmap::MakeMapId(traffic_sign_overlap->object_id));
          if (nullptr != stop_sign_ptr &&
              hdmap::StopSign::STOP ==
                  stop_sign_ptr->stop_sign().motion_type()) {
            scenario_type =
                SelectStopSignScenario(frame, *traffic_sign_overlap);
          }
        }
        break;
      case ReferenceLineInfo::SIGNAL:
        if (FLAGS_enable_scenario_traffic_light) {
          scenario_type =
              SelectTrafficLightScenario(frame, *traffic_sign_overlap);
        }
        break;
      case ReferenceLineInfo::YIELD_SIGN:
        if (FLAGS_enable_scenario_yield_sign) {
          scenario_type = SelectYieldSignScenario(frame, *traffic_sign_overlap);
        }
        break;
      default:
        break;
    }
  } else if (pnc_junction_overlap) {
    // bare intersection
    if (FLAGS_enable_scenario_bare_intersection) {
      scenario_type =
          SelectBareIntersectionScenario(frame, *pnc_junction_overlap);
    }
  }

  return scenario_type;
}

void ScenarioManager::CheckOverLap(
    const Frame& frame, hdmap::PathOverlap** const traffic_sign_overlap,
    hdmap::PathOverlap** const pnc_junction_overlap,
    ReferenceLineInfo::OverlapType* const overlap_type) {
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  // note: first_encountered_overlaps already sorted
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      *overlap_type = overlap.first;
      *traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    } else if (overlap.first == ReferenceLineInfo::PNC_JUNCTION) {
      *pnc_junction_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
    }
  }
}

ScenarioConfig::ScenarioType ScenarioManager::SelectStopSignScenario(
    const Frame& frame, const hdmap::PathOverlap& stop_sign_overlap) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]
          .stop_sign_unprotected_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_stop_sign =
      stop_sign_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign[" << stop_sign_overlap.object_id
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap.start_s << "]";

  const bool stop_sign_scenario =
      (adc_distance_to_stop_sign > 0.0 &&
       adc_distance_to_stop_sign <=
           scenario_config.start_stop_sign_scenario_distance());
  const bool stop_sign_all_way = false;  // TODO(all)

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (stop_sign_scenario) {
        return stop_sign_all_way ? ScenarioConfig::STOP_SIGN_PROTECTED
                                 : ScenarioConfig::STOP_SIGN_UNPROTECTED;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectTrafficLightScenario(
    const Frame& frame, const hdmap::PathOverlap& traffic_light_overlap) {
  // some scenario may need start sooner than the others
  const double start_check_distance = std::max(
      {config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]
           .traffic_light_protected_config()
           .start_traffic_light_scenario_distance(),
       config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]
           .traffic_light_unprotected_left_turn_config()
           .start_traffic_light_scenario_distance(),
       config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]
           .traffic_light_unprotected_right_turn_config()
           .start_traffic_light_scenario_distance()});

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  // find all the traffic light belong to
  // the same group as first encountered traffic light
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  for (const auto& overlap : traffic_light_overlaps) {
    const double dist = overlap.start_s - traffic_light_overlap.start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      next_traffic_lights.push_back(overlap);
    }
  }

  bool traffic_light_scenario = false;
  // note: need iterate all lights to check no RED/YELLOW/UNKNOWN
  for (const auto& traffic_light_overlap : next_traffic_lights) {
    const double adc_distance_to_traffic_light =
        traffic_light_overlap.start_s - adc_front_edge_s;

    // enter traffic-light scenarios: based on distance only
    if (adc_distance_to_traffic_light <= 0.0 ||
        adc_distance_to_traffic_light > start_check_distance) {
      continue;
    }

    traffic_light_scenario = true;

    // Update stopline_sl into planning-context.
    common::SLPoint stopline_sl;
    const double front_to_axis =
        common::VehicleConfigHelper::GetConfig().vehicle_param().length() -
        common::VehicleConfigHelper::GetConfig()
            .vehicle_param()
            .back_edge_to_center();
    stopline_sl.set_s(traffic_light_overlap.start_s - front_to_axis);
    stopline_sl.set_l(kStoplinePoseLatOffset);
    common::math::Vec2d stopline_xy;
    reference_line_info.reference_line().SLToXY(stopline_sl, &stopline_xy);
    auto* adjust_pose = injector_->planning_context()
                            ->mutable_planning_status()
                            ->mutable_traffic_light()
                            ->mutable_adc_adjust_end_pose();
    adjust_pose->set_x(stopline_xy.x());
    adjust_pose->set_y(stopline_xy.y());
    perception::TrafficLight traffic_light;
    frame.GetSignal(traffic_light_overlap.object_id, &traffic_light);
    const auto& signal_color = traffic_light.color();

    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "]";
  }

  bool traffic_light_protected_scenario = false;
  if (traffic_light_scenario) {
    // const auto& turn_type =
    //     reference_line_info.GetPathTurnType(traffic_light_overlap.start_s);
    // const bool right_turn = (turn_type == hdmap::Lane::RIGHT_TURN);
    // const bool left_turn = (turn_type == hdmap::Lane::LEFT_TURN);
    const double adc_distance_to_traffic_light =
        traffic_light_overlap.start_s - adc_front_edge_s;

    // turn type in map may error(such as binyuxincun back in longchi road)
    // for use right turn light ,then enter into
    // TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN; temp only use
    // traffic_light_protected_scenario

    const auto& scenario_config =
        config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]
            .traffic_light_protected_config();
    if (adc_distance_to_traffic_light <
        scenario_config.start_traffic_light_scenario_distance()) {
      traffic_light_protected_scenario = true;
    }
  }

  return GetCurrentScenario(traffic_light_protected_scenario);
}

ScenarioConfig::ScenarioType ScenarioManager::GetCurrentScenario(
    const bool traffic_light_protected_scenario) {
  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (traffic_light_protected_scenario) {
        SetTebCommonStatus();
        return ScenarioConfig::TRAFFIC_LIGHT_PROTECTED;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;

    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectYieldSignScenario(
    const Frame& frame, const hdmap::PathOverlap& yield_sign_overlap) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::YIELD_SIGN].yield_sign_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_yield_sign =
      yield_sign_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_yield_sign[" << adc_distance_to_yield_sign
         << "] yield_sign[" << yield_sign_overlap.object_id
         << "] yield_sign_overlap_start_s[" << yield_sign_overlap.start_s
         << "]";

  const bool yield_sign_scenario =
      (adc_distance_to_yield_sign > 0.0 &&
       adc_distance_to_yield_sign <=
           scenario_config.start_yield_sign_scenario_distance());

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (yield_sign_scenario) {
        return ScenarioConfig::YIELD_SIGN;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectBareIntersectionScenario(
    const Frame& frame, const hdmap::PathOverlap& pnc_junction_overlap) {
  const auto& reference_line_info = frame.reference_line_info().front();
  if (reference_line_info.GetIntersectionRightofWayStatus(
          pnc_junction_overlap)) {
    return default_scenario_type_;
  }

  const auto& scenario_config =
      config_map_[ScenarioConfig::BARE_INTERSECTION_UNPROTECTED]
          .bare_intersection_unprotected_config();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_pnc_junction =
      pnc_junction_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_pnc_junction[" << adc_distance_to_pnc_junction
         << "] pnc_junction[" << pnc_junction_overlap.object_id
         << "] pnc_junction_overlap_start_s[" << pnc_junction_overlap.start_s
         << "]";

  const bool bare_junction_scenario =
      (adc_distance_to_pnc_junction > 0.0 &&
       adc_distance_to_pnc_junction <=
           scenario_config.start_bare_intersection_scenario_distance());

  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::PULL_OVER:
      if (bare_junction_scenario) {
        return ScenarioConfig::BARE_INTERSECTION_UNPROTECTED;
      }
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::YIELD_SIGN:
    case ScenarioConfig::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectValetParkingScenario(
    const Frame& frame) {
  const auto& scenario_config =
      config_map_[ScenarioConfig::VALET_PARKING].valet_parking_config();

  // TODO(All) trigger valet parking by route message definition as of now
  double parking_spot_range_to_start =
      scenario_config.parking_spot_range_to_start();
  if (scenario::valet_parking::ValetParkingScenario::IsTransferable(
          frame, parking_spot_range_to_start)) {
    return ScenarioConfig::VALET_PARKING;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectDeadEndScenario(
    const Frame& frame) {
  // lwt:new uturn
  if (injector_->is_need_to_uturn_ && FLAGS_enable_use_deadend_mode) {
    SetTebCommonStatus();
    return ScenarioConfig::UTURN_TEB;
  }
  size_t waypoint_num =
      frame.local_view().routing->routing_request().waypoint().size();
  const auto& routing_type = frame.local_view()
                                 .routing->routing_request()
                                 .dead_end_info()
                                 .dead_end_routing_type();
  if (routing_type == routing::ROUTING_IN) {
    dead_end_point_ = frame.local_view()
                          .routing->routing_request()
                          .waypoint()
                          .at(waypoint_num - 1)
                          .pose();
  } else if (routing_type == routing::ROUTING_OUT) {
    dead_end_point_ =
        frame.local_view().routing->routing_request().waypoint().at(0).pose();
  }
  const auto& scenario_config = config_map_[ScenarioConfig::DEADEND_TURNAROUND]
                                    .deadend_turnaround_config();
  double dead_end_start_range = scenario_config.dead_end_start_range();
  if (scenario::deadend_turnaround::DeadEndTurnAroundScenario::IsTransferable(
          frame, dead_end_point_, dead_end_start_range) &&
      routing_type == routing::ROUTING_IN) {
    return ScenarioConfig::DEADEND_TURNAROUND;
  }

  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectRescueScenario() {
  ADEBUG << __func__ << ", need_to_rescue: " << injector_->need_to_rescue();
  if (injector_->need_to_rescue()) {
    auto* rescue_status = injector_->planning_context()
                              ->mutable_planning_status()
                              ->mutable_rescue();
    if (!rescue_init_ || std::isnan(rescue_status->adc_init_position().x())) {
      rescue_status->Clear();
      century::planning::RescueStatus pb_rescue_status;
      if (!century::cyber::common::GetProtoFromFile(
              FLAGS_rescue_status_config_file, &pb_rescue_status)) {
        AERROR << "----Failed to load txt rescue status from "
               << FLAGS_rescue_status_config_file;
      } else {
        ADEBUG << "----Loaded txt rescue status from "
               << FLAGS_rescue_status_config_file;
      }
      SetTebCommonStatus();
      rescue_status->CopyFrom(pb_rescue_status);
      ADEBUG << "injector_->vehicle_state()->x()"
             << injector_->vehicle_state()->x();
      rescue_status->mutable_adc_init_position()->set_x(
          injector_->vehicle_state()->x());
      rescue_status->mutable_adc_init_position()->set_y(
          injector_->vehicle_state()->y());
      rescue_status->mutable_adc_init_position()->set_z(0.0);
      rescue_status->set_adc_init_heading(
          injector_->vehicle_state()->heading());
      rescue_init_ = true;
    }
    return FLAGS_enable_use_teb ? ScenarioConfig::RESCUE_TEB
                                : ScenarioConfig::RESCUE;
  }
  return default_scenario_type_;
}

ScenarioConfig::ScenarioType ScenarioManager::SelectParkAndGoScenario(
    const Frame& frame) {
  bool park_and_go = false;
  //const auto& scenario_config =
  //    config_map_[ScenarioConfig::PARK_AND_GO].park_and_go_config();
  const auto vehicle_state_provider = injector_->vehicle_state();
  common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
  // TODO(SHU) might consider gear == GEAR_PARKING
  double adc_speed = vehicle_state_provider->linear_velocity();
  //double s = 0.0;
  //double l = 0.0;
  const double max_abs_speed_when_stopped =
      common::VehicleConfigHelper::Instance()
          ->GetConfig()
          .vehicle_param()
          .max_abs_speed_when_stopped();

  //hdmap::LaneInfoConstPtr lane = nullptr;

  // check ego vehicle distance to destination
  //const auto& routing = frame.local_view().routing;
  //const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  //common::SLPoint dest_sl;
  //const auto& reference_line_info = frame.reference_line_info().front();
  //const auto& reference_line = reference_line_info.reference_line();
  //reference_line.XYToSL(routing_end.pose(), &dest_sl);
  //const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  auto vehicle_params = century::common::VehicleConfigHelper::GetConfig()
                            .vehicle_param();
  const Vec2d adc_position(adc_point.x(), adc_point.y());
  const double adc_length = vehicle_params.length();
  const double adc_width = vehicle_params.width();
  Box2d adc_box(adc_position, vehicle_state.heading(), adc_length, adc_width);
  double shift_distance = adc_length * 0.5 - vehicle_params.back_edge_to_center();
  const Vec2d shift_vec(shift_distance * std::cos(vehicle_state.heading()),
                        shift_distance * std::sin(vehicle_state.heading()));
  // current vehicle box
  adc_box.Shift(shift_vec);

  std::vector<hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
  HDMapUtil::BaseMap().GetParkingSpaces(adc_point, 2.0, &parking_spaces);
  for (auto parking_space : parking_spaces) {
    // parking space
    Polygon2d parking_space_polygon = parking_space->polygon();
    // vehicle is in parking space and stopped
    if (parking_space_polygon.Contains(common::math::Polygon2d(adc_box)) &&
        std::fabs(adc_speed) < max_abs_speed_when_stopped) {
      AINFO << "vehicle is in parking space " << parking_space->id().id();
      park_and_go = true;
      break;
    }
  }

  //const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;
  // if vehicle is static, far enough to destination and (off-lane or not on
  // city_driving lane)
  //if (std::fabs(adc_speed) < max_abs_speed_when_stopped &&
  //    adc_distance_to_dest > scenario_config.min_dist_to_dest() &&
  //    (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
  //         adc_point, 2.0, vehicle_state.heading(), M_PI / 3.0, &lane, &s,
  //         &l) != 0 ||
  //     lane->lane().type() != hdmap::Lane::CITY_DRIVING)) {
  //  park_and_go = true;
  //}

  if (park_and_go) {
    return ScenarioConfig::PARK_AND_GO;
  }

  return default_scenario_type_;
}

void ScenarioManager::Observe(const Frame& frame) {
  // init first_encountered_overlap_map_
  first_encountered_overlap_map_.clear();
  // each frame needs to be initialized.
  injector_->near_traffic_line_ = false;
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
        overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      first_encountered_overlap_map_[overlap.first] = overlap.second;
    }
    // Determine if it is approaching the traffic light intersection (within 35m
    // ahead); It will only be added within 100m in front of the ADC; ADC
    // exceeds overlay. second. end_ S will not be added either;
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      ADEBUG << "overlap.second.start_s=" << overlap.second.start_s
             << "  overlap.second.end_s=" << overlap.second.end_s;
      double near_traffic_line_dist = std::numeric_limits<double>::max();
      near_traffic_line_dist = std::fabs(overlap.second.start_s - adc_end_s);
      if (near_traffic_line_dist < kDistanceNeartTrafficlight) {
        injector_->near_traffic_line_ = true;
        // approaching first signal.
        const auto& reference_line = reference_line_info.reference_line();
        if (std::fabs(injector_->last_signal_overlap_end_xy_.x()) < KEsilon &&
            std::fabs(injector_->last_signal_overlap_end_xy_.y()) < KEsilon) {
          common::SLPoint signal_overlap_end_sl;
          signal_overlap_end_sl.set_s(overlap.second.end_s);
          signal_overlap_end_sl.set_l(0.0);
          common::math::Vec2d signal_overlap_end_xy;
          if (reference_line.SLToXY(signal_overlap_end_sl,
                                    &signal_overlap_end_xy)) {
            injector_->last_signal_overlap_end_xy_.set_x(
                signal_overlap_end_xy.x());
            injector_->last_signal_overlap_end_xy_.set_y(
                signal_overlap_end_xy.y());
            ADEBUG << "first update last signal overlap";
          }
          continue;
        }
        // last signal overlap
        common::SLPoint last_signal_overlap_end_sl;
        if (reference_line.XYToSL(injector_->last_signal_overlap_end_xy_,
                                  &last_signal_overlap_end_sl)) {
          ADEBUG << "last_signal_overlap_end_sl = "
                 << last_signal_overlap_end_sl.s() << "           "
                 << last_signal_overlap_end_sl.l();
          ADEBUG << "injector_->last_signal_overlap_end_xy_XY= "
                 << injector_->last_signal_overlap_end_xy_.x() << "           "
                 << injector_->last_signal_overlap_end_xy_.y();
          // More than 20 after adc and approaching a new signal overlap.
          if (reference_line_info.AdcSlBoundary().start_s() -
                  (last_signal_overlap_end_sl.s()) >
              kDistanceToSignalOverlap) {
            common::SLPoint signal_overlap_end_sl;
            signal_overlap_end_sl.set_s(overlap.second.end_s);
            signal_overlap_end_sl.set_l(0.0);
            common::math::Vec2d signal_overlap_end_xy;
            if (reference_line.SLToXY(signal_overlap_end_sl,
                                      &signal_overlap_end_xy)) {
              injector_->last_signal_overlap_end_xy_.set_x(
                  signal_overlap_end_xy.x());
              injector_->last_signal_overlap_end_xy_.set_y(
                  signal_overlap_end_xy.y());
              ADEBUG << "update last signal overlap.";
            }
          }
        }
      }
    }
  }
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  ACHECK(!frame.reference_line_info().empty());

  Observe(frame);

  ScenarioDispatch(frame);
}

void ScenarioManager::ScenarioDispatch(const Frame& frame) {
  ACHECK(!frame.reference_line_info().empty());
  ScenarioConfig::ScenarioType scenario_type;

  int history_points_len = 0;
  if (injector_->learning_based_data() &&
      injector_->learning_based_data()->GetLatestLearningDataFrame()) {
    history_points_len = injector_->learning_based_data()
                             ->GetLatestLearningDataFrame()
                             ->adc_trajectory_point_size();
  }
  if ((planning_config_.learning_mode() == PlanningConfig::E2E ||
       planning_config_.learning_mode() == PlanningConfig::E2E_TEST) &&
      history_points_len >= FLAGS_min_past_history_points_len) {
    scenario_type = ScenarioDispatchLearning();
  } else {
    scenario_type = ScenarioDispatchNonLearning(frame);
  }

  AINFO << "select scenario: "
        << ScenarioConfig::ScenarioType_Name(scenario_type);

  // update PlanningContext
  UpdatePlanningContext(frame, scenario_type);

  if (current_scenario_->scenario_type() != scenario_type) {
    AINFO << "switch scenario from " << current_scenario_->Name() << " to "
          << ScenarioConfig::ScenarioType_Name(scenario_type);
    current_scenario_ = CreateScenario(scenario_type);
  }
}

ScenarioConfig::ScenarioType ScenarioManager::ScenarioDispatchLearning() {
  ////////////////////////////////////////
  // learning model scenario
  ScenarioConfig::ScenarioType scenario_type =
      ScenarioConfig::LEARNING_MODEL_SAMPLE;
  return scenario_type;
}

bool ScenarioManager::JudgeReachTargetPoint(
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

ScenarioConfig::ScenarioType ScenarioManager::ScenarioDispatchNonLearning(
    const Frame& frame) {
  ////////////////////////////////////////
  // default: LANE_FOLLOW
  ScenarioConfig::ScenarioType scenario_type = default_scenario_type_;
  ////////////////////////////////////////
  // Pad Msg scenario
  scenario_type = SelectPadMsgScenario(frame);
  ADEBUG << "current_scenario_ " << current_scenario_->scenario_type();
  const auto vehicle_state_provider = injector_->vehicle_state();
  common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  if (scenario_type == default_scenario_type_) {
    // check current_scenario (not switchable)
    CheckCurrentScenario(frame, &scenario_type);
  }
  ////////////////////////////////////////
  // ParkAndGo / starting scenario
  if (scenario_type == default_scenario_type_) {
    if (FLAGS_enable_scenario_park_and_go && !reach_target_pose_) {
      scenario_type = SelectParkAndGoScenario(frame);
    }
  }

  ////////////////////////////////////////
  // intersection scenarios
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectInterceptionScenario(frame);
  }

  ////////////////////////////////////////
  // pull-over scenario
  // now  pull-over is entered in rescue scenario
  // if (scenario_type == default_scenario_type_) {
  //   if (FLAGS_enable_scenario_pull_over) {
  //     scenario_type = SelectPullOverScenario(frame);
  //   }
  // }

  ////////////////////////////////////////
  // VALET_PARKING scenario
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectValetParkingScenario(frame);
  }
  ////////////////////////////////////////
  // dead end
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectDeadEndScenario(frame);
  }
  ////////////////////////////////////////

  ////////////////////////////////////////
  // rescue
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectRescueScenario();
  }
  if (scenario_type == default_scenario_type_) {
    // enter rescue failed
    ADEBUG << "enter rescue failed ";
    rescue_init_ = false;
  }
  ////////////////////////////////////////
  return scenario_type;
}

void ScenarioManager::CheckCurrentScenario(
    const Frame& frame, ScenarioConfig::ScenarioType* const scenario_type) {
  const common::PointENU& target_point = frame.local_view()
                                             .routing->routing_request()
                                             .dead_end_info()
                                             .target_point();
  const common::VehicleState& car_position = frame.vehicle_state();
  switch (current_scenario_->scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::PULL_OVER:
      break;
    case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioConfig::EMERGENCY_PULL_OVER:
    case ScenarioConfig::PARK_AND_GO:
    case ScenarioConfig::STOP_SIGN_PROTECTED:
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioConfig::VALET_PARKING:
    case ScenarioConfig::DEADEND_TURNAROUND:
      // transfer dead_end to lane follow, should enhance transfer logic
      if (JudgeReachTargetPoint(car_position, target_point)) {
        *scenario_type = ScenarioConfig::LANE_FOLLOW;
        reach_target_pose_ = true;
        auto* destination = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_destination();
        destination->set_has_reached_destination(true);
        AERROR << "reach_target_pose_ set_has_reached_destination";
        break;
      } else {
        reach_target_pose_ = false;
        auto* destination = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_destination();
        destination->set_has_reached_destination(false);
      }
    case ScenarioConfig::YIELD_SIGN:
      // must continue until finish
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        *scenario_type = current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }
}

bool ScenarioManager::IsBareIntersectionScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::BARE_INTERSECTION_UNPROTECTED);
}

bool ScenarioManager::IsStopSignScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::STOP_SIGN_PROTECTED ||
          scenario_type == ScenarioConfig::STOP_SIGN_UNPROTECTED);
}

bool ScenarioManager::IsTrafficLightScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_PROTECTED ||
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
      scenario_type == ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN);
}

bool ScenarioManager::IsYieldSignScenario(
    const ScenarioConfig::ScenarioType& scenario_type) {
  return (scenario_type == ScenarioConfig::YIELD_SIGN);
}

void ScenarioManager::UpdatePlanningContext(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  // BareIntersection scenario
  UpdatePlanningContextBareIntersectionScenario(frame, scenario_type);

  // EmergencyStop scenario
  UpdatePlanningContextEmergencyStopcenario(frame, scenario_type);

  // PullOver & EmergencyPullOver scenarios
  UpdatePlanningContextPullOverScenario(frame, scenario_type);

  // StopSign scenario
  UpdatePlanningContextStopSignScenario(frame, scenario_type);

  // TrafficLight scenario
  UpdatePlanningContextTrafficLightScenario(frame, scenario_type);

  // YieldSign scenario
  UpdatePlanningContextYieldSignScenario(frame, scenario_type);
}

// update: bare_intersection status in PlanningContext
void ScenarioManager::UpdatePlanningContextBareIntersectionScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  auto* bare_intersection = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_bare_intersection();

  if (!IsBareIntersectionScenario(scenario_type)) {
    bare_intersection->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered pnc_junction
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::PNC_JUNCTION);
  if (map_itr != first_encountered_overlap_map_.end()) {
    bare_intersection->set_current_pnc_junction_overlap_id(
        map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered pnc_junction["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: emergency_stop status in PlanningContext
void ScenarioManager::UpdatePlanningContextEmergencyStopcenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  auto* emergency_stop = injector_->planning_context()
                             ->mutable_planning_status()
                             ->mutable_emergency_stop();
  if (scenario_type != ScenarioConfig::EMERGENCY_STOP) {
    emergency_stop->Clear();
  }
}

// update: stop_sign status in PlanningContext
void ScenarioManager::UpdatePlanningContextStopSignScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsStopSignScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered stop_sign
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::STOP_SIGN);
  if (map_itr != first_encountered_overlap_map_.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->set_current_stop_sign_overlap_id(map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered stop sign["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: traffic_light(s) status in PlanningContext
void ScenarioManager::UpdatePlanningContextTrafficLightScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsTrafficLightScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // get first_encountered traffic_light
  std::string current_traffic_light_overlap_id;
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::SIGNAL);
  if (map_itr != first_encountered_overlap_map_.end()) {
    current_traffic_light_overlap_id = map_itr->second.object_id;
  }

  if (current_traffic_light_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  // find all the traffic light at/within the same location/group
  const auto& reference_line_info = frame.reference_line_info().front();
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_itr = std::find_if(
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
  if (traffic_light_overlap_itr == traffic_light_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_traffic_light_overlap_start_s =
      traffic_light_overlap_itr->start_s;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_current_traffic_light_overlap_id(
              traffic_light_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered traffic_light["
             << traffic_light_overlap.object_id << "] start_s["
             << traffic_light_overlap.start_s << "]";
    }
  }
}

// update: yield_sign status in PlanningContext
void ScenarioManager::UpdatePlanningContextYieldSignScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  if (!IsYieldSignScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // get first_encountered yield_sign
  std::string current_yield_sign_overlap_id;
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::YIELD_SIGN);
  if (map_itr != first_encountered_overlap_map_.end()) {
    current_yield_sign_overlap_id = map_itr->second.object_id;
  }

  if (current_yield_sign_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  // find all the yield_sign at/within the same location/group
  const auto& reference_line_info = frame.reference_line_info().front();
  const std::vector<PathOverlap>& yield_sign_overlaps =
      reference_line_info.reference_line().map_path().yield_sign_overlaps();
  auto yield_sign_overlap_itr = std::find_if(
      yield_sign_overlaps.begin(), yield_sign_overlaps.end(),
      [&current_yield_sign_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_yield_sign_overlap_id;
      });
  if (yield_sign_overlap_itr == yield_sign_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_yield_sign_overlap_start_s =
      yield_sign_overlap_itr->start_s;
  for (const auto& yield_sign_overlap : yield_sign_overlaps) {
    const double dist =
        yield_sign_overlap.start_s - current_yield_sign_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_yield_sign()
          ->add_current_yield_sign_overlap_id(yield_sign_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered yield_sign["
             << yield_sign_overlap.object_id << "] start_s["
             << yield_sign_overlap.start_s << "]";
    }
  }
}

// update: pull_over status in PlanningContext
void ScenarioManager::UpdatePlanningContextPullOverScenario(
    const Frame& frame, const ScenarioConfig::ScenarioType& scenario_type) {
  auto* pull_over = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_pull_over();
  if (scenario_type == ScenarioConfig::PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::PULL_OVER);
    pull_over->set_plan_pull_over_path(true);
    return;
  } else if (scenario_type == ScenarioConfig::EMERGENCY_PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::EMERGENCY_PULL_OVER);
    return;
  }

  pull_over->set_plan_pull_over_path(false);

  // check pull_over_status left behind
  // keep it if close to destination, to keep stop fence
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    const auto& routing = frame.local_view().routing;
    if (routing->routing_request().waypoint_size() >= 2) {
      // keep pull-over stop fence if destination not changed
      const auto& reference_line_info = frame.reference_line_info().front();
      const auto& reference_line = reference_line_info.reference_line();

      common::SLPoint dest_sl;
      const auto& routing_end =
          *(routing->routing_request().waypoint().rbegin());
      reference_line.XYToSL(routing_end.pose(), &dest_sl);

      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

      static constexpr double kDestMaxDelta = 30.0;  // meter
      if (std::fabs(dest_sl.s() - pull_over_sl.s()) > kDestMaxDelta) {
        injector_->planning_context()
            ->mutable_planning_status()
            ->clear_pull_over();
      }
    }
  }
}

void ScenarioManager::SetTebCommonStatus() {
  auto* teb_common = injector_->planning_context()
                         ->mutable_planning_status()
                         ->mutable_teb_common();
  if (std::isnan(teb_common->adc_init_position().x())) {
    teb_common->Clear();
    AINFO << "injector_->vehicle_state()->x()"
          << injector_->vehicle_state()->x();
    teb_common->mutable_adc_init_position()->set_x(
        injector_->vehicle_state()->x());
    teb_common->mutable_adc_init_position()->set_y(
        injector_->vehicle_state()->y());
    teb_common->mutable_adc_init_position()->set_z(0.0);
    teb_common->set_adc_init_heading(injector_->vehicle_state()->heading());
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace century
