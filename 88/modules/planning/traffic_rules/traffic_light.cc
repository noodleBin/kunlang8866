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

#include "modules/planning/traffic_rules/traffic_light.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::common::Status;
using century::hdmap::HDMapUtil;
using century::hdmap::PathOverlap;

namespace {
constexpr double kEStopSpeed = 0.15;
constexpr double kDistanceBuffer = 5.0;
constexpr double kDistanceToStopLineBuffer = 20.0;
constexpr double kForwardBuffer = 3.0;
constexpr double kTimeBuffer = 0.4;  // s_buffer time(s)
}  // namespace

TrafficLight::TrafficLight(const TrafficRuleConfig& config,
                           const std::shared_ptr<DependencyInjector>& injector)
    : TrafficRule(config, injector) {}

Status TrafficLight::ApplyRule(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);

  return Status::OK();
}

void TrafficLight::MakeDecisions(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.traffic_light().enabled()) {
    return;
  }

  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // debug info
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();
  signal_light_debug->set_adc_front_s(adc_front_edge_s);
  signal_light_debug->set_adc_speed(
      injector_->vehicle_state()->linear_velocity());

  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info->reference_line().map_path().signal_overlaps();
  std::unordered_map<std::string, TrafficLightMatching> traffic_light_matchings;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    // check is car near stop line
    if (std::fabs(traffic_light_overlap.start_s - adc_front_edge_s) <
        kDistanceToStopLineBuffer) {
      // set car near stop line
      reference_line_info->SetNearTrafficLightStopLine(true);
    }

    if (CheckRoundRouting(reference_line_info, traffic_light_overlap,
                          adc_front_edge_s)) {
      continue;
    }

    if (!CheckTrafficLightMatching(reference_line_info, traffic_light_overlap,
                                   &traffic_light_matchings)) {
      continue;
    }

    reference_line_info->SetNoGreenInJunction(
        adc_front_edge_s > traffic_light_overlap.end_s &&
        frame->NoGreenTrafficLight(traffic_light_overlap.object_id));

    if (traffic_light_overlap.end_s + kForwardBuffer <= adc_back_edge_s) {
      continue;
    }

    perception::TrafficLight traffic_light;
    frame->GetSignal(traffic_light_overlap.object_id, &traffic_light);

    double adc_speed = injector_->vehicle_state()->linear_velocity();
    // stop_distance = v0*v0/2a + s_buffer(0.4*v0)
    double stop_distance =
        (adc_speed * adc_speed) /
            (2 * config_.traffic_light().max_stop_deceleration()) +
        (kTimeBuffer * adc_speed);
    double stop_s = adc_front_edge_s + stop_distance;
    if (stop_s < traffic_light_overlap.start_s) {
      stop_s = traffic_light_overlap.start_s;
    }

    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << stop_s << "] color[" << traffic_light.color()
           << "] stop_deceleration["
           << config_.traffic_light().max_stop_deceleration() << "]";

    // debug info
    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    signal_debug->set_adc_stop_deceleration(
        config_.traffic_light().max_stop_deceleration());
    signal_debug->set_color(traffic_light.color());
    signal_debug->set_light_id(traffic_light_overlap.object_id);
    signal_debug->set_light_stop_s(stop_s);
    signal_debug->set_confidence(traffic_light.confidence());
    frame->stop_line_s_ = stop_s;
    frame->signal_color_ = traffic_light.color();

    if (perception::TrafficLight::GREEN == traffic_light.color()) {
      continue;
    }

    // Red/Yellow/Unknown: check deceleration
    if (stop_s > traffic_light_overlap.start_s + kForwardBuffer) {
      AWARN << "Stop distance too big to achieve.  SKIP red light";
      continue;
    }

    CheckTrafficLightRequest(reference_line_info, traffic_light.color(),
                             stop_s);

    // build stop decision
    ADEBUG << "BuildStopDecision: traffic_light["
           << traffic_light_overlap.object_id << "] start_s[" << stop_s << "]";
    std::string virtual_obstacle_id = TRAFFIC_LIGHT_VO_ID_PREFIX +
                                      traffic_light_overlap.object_id +
                                      std::string("_") + std::to_string(stop_s);
    const std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(
        virtual_obstacle_id, stop_s, config_.traffic_light().stop_distance(),
        StopReasonCode::STOP_REASON_SIGNAL, wait_for_obstacles,
        TrafficRuleConfig::RuleId_Name(config_.rule_id()), frame,
        reference_line_info);
  }

  MakeDecisionsForNoMatching(frame, reference_line_info,
                             traffic_light_matchings, adc_back_edge_s);
}

bool TrafficLight::CheckRoundRouting(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap& traffic_light_overlap,
    const double adc_front_edge_s) {
  // work around incorrect s-projection along round routing
  static constexpr double kSDiscrepanceTolerance = 10.0;
  const auto& reference_line = reference_line_info->reference_line();
  common::SLPoint traffic_light_sl;
  traffic_light_sl.set_s(traffic_light_overlap.start_s);
  traffic_light_sl.set_l(0);
  common::math::Vec2d traffic_light_point;
  reference_line.SLToXY(traffic_light_sl, &traffic_light_point);
  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  const double distance =
      common::util::DistanceXY(traffic_light_point, adc_position);
  const double s_distance = traffic_light_overlap.start_s - adc_front_edge_s;
  ADEBUG << "traffic_light[" << traffic_light_overlap.object_id << "] start_s["
         << traffic_light_overlap.start_s << "] s_distance[" << s_distance
         << "] actual_distance[" << distance << "]";
  if (s_distance >= 0 && fabs(s_distance - distance) > kSDiscrepanceTolerance) {
    ADEBUG << "SKIP traffic_light[" << traffic_light_overlap.object_id
           << "] close in position, but far away along reference line";
    return true;
  }

  return false;
}

bool TrafficLight::CheckTrafficLightMatching(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap& traffic_light_overlap,
    std::unordered_map<std::string, TrafficLightMatching>* const
        traffic_light_matchings) {
  hdmap::SignalInfoConstPtr signal_info_ptr =
      HDMapUtil::BaseMap().GetSignalById(
          hdmap::MakeMapId(traffic_light_overlap.object_id));
  auto signal_light_index =
      reference_line_info->reference_line().map_path().GetLaneIndexFromS(
          (traffic_light_overlap.start_s + traffic_light_overlap.end_s) * 0.5);
  auto& lane_segments =
      reference_line_info->reference_line().map_path().lane_segments();
  if (signal_light_index.id >= static_cast<int>(lane_segments.size())) {
    ADEBUG << "Failed get signal light lane trun type.";
    return false;
  }

  auto signal_light_lane_id =
      lane_segments[signal_light_index.id].lane->lane().id().id();
  ADEBUG << "signal_light_lane_id = " << signal_light_lane_id;
  auto signal_light_lane_trun_type =
      lane_segments[signal_light_index.id].lane->lane().turn();
  ADEBUG << "signal_light_lane_trun_type = "
         << hdmap::Lane_LaneTurn_Name(signal_light_lane_trun_type);
  auto sub_signal_type = signal_info_ptr->signal().subsignal()[0].type();
  ADEBUG << "traffic_light_overlap.object_id = "
         << traffic_light_overlap.object_id << " sub_signal_type = "
         << hdmap::Subsignal_Type_Name(sub_signal_type);
  if (hdmap::Lane::NO_TURN == signal_light_lane_trun_type) {
    if (hdmap::Subsignal::ARROW_FORWARD != sub_signal_type &&
        hdmap::Subsignal::CIRCLE != sub_signal_type &&
        hdmap::Subsignal::ARROW_LEFT_AND_FORWARD != sub_signal_type &&
        hdmap::Subsignal::ARROW_RIGHT_AND_FORWARD != sub_signal_type) {
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      return false;
    } else {
      (*traffic_light_matchings)[signal_light_lane_id].traffic_light_id =
          traffic_light_overlap.object_id;
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      (*traffic_light_matchings)[signal_light_lane_id].is_matching = true;
    }
  } else if (hdmap::Lane::LEFT_TURN == signal_light_lane_trun_type) {
    if (hdmap::Subsignal::ARROW_LEFT != sub_signal_type &&
        hdmap::Subsignal::ARROW_LEFT_AND_FORWARD != sub_signal_type) {
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      return false;
    } else {
      (*traffic_light_matchings)[signal_light_lane_id].traffic_light_id =
          traffic_light_overlap.object_id;
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      (*traffic_light_matchings)[signal_light_lane_id].is_matching = true;
    }
  } else if (hdmap::Lane::RIGHT_TURN == signal_light_lane_trun_type) {
    if (hdmap::Subsignal::ARROW_RIGHT != sub_signal_type &&
        hdmap::Subsignal::ARROW_RIGHT_AND_FORWARD != sub_signal_type) {
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      return false;
    } else {
      (*traffic_light_matchings)[signal_light_lane_id].traffic_light_id =
          traffic_light_overlap.object_id;
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      (*traffic_light_matchings)[signal_light_lane_id].is_matching = true;
    }
  } else if (hdmap::Lane::U_TURN == signal_light_lane_trun_type) {
    if (hdmap::Subsignal::ARROW_U_TURN != sub_signal_type) {
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      return false;
    } else {
      (*traffic_light_matchings)[signal_light_lane_id].traffic_light_id =
          traffic_light_overlap.object_id;
      (*traffic_light_matchings)[signal_light_lane_id].start_s =
          traffic_light_overlap.start_s;
      (*traffic_light_matchings)[signal_light_lane_id].end_s =
          traffic_light_overlap.end_s;
      (*traffic_light_matchings)[signal_light_lane_id].is_matching = true;
    }
  }

  return true;
}

void TrafficLight::CheckTrafficLightRequest(
    ReferenceLineInfo* const reference_line_info,
    const perception::TrafficLight_Color signal_color, const double stop_s) {
  if (perception::TrafficLight::UNKNOWN == signal_color) {
    const double adc_v = injector_->vehicle_state()->linear_velocity();
    const auto& adc_sl = reference_line_info->AdcSlBoundary();
    if (adc_v < kEStopSpeed && stop_s > adc_sl.end_s() &&
        stop_s - adc_sl.end_s() < kDistanceBuffer) {
      // Request remote takeover traffic light
      reference_line_info->SetTrafficLightRequest(true);
    }
  }
}

void TrafficLight::MakeDecisionsForNoMatching(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const std::unordered_map<std::string, TrafficLightMatching>&
        traffic_light_matchings,
    const double adc_back_edge_s) {
  TrafficLightMatching traffic_light_matching;
  traffic_light_matching.start_s = std::numeric_limits<double>::max();
  for (const auto& matching : traffic_light_matchings) {
    if (matching.second.end_s + kForwardBuffer <= adc_back_edge_s) {
      continue;
    }

    if (!matching.second.is_matching) {
      AERROR << "Map binding traffic light error, so that failed get matched "
                "traffic light, lane id : "
             << matching.first;
      // build stop decision
      ADEBUG << "BuildStopDecision: traffic_light["
             << matching.second.traffic_light_id << "] start_s["
             << matching.second.start_s << "]";
      std::string virtual_obstacle_id =
          TRAFFIC_LIGHT_VO_ID_PREFIX + matching.second.traffic_light_id +
          std::string("_") + std::to_string(matching.second.start_s);
      const std::vector<std::string> wait_for_obstacles;
      util::BuildStopDecision(virtual_obstacle_id, matching.second.start_s,
                              config_.traffic_light().stop_distance(),
                              StopReasonCode::STOP_REASON_SIGNAL,
                              wait_for_obstacles,
                              TrafficRuleConfig::RuleId_Name(config_.rule_id()),
                              frame, reference_line_info);
    }

    if (matching.second.start_s < traffic_light_matching.start_s) {
      traffic_light_matching.traffic_light_id =
          matching.second.traffic_light_id;
      traffic_light_matching.start_s = matching.second.start_s;
      traffic_light_matching.is_matching = matching.second.is_matching;
    }
  }
  ADEBUG << "traffic_light_matching.traffic_light_id: "
         << traffic_light_matching.traffic_light_id;
  frame->SetCurrentTrafficLightId(traffic_light_matching.traffic_light_id);
}

}  // namespace planning
}  // namespace century
