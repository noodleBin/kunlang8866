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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/planning/common/planning_context.h"
#include "modules/planning/traffic_rules/traffic_rule.h"

namespace century {
namespace planning {
using century::perception::PerceptionObstacle;
using CrosswalkToStop =
    std::vector<std::pair<const hdmap::PathOverlap*, std::vector<std::string>>>;
using CrosswalkStopTimer =
    std::unordered_map<std::string, std::unordered_map<std::string, double>>;

class Crosswalk : public TrafficRule {
 public:
  Crosswalk(const TrafficRuleConfig& config,
            const std::shared_ptr<DependencyInjector>& injector);
  virtual ~Crosswalk() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool FindCrosswalks(ReferenceLineInfo* const reference_line_info);
  bool CheckStopForObstacle(ReferenceLineInfo* const reference_line_info,
                            const hdmap::CrosswalkInfoConstPtr crosswalk_ptr,
                            const Obstacle& obstacle,
                            const double stop_deceleration);
  bool CheckObstacleType(const PerceptionObstacle::Type& obstacle_type,
                         const std::string& obstacle_id,
                         const std::string& obstacle_type_name);
  bool CheckObstacleDirection(ReferenceLineInfo* const reference_line_info,
                              const Obstacle& obstacle);
  bool ExpandCrosswalkPolygon(ReferenceLineInfo* const reference_line_info,
                              const Obstacle& obstacle,
                              const hdmap::CrosswalkInfoConstPtr crosswalk_ptr);
  void CheckStopForPedestrian(const Obstacle& obstacle,
                              const std::string& crosswalk_id,
                              bool* const stop);
  void CheckStopDeceleration(const double stop_deceleration,
                             const double obstacle_l_distance,
                             const std::string& crosswalk_id, bool* const stop);
  void MakeDecisionsForObstacle(
      ReferenceLineInfo* const reference_line_info,
      const hdmap::CrosswalkInfoConstPtr crosswalk_ptr,
      const double stop_deceleration, const double adc_front_edge_s,
      const std::string& crosswalk_id,
      const hdmap::PathOverlap* crosswalk_overlap,
      CrosswalkStopTimer* crosswalk_stop_timer,
      std::vector<std::string>* const pedestrians);
  void BuildStopDecisionAndUpdateCrosswalkStatus(
      Frame* const frame, ReferenceLineInfo* const reference_line_info,
      const CrosswalkToStop& crosswalks_to_stop,
      CrosswalkStopTimer* crosswalk_stop_timer,
      CrosswalkStatus* mutable_crosswalk_status);

 private:
  static constexpr char const* CROSSWALK_VO_ID_PREFIX = "CW_";
  std::vector<const hdmap::PathOverlap*> crosswalk_overlaps_;
};

}  // namespace planning
}  // namespace century
