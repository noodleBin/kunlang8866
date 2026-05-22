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

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

#include "modules/planning/traffic_rules/traffic_rule.h"
namespace century {
namespace planning {

struct TrafficLightMatching {
  TrafficLightMatching() = default;
  std::string traffic_light_id = "-1";
  double start_s = 0.0;
  double end_s = 0.0;
  bool is_matching = false;
};

class TrafficLight : public TrafficRule {
 public:
  TrafficLight(const TrafficRuleConfig& config,
               const std::shared_ptr<DependencyInjector>& injector);

  virtual ~TrafficLight() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);

  bool CheckRoundRouting(ReferenceLineInfo* const reference_line_info,
                         const hdmap::PathOverlap& traffic_light_overlap,
                         const double adc_front_edge_s);
  bool CheckTrafficLightMatching(
      ReferenceLineInfo* const reference_line_info,
      const hdmap::PathOverlap& traffic_light_overlap,
      std::unordered_map<std::string, TrafficLightMatching>* const
          traffic_light_matchings);
  void CheckTrafficLightRequest(
      ReferenceLineInfo* const reference_line_info,
      const perception::TrafficLight_Color signal_color, const double stop_s);
  void MakeDecisionsForNoMatching(
      Frame* const frame, ReferenceLineInfo* const reference_line_info,
      const std::unordered_map<std::string, TrafficLightMatching>&
          traffic_light_matchings,
      const double adc_back_edge_s);

 private:
  static constexpr char const* TRAFFIC_LIGHT_VO_ID_PREFIX = "TL_";
};

}  // namespace planning
}  // namespace century
