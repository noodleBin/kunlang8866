/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/planning/tasks/deciders/decider.h"

namespace century {
namespace planning {

enum PathRelationType {
  NONE_RELATION = 0,
  SAME_DIRECTION_OVERLAP = 1,
  OPPOSITE_DIRECTION_OVERLAP = 2,
  CROSSING_RELATION = 3,
};

class TopBullDecider : public Decider {
 public:
  TopBullDecider(const TaskConfig& config,
                 const std::shared_ptr<DependencyInjector>& injector);

  PathRelationType AnalyzePathRelation(
      const std::vector<planning::PathInfo>& path_a,
      const std::vector<planning::PathInfo>& path_b,
      double dist_threshold = 2.0, double angle_threshold = M_PI / 2);

 private:
  century::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  void MakeTopBullDecision(Frame* frame,
                           ReferenceLineInfo* reference_line_info);

  bool CheckTopBullConditions(std::string& top_bull_msg,
                              std::string& blocking_igv_vehicle_id,
                              VehicleInfo& blocking_igv_info,
                              TopBullStatus* top_bull);

  void MakeTopBullAction(const VehicleInfo& blocking_igv_info,
                         const PathRelationType& path_relation_type,
                         std::string& top_bull_msg, TopBullStatus* top_bull);

  void CheckExitTopBull(std::string& top_bull_msg, TopBullStatus* top_bull);

  void CalculateMeanAndVar(const std::vector<int>& data, double& mean,
                           double& variance);

  bool NoPathCollision(const std::string& blocking_igv_vehicle_id);

  uint64_t GenerateRandomNumber();

  void ClearTopBullStatus(TopBullStatus* top_bull);

 private:
  Frame* frame_;
  ReferenceLineInfo* reference_line_info_;
};

}  // namespace planning
}  // namespace century
