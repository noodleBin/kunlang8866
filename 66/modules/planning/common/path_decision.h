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
#include <string>

#include "modules/planning/proto/decision.pb.h"

#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/obstacle.h"

namespace century {
namespace planning {

/**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */
class PathDecision {
 public:
  PathDecision() = default;

  Obstacle *AddObstacle(const Obstacle &obstacle);

  const IndexedList<std::string, Obstacle> &obstacles() const;

  bool AddLateralDecision(const std::string &tag, const std::string &object_id,
                          const ObjectDecisionType &decision);
  bool AddLongitudinalDecision(const std::string &tag,
                               const std::string &object_id,
                               const ObjectDecisionType &decision);

  const Obstacle *Find(const std::string &object_id) const;

  const perception::PerceptionObstacle *FindPerceptionObstacle(
      const std::string &perception_obstacle_id) const;

  Obstacle *Find(const std::string &object_id);

  void SetSTBoundary(const std::string &id, const STBoundary &boundary);
  void EraseStBoundaries();
  MainStop main_stop() const { return main_stop_; }
  double stop_reference_line_s() const { return stop_reference_line_s_; }
  bool MergeWithMainStop(const ObjectStop &obj_stop, const std::string &obj_id,
                         const ReferenceLine &ref_line,
                         const SLBoundary &adc_sl_boundary);
  double linear_velocity() const { return linear_velocity_; }
  void set_linear_velocity(const double velocity) {
    linear_velocity_ = velocity;
  }

 private:
  IndexedList<std::string, Obstacle> obstacles_;
  MainStop main_stop_;
  double stop_reference_line_s_ = std::numeric_limits<double>::max();
  double linear_velocity_ = 0.0;
};

}  // namespace planning
}  // namespace century
