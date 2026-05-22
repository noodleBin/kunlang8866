/******************************************************************************
 * Copyright 2019 The Century Authors. All Rights Reserved.
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

#pragma once

#include <string>
#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"

namespace century {
namespace planning {
namespace util {

int BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);

int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);

void GenerateObjectStopDecision(const Obstacle& obstacle, double stop_distance,
                                const std::string& tag,
                                const ReferenceLineInfo& reference_line_info,
                                ObjectStop* const object_stop);

void GenerateObjectStopDecision(const double stop_limit_s, double stop_distance,
                                const std::string& tag,
                                const ReferenceLineInfo& reference_line_info,
                                ObjectStop* const object_stop);
}  // namespace util
}  // namespace planning
}  // namespace century
