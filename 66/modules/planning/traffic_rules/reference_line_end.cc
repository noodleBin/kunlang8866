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

#include "modules/planning/traffic_rules/reference_line_end.h"

#include <memory>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"

namespace century {
namespace planning {

using century::common::Status;

ReferenceLineEnd::ReferenceLineEnd(
    const TrafficRuleConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : TrafficRule(config, injector) {}

Status ReferenceLineEnd::ApplyRule(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();

  ADEBUG << "ReferenceLineEnd length[" << reference_line.Length() << "]";
  for (const auto& segment : reference_line_info->Lanes()) {
    ADEBUG << "   lane[" << segment.lane->lane().id().id() << "]";
  }
  // check
  double remain_s =
      reference_line.Length() - reference_line_info->AdcSlBoundary().end_s();
  if (remain_s >
      config_.reference_line_end().min_reference_line_remain_length()) {
    return Status::OK();
  }

  // create avirtual stop wall at the end of reference line to stop the adc
  std::string virtual_obstacle_id =
      REF_LINE_END_VO_ID_PREFIX + reference_line_info->Lanes().Id();
  double obstacle_start_s =
      reference_line.Length() - 2 * FLAGS_virtual_stop_wall_length;
  const std::vector<std::string> wait_for_obstacle_ids;

  int result = util::BuildStopDecision(
      virtual_obstacle_id, obstacle_start_s,
      config_.reference_line_end().stop_distance(),
      StopReasonCode::STOP_REASON_REFERENCE_END, wait_for_obstacle_ids,
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), frame,
      reference_line_info);

  if (-1 == result) {
    return Status(common::PLANNING_ERROR,
                  "Failed to build stop decision on reference line end");
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace century
