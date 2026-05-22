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

#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_decider.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/time/clock.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

using century::common::Status;
using century::cyber::Clock;

LaneOvertakeDecider::LaneOvertakeDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  overtake_fsm_ = std::make_shared<OverTakeFsm>();
  overtake_fsm_->InitOverTakeFsm(config, injector);
}

Status LaneOvertakeDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Check if over take is needed, if so, over take.
  if (Decider::config_.lane_overtake_decider_config().allow_overtake_fsm()) {
    overtake_fsm_->ExecuteFsm(frame, reference_line_info);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace century
