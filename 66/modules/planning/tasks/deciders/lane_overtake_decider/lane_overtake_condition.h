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

#pragma once

#include <memory>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_fsm_common.h"

namespace century {
namespace planning {

class FsmCondition {
 public:
  explicit FsmCondition(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

  bool Default2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Default2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Prepare2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Prepare2Turn(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);
  bool Prepare2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Turn2Return(Frame* const frame,
                   ReferenceLineInfo* const reference_line_info);
  bool Turn2Overtake(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool Turn2Turn(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);
  bool Overtake2Finish(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Overtake2Fail(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool Overtake2Return(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Overtake2Overtake(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);
  bool Return2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool Return2Hold(Frame* const frame,
                   ReferenceLineInfo* const reference_line_info);
  bool Return2Return(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool Hold2Fail(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);
  bool Hold2Return(Frame* const frame,
                   ReferenceLineInfo* const reference_line_info);
  bool Hold2Hold(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);
  bool Finish2Default(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);
  bool Finish2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool Fail2Default(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);
  bool Fail2Fail(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

 private:
  bool BlockStopForLong(Frame* const frame,
                        ReferenceLineInfo* const reference_line_info);

 private:
  int cancel_turn_count_ = 0;
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace century
