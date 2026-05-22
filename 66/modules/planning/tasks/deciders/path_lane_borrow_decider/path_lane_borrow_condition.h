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
#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_fsm_common.h"

namespace century {
namespace planning {

class LaneBorrowFsmCondition {
 public:
  explicit LaneBorrowFsmCondition(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

  bool Default2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Default2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Prepare2Default(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool Prepare2LeftBorrow(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info);
  bool Prepare2RightBorrow(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  bool Prepare2Prepare(Frame* const frame,
                       ReferenceLineInfo* const reference_line_info);
  bool LeftBorrow2Return(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);
  bool LeftBorrow2LeftBorrow(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info);
  bool RightBorrow2Return(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info);
  bool RightBorrow2RightBorrow(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);
  bool Return2LeftBorrow(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);
  bool Return2RightBorrow(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info);
  bool Return2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool Return2Return(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool Finish2Default(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);
  bool Finish2Finish(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);

 private:
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace century
