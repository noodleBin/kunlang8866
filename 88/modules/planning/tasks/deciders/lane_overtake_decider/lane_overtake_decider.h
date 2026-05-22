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

#include "gtest/gtest.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/tasks/deciders/lane_overtake_decider/lane_overtake_fsm.h"

namespace century {
namespace planning {

class LaneOvertakeDecider : public Decider {
 public:
  LaneOvertakeDecider(const TaskConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) override;

 private:
  std::shared_ptr<OverTakeFsm> overtake_fsm_;
};

}  // namespace planning
}  // namespace century
