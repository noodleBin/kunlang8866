/******************************************************************************
 * Copyright 2020 The Century Authors. All Rights Reserved.
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

#include "modules/planning/tasks/task.h"

namespace century {
namespace planning {

class LearningModelInferenceTrajectoryTask : public Task {
 public:
  LearningModelInferenceTrajectoryTask(
      const TaskConfig &config,
      const std::shared_ptr<DependencyInjector> &injector);

  century::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  century::common::Status Process(Frame *frame,
                                 ReferenceLineInfo *reference_line_info);
};

}  // namespace planning
}  // namespace century
