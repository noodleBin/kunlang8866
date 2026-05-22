/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/localization/proto/localization.pb.h"
#include "modules/storytelling/proto/story.pb.h"

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/message/raw_message.h"
#include "modules/prediction/common/message_process.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/submodules/submodule_output.h"

using century::localization::LocalizationEstimate;

namespace century {
namespace prediction {

using PredictionWriter = cyber::Writer<PredictionObstacles>;
using PredictionWriterPtr = std::shared_ptr<PredictionWriter>;
using LocalizationReaderPtr =
    std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>;
using NodePtr = std::shared_ptr<cyber::Node>;

class PredictionVirtualObstaclesComponent : public cyber::Component<> {
 public:
  PredictionVirtualObstaclesComponent() = default;
  ~PredictionVirtualObstaclesComponent();

  bool Init() override;

 private:
  void PublishObstaclesTask() noexcept;

 private:
  std::thread task_thread_;
};

CYBER_REGISTER_COMPONENT(PredictionVirtualObstaclesComponent);

}  // namespace prediction
}  // namespace century