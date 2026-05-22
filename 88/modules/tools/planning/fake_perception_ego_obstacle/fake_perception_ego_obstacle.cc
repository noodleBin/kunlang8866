/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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
 * @file fake_perception_ego_obstacle.cc
 */

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/file.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

DEFINE_string(fake_perception_ego_obstacle_test_file,
              "modules/tools/planning/fake_perception_ego_obstacle/fake_perception_ego_obstacle.txt",
              "Used for sending fake_perception_ego_obstacle test obstacle.");

namespace century {
namespace plann {

class FakePerceptionEgoObstacleComponent
    : public century::cyber::TimerComponent {
 public:
  bool Init() override {
    fake_perception_ego_obstacle_writer_ =
        node_->CreateWriter<perception::PerceptionObstacles>(
            "/century/perception/around_ego/obstalces");
    AINFO << "create fake perception around ego obstalces writer!!!!";
    return true;
  }
  bool Proc() override {
    // auto prediction = std::make_shared<PredictionObstacles>();
    perception::PerceptionObstacles fake_perception_ego_obstacle_test;
    AINFO << "Sending perception obstacles: "
          << fake_perception_ego_obstacle_test.ShortDebugString();
    if (!cyber::common::GetProtoFromFile("/century/modules/tools/planning/fake_perception_ego_obstacle/fake_perception_ego_obstacle_test.pb.txt",
                                         &fake_perception_ego_obstacle_test)) {
      return false;
    }
    common::util::FillHeader("fake_perception_ego_obstacle",
                             &fake_perception_ego_obstacle_test);

    fake_perception_ego_obstacle_writer_->Write(
        fake_perception_ego_obstacle_test);
    return true;
  }

 private:
  std::shared_ptr<century::cyber::Writer<perception::PerceptionObstacles>>
      fake_perception_ego_obstacle_writer_;
};
CYBER_REGISTER_COMPONENT(FakePerceptionEgoObstacleComponent);

}  // namespace plann
}  // namespace century
