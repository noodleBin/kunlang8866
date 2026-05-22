/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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
 */

#include "modules/planning/proto/temporary_parking_request.pb.h"

#include "cyber/common/file.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

DEFINE_string(temporary_parking_request_test_file,
              "modules/tools/planning/fake_temporary_parking_request/fake_temporary_parking_request_test.pb.txt",
              "Used for sending fake_pass_stacker test obstacle.");

namespace century {
namespace plann {



class FakeTemporaryParkingRequestComponent : public century::cyber::TimerComponent {
 public:
  bool Init() override {
    fake_temporary_parking_request_writer_ =
        node_->CreateWriter<planning::TemporaryParkingRequest>("/century/temporary_parking_request");
    return true;
  }
  bool Proc() override {
    // auto prediction = std::make_shared<PredictionObstacles>();
    planning::TemporaryParkingRequest fake_temporary_parking_request_test;
    if (!cyber::common::GetProtoFromFile("/century/modules/tools/planning/fake_temporary_parking_request/temporary_parking_request_test.pb.txt",
                                         &fake_temporary_parking_request_test)) {

      return false;
    }
    common::util::FillHeader("fake_temporary_parking_request", &fake_temporary_parking_request_test);

    fake_temporary_parking_request_writer_->Write(fake_temporary_parking_request_test);
    return true;
  }

 private:
  std::shared_ptr<century::cyber::Writer<planning::TemporaryParkingRequest>>
      fake_temporary_parking_request_writer_;
};
CYBER_REGISTER_COMPONENT(FakeTemporaryParkingRequestComponent);

}  // namespace prediction
}  // namespace century
