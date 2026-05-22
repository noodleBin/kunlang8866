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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "modules/common/kv_db/kv_db.h"
#include "cyber/service/service.h"
#include "modules/fas_aeb_backend/proto/fas_aeb_backend.pb.h"
#include "modules/planning/proto/planning_aeb.pb.h"

namespace century {
namespace fast_aeb_backend {

using century::cyber::Component;

class FAS_AEB_BACKEND : public century::cyber::TimerComponent{
 public:
  FAS_AEB_BACKEND();
  ~FAS_AEB_BACKEND();;
  bool Init() override;
  bool Proc() override;
  bool VerifyPassword(const std::string& input_pwd, const std::string& input_pwd_long = "");
  bool ModifyPassword(const std::string& old_pwd, const std::string& new_pwd, bool is_short = false);

  bool ModifyConfig(const std::string& key, const std::string& value);
  void OnAebResult(const std::shared_ptr<century::planning::AebResult>& aeb_result);

 private:
  bool is_short_ = {false};
  double short_time_{0.0};
  double long_time_{0.0};
  bool last_aeb_status_ = {true};
  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Service<century::fas_aeb_backend::Request, century::fas_aeb_backend::Response>> service_;
  std::shared_ptr<cyber::Writer<century::fas_aeb_backend::FasAebInfo>> fas_aeb_info_writer_;
  std::shared_ptr<cyber::Reader<century::planning::AebResult>> aeb_result_reader_;
};

CYBER_REGISTER_COMPONENT(FAS_AEB_BACKEND)

}  // namespace fast_aeb_backend
}  // namespace century
