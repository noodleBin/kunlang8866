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
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "modules/fas_aeb_backend/fas_aeb_backend.h"
#include "modules/common/util/message_util.h"

using century::common::KVDB;
using century::common::util::FillHeader;

using century::fas_aeb_backend::Request;
using century::fas_aeb_backend::Response;
using century::fas_aeb_backend::FasAebInfo;
using century::planning::AebResult;

namespace century {
namespace fast_aeb_backend {

FAS_AEB_BACKEND::FAS_AEB_BACKEND() : node_(cyber::CreateNode("1111")) {}

FAS_AEB_BACKEND::~FAS_AEB_BACKEND() {

}

bool FAS_AEB_BACKEND::Init() {
  std::ifstream config_file("/century/modules/fas_aeb_backend/conf/config.json");
  if (!config_file.is_open()) {
      std::cerr << "Failed to open config.json" << std::endl;
      return -1;
  }

  nlohmann::json config;
  config_file >> config;

  short_time_=config["short_time"];
  long_time_=config["long_time"];

  



  if ("" == *KVDB::Get("password")) {
    ModifyPassword("", "123456", true);
  }

  if ("" == *KVDB::Get("password_long")) {
    
    KVDB::Put("password_long", "654321");
    // ModifyPassword("", "654321", false);
  }

  fas_aeb_info_writer_ = node_->CreateWriter<FasAebInfo>("/century/fas_aeb_info");

  aeb_result_reader_ = node_->CreateReader<AebResult>(
      "/century/planning/planning_aeb",
      [this](const std::shared_ptr<AebResult>& aeb_result) {
        this->OnAebResult(aeb_result);
      });

  service_ = node_->CreateService<Request, Response>(
      "fas_aeb_backend", [this](const std::shared_ptr<Request> request,
                            const std::shared_ptr<Response>& response) {
        response->set_result(false);
        
        if (false == request->has_type()) {
          response->set_message("request type is empty");
          return;
        }

        switch(request->type()) {
          case Request::MODIFY_PASSWORD: {
            if (!request->has_modify_password()) {
              response->set_message("modify password request is empty");
              return;
            }
            if (ModifyPassword(request->modify_password().old_password(), request->modify_password().new_password())) {
              response->set_result(true);
              response->set_message("modify password success");
            } else {
              response->set_message("modify password failed");
            }
            break;
          }
          case Request::MODIFY_CONFIG_CLOUD:
            if (!request->has_modify_config()) {
              response->set_message("modify config request is empty");
              return;
            }
            AINFO << "modify config cloud";
            AINFO << "key: " << request->modify_config().key() << ", value: " << request->modify_config().value();
            if (ModifyConfig(request->modify_config().key(), request->modify_config().value())) {
              response->set_result(true);
              is_short_ = true;
              response->set_message("modify config success");
              AINFO << "modify config cloud success";
            } else {
              response->set_message("modify config failed");
              AINFO << "modify config cloud failed";
            }
            break;
          case Request::MODIFY_CONFIG: {
            if (!request->has_modify_config()) {
              response->set_message("modify config request is empty");
              return;
            }
            auto password =  request->modify_config().password();
            auto password_long = request->modify_config().password_longth();
            is_short_ = request->modify_config().is_short();
            if ("false" == request->modify_config().value() && !VerifyPassword(password, password_long)) {
              response->set_message("password error");
              return;
            }
            
            if (ModifyConfig(request->modify_config().key(), request->modify_config().value())) {
              response->set_result(true);
              response->set_message("modify config success");
            } else {
              response->set_message("modify config failed");
            }
            break;
          }
          default:
            response->set_message("unknown request type");
            break;
        }
      });
  ModifyConfig("switch_fas_aeb", "true");
  return true;
}

bool FAS_AEB_BACKEND::VerifyPassword(const std::string& input_pwd, const std::string& input_pwd_long) {
  auto pwd = KVDB::Get("password");
  auto pwd_long = KVDB::Get("password_long");

  if (is_short_) {
    return *pwd == input_pwd;
  }

  return ((*pwd == input_pwd) && (*pwd_long == input_pwd_long));
}

bool FAS_AEB_BACKEND::ModifyPassword(const std::string& old_pwd, const std::string& new_pwd, bool is_short) {
  if (!VerifyPassword(old_pwd)) {
    return false;
  }

  
  if (is_short) {
    return KVDB::Put("password", new_pwd);;
  }
  return true;
}

bool FAS_AEB_BACKEND::ModifyConfig(const std::string& key, const std::string& value) {
  auto now = cyber::Time::Now().ToSecond();
  KVDB::Put("switch_fas_aeb_modify_time", std::to_string(now));
  return KVDB::Put(key, value);
}

void FAS_AEB_BACKEND::OnAebResult(const std::shared_ptr<AebResult>& aeb_result) {
  if (!aeb_result->has_ready_to_enable_aeb()) {
    AERROR << "AebResult does not have ready_to_enable_aeb field";
    return;
  }

  if (aeb_result->ready_to_enable_aeb() && is_short_) {
    ModifyConfig("switch_fas_aeb", "true");
    AINFO << "AEB ready to enable, set switch_fas_aeb to true";
  } else {
    AINFO << "AEB not ready to enable";
  }
}


bool FAS_AEB_BACKEND::Proc() {
  auto status = KVDB::Get("switch_fas_aeb");
  if (status.has_value()) {
   last_aeb_status_ = KVDB::Get("switch_fas_aeb") == "true" ? true : false;
  }
  auto now = cyber::Time::Now().ToSecond();
  auto last_modify_time = std::stod(*KVDB::Get("switch_fas_aeb_modify_time"));
  auto diff = now - last_modify_time;
  auto interval_time = is_short_ ? short_time_ : long_time_;
  if (false == last_aeb_status_ && diff > interval_time) {
    ModifyConfig("switch_fas_aeb", "true");
    last_aeb_status_ = true;
  }
  auto fas_aeb_info_msg = std::make_shared<FasAebInfo>();
  FillHeader("fas_aeb_info", fas_aeb_info_msg.get());
  fas_aeb_info_msg->set_fas_aeb_switch(last_aeb_status_);
  fas_aeb_info_msg->set_last_modify_time(last_modify_time);
  fas_aeb_info_writer_->Write(fas_aeb_info_msg);
  return true;
}

}  // namespace fast_aeb_backend
}  // namespace century
