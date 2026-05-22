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

#include <boost/format.hpp>
#include <boost/uuid/detail/sha1.hpp>
#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "led_controller.h"
#include "nlohmann/json.hpp"

namespace century {
namespace led_monitor {
using json = nlohmann::json;

std::string Sha1Boost(const std::string &input) {
  boost::uuids::detail::sha1 sha1;
  sha1.process_bytes(input.c_str(), input.size());

  unsigned int hash[5];
  sha1.get_digest(hash);

  std::ostringstream oss;
  for (unsigned int i: hash) { oss << std::hex << std::setw(8) << std::setfill('0') << i; }
  return oss.str();
}

LedController::LedController(const std::string &ip) : ip_(ip) {}

/*
 *
 * POST / HTTP/1.1
 * Host: 192.168.0.199
 * Accept:text/json
 * Content-Type: application/json;charset=UTF-8
 * Content-Length: 96
 * {"protocol":{"name":"YQ-COM2","version":"1.0", "remotefunction":{"name":"getVerificationCode"}}}
 *
 * POST / HTTP/1.1
 * Host: 192.168.8.39
 * Accept:text/json
 * Content-Type: application/json;charset=UTF-8
 * Content-Length: 212
 * {"protocol":{"name":"YQ-COM2","version":"1.0","remotefunction":{"name": "UserLogin","input":{"password":"35c83b5baf44bf36c8340fa87845d8043a46a225","username":"guest","verificationcode":"20180517093429462773"}}}}
 *
 */

bool LedController::Init(const LedMonitorConfig& config) {
  is_front_ = config.is_front();
  for (std::size_t i = 0; i < 30; i++) {
    valid_display_types_.emplace(i);
  }

  for (size_t i = 0; i < 30; i++) {
    program_map_[i] = i;
    if (is_front_) {
      program_map_[static_cast<int>(planning::DisplayType::TURN_LEFT)] = 4;
      program_map_[static_cast<int>(planning::DisplayType::TURN_RIGHT)] = 3;
    }
  }
  

  if (config.post_process()) {
    program_map_ = {
      {static_cast<int>(planning::DisplayType::U_TURN), 3},
      {static_cast<int>(planning::DisplayType::DIAGNAL_LEFT), 3},
      {static_cast<int>(planning::DisplayType::DIAGNAL_RIGHT), 4}
    };

    if (is_front_) {
      program_map_[static_cast<int>(planning::DisplayType::U_TURN)] = 4;
      program_map_[static_cast<int>(planning::DisplayType::DIAGNAL_LEFT)] = 4;
      program_map_[static_cast<int>(planning::DisplayType::DIAGNAL_RIGHT)] = 3;
    }
  }

  http_client_ = std::make_unique<httplib::Client>(ip_);
  if (!http_client_) {
    AERROR << "Failed to create http client.";
    return false;
  }
  http_client_->set_connection_timeout(std::chrono::seconds(5));

  httplib::Headers headers = {{"Accept", "text/json"}, {"Content-Type", "application/json;charset=UTF-8"}};

  http_client_->set_default_headers(headers);

  // comment it in case of led monitor is offline or not connected
  // connect and login
  // if (!Connect()) {
  //   AERROR << "Failed to connect";
  //   return false;
  // }

  return true;
}

bool LedController::Stop() {
  // if (status_ == Status::CONNECTED) {
  //   if (!UserLogOut()) {
  //     AERROR << "Failed to logout";
  //     return false;
  //   }
  // }

  return true;
}

bool LedController::Process(const std::shared_ptr<planning::ADCTrajectory>& msg) {
  auto program = msg->display_type();

  // check if the display type is valid or not
  if (valid_display_types_.end() == valid_display_types_.find(static_cast<int>(program))) {
    AERROR << "Invalid display type: " << static_cast<int>(program);
    return false;
  }

  // get the program index
  auto program_index = program_map_[static_cast<int>(program)];

  if (PlayProgram(program_index)) {
    AINFO << "Play program: " << program_index; // play program
  } else {
    AERROR << "Failed to play program: " << program_index;// failed to play program
    return false;
  }

  return true;
}

bool LedController::PlayProgram(const int program) {
  AINFO << "PlayProgram: " << program;
  if (current_program_.load() == program) {
    AINFO << "Already playing program: " << program;
    return true;
  }
  
  current_program_.store(-1);
  if (status_ != Status::CONNECTED) {
    AWARN << "Not connected reconnecting";
    if (!Connect()) {
      AERROR << "Failed to connect";
      return false;
    }

    if (!LockProgram(program)) {
      AERROR << "Failed to lock program";
      status_ = Status::ERROR;
      return false;
    }

    return true;
  } else {
    if (!LockProgram(program)) {
      AERROR << "Failed to lock program";
      status_ = Status::ERROR;
      return false;
    }
  }
  current_program_.store(program);

  return true;
}

bool LedController::Connect() {
  if (Status::CONNECTED == status_) {
    AINFO << "Already connected";
    return true;
  }

  if (!GetVerificationCode()) {
    AERROR << "Failed to get verification code";
    return false;
  }

  if (!UserLogin()) {
    AERROR << "Failed to login";
    return false;
  }

  status_ = Status::CONNECTED;
  return true;
}

void LedController::ShowHttpLogInfo(const httplib::Result &res) {
  AINFO << "================================================================";
  auto err = res.error();
  AINFO << "error: " << httplib::to_string(err);
  AINFO << "status: " << res->status;
  AINFO << "body: " << res->body;
  AINFO << "================================================================";
}

void LedController::ParseRawData(const std::string &msg) {}

bool LedController::GetVerificationCode() {
  // JSON data to be sent
  std::string json_body = R"({"protocol":{"name":"YQ-COM2","version":"1.0","remotefunction":{"name":"getVerificationCode"}}})";

  // Send POST request
  auto res = http_client_->Post("/", json_body, "application/json");

  // Handle response
  if (res) {
    ShowHttpLogInfo(res);
    auto body = res->body;

    try {
      // Parse JSON string
      json j = json::parse(body);

      // Check if "remotefunction" field exists and is an object
      if (!j.contains("remotefunction") || !j["remotefunction"].is_object()) {
        AWARN << "Missing or invalid 'remotefunction' field";
        return false;
      }
      json remotefunction = j["remotefunction"];

      // Check if "name" field exists and is a string, and its value must be "getverificationcode"
      if (!remotefunction.contains("name") || !remotefunction["name"].is_string() ||
          remotefunction["name"] != "getverificationcode") {
        AWARN << "Missing or invalid 'name' field, or its value is not 'getverificationcode'";
        return false;
      }

      // Check if "output" field exists and is an object
      if (!remotefunction.contains("output") || !remotefunction["output"].is_object()) {
        AWARN << "Missing or invalid 'output' field";
        return false;
      }
      json output = remotefunction["output"];

      // Check if "verificationcode" field exists and is a string
      if (!output.contains("verificationcode") || !output["verificationcode"].is_string()) {
        AWARN << "Missing or invalid 'verificationcode' field";
        return false;
      }

      // Get the value of the verificationcode field
      std::string verificationCode = output["verificationcode"];
      AINFO << "verificationcode: " << verificationCode;
      verification_code_ = verificationCode;
    }
    // Catch nlohmann::json related exceptions (e.g., parsing errors, access errors, etc.)
    catch (const nlohmann::json::exception &e) {
      AERROR << "JSON parsing or access error: " << e.what();
      return false;
    }
    // Catch other standard exceptions
    catch (const std::exception &e) {
      AERROR << "Exception: " << e.what();
      return false;
    }
    // Catch all other unknown exceptions
    catch (...) {
      AERROR << "Unknown exception occurred";
      return false;
    }

  } else {
    AERROR << "Request failed: " << res.error();
    return false;
  }

  return true;
}

// Function: Construct JSON string based on verificationcode and password
std::string LedController::CreateUserLoginJson() {
  auto password = Sha1Boost(verification_code_ + Sha1Boost("guest"));

  // Use Raw string to define JSON template, %1% and %2% as placeholders
  std::string json_template =
    R"({"protocol":{"name":"YQ-COM2","version":"1.0","remotefunction":{"name":"userLogin","input":{"username":"guest","verificationcode":"%1%","password":"%2%"}}}})";
  // Use boost::format to replace placeholders with corresponding parameters
  return (boost::format(json_template) % verification_code_ % password).str();
}

bool LedController::UserLogin() {
  AINFO << "UserLogin: ";
  // JSON data to be sent
  std::string json_body = CreateUserLoginJson();

  // Send POST request
  auto res = http_client_->Post("/", json_body, "application/json");

  // Handle response
  if (res) {
    ShowHttpLogInfo(res);
    auto body = res->body;

    try {
      // Parse JSON string
      json j = json::parse(body);

      // Check if "remotefunction" field exists and is an object
      if (!j.contains("remotefunction") || !j["remotefunction"].is_object()) {
        AWARN << "Missing or invalid 'remotefunction' field";
        return false;
      }
      json remotefunction = j["remotefunction"];

      // Check if "name" field exists and is a string, and its value must be "userlogin"
      if (!remotefunction.contains("name") || !remotefunction["name"].is_string() || remotefunction["name"] != "userlogin") {
        AWARN << "Missing or invalid 'name' field, or its value is not 'userLogin'";
        return false;
      }

      // Check if "output" field exists and is an object
      if (!remotefunction.contains("output") || !remotefunction["output"].is_object()) {
        AWARN << "Missing or invalid 'output' field";
        return false;
      }
      json output = remotefunction["output"];

      // Check if "sessionID" field exists and is a string
      if (!output.contains("sessionID") || !output["sessionID"].is_string()) {
        AWARN << "Missing or invalid 'sessionID' field";
        return false;
      }

      // Get the value of the session_id field
      std::string session_id = output["sessionID"];
      AINFO << "session_id: " << session_id;
      session_id_ = session_id;
    }
    // Catch nlohmann::json related exceptions (e.g., parsing errors, access errors, etc.)
    catch (const nlohmann::json::exception &e) {
      AERROR << "JSON parsing or access error: " << e.what();
      return false;
    }
    // Catch other standard exceptions
    catch (const std::exception &e) {
      AERROR << "Exception: " << e.what();
      return false;
    }
    // Catch all other unknown exceptions
    catch (...) {
      AERROR << "Unknown exception occurred";
      return false;
    }
  } else {
    AERROR << "Request failed: " << res.error();
    return false;
  }
  return true;
}

bool LedController::UserLogOut() {
  AINFO << "UserLogOut: ";
  // Use Raw string to define JSON template, %1% and %2% as placeholders
  std::string json_body =
    R"({"protocol":{"name":"YQ-COM2","version":"1.0","remotefunction":{"name":"userLogout","input":{"username":"guest"}}}})";

  // Send POST request
  auto res = http_client_->Post("/", json_body, "application/json");

  // Handle response
  if (res) {
    ShowHttpLogInfo(res);
    auto body = res->body;

    try {
      // Parse JSON string
      json j = json::parse(body);

      // Check if "remotefunction" field exists and is an object
      if (!j.contains("remotefunction") || !j["remotefunction"].is_object()) {
        AWARN << "Missing or invalid 'remotefunction' field";
        return false;
      }
      json remotefunction = j["remotefunction"];

      // Check if "name" field exists and is a string, and its value must be "userLogout"
      if (!remotefunction.contains("name") || !remotefunction["name"].is_string() || remotefunction["name"] != "userLogout") {
        AWARN << "Missing or invalid 'name' field, or its value is not 'userLogout'";
        return false;
      }

      // Check if "output" field exists and is an object
      if (!remotefunction.contains("output") || !remotefunction["output"].is_object()) {
        AWARN << "Missing or invalid 'output' field";
        return false;
      }
      json output = remotefunction["output"];

      // Check if "type" field exists and is a string
      if (!output.contains("type") || !output["type"].is_string() || output["type"] != "true") {
        AWARN << "Missing or invalid 'type' field or type != 'true'";
        return false;
      }
    }
    // Catch nlohmann::json related exceptions (e.g., parsing errors, access errors, etc.)
    catch (const nlohmann::json::exception &e) {
      AERROR << "JSON parsing or access error: " << e.what();
      return false;
    }
    // Catch other standard exceptions
    catch (const std::exception &e) {
      AERROR << "Exception: " << e.what();
      return false;
    }
    // Catch all other unknown exceptions
    catch (...) {
      AERROR << "Unknown exception occurred";
      return false;
    }

  } else {
    AERROR << "Request failed: " << res.error();
    return false;
  }
  return true;
}

// Function: Construct JSON string based on program
std::string LedController::CreateLockProgramJson(const int program) {
  // Use Raw string to define JSON template, %1% and %2% as placeholders
  std::string json_template =
    R"({"protocol":{"name":"YQ-COM2","version":"1.0","remotefunction":{"name":"lockProgram","input":{"programlockedstatus":"on","programlockedorder":"%1%","notsave":"yes"}}}})";
  // Use boost::format to replace placeholders with corresponding parameters
  return (boost::format(json_template) % program).str();
}

bool LedController::LockProgram(const int program) {
  AINFO << "LockProgram: " << program;
  std::string path = "/;stok=" + session_id_ + "/";
  // JSON data to be sent
  std::string json_body = CreateLockProgramJson(program);
  AINFO << json_body;

  // Send POST request
  auto res = http_client_->Post(path, json_body, "application/json");

  // Handle response
  if (res) {
    ShowHttpLogInfo(res);
    auto body = res->body;

    try {
      // Parse JSON string
      json j = json::parse(body);

      // Check if "remotefunction" field exists and is an object
      if (!j.contains("remotefunction") || !j["remotefunction"].is_object()) {
        AWARN << "Missing or invalid 'remotefunction' field";
        return false;
      }
      json remotefunction = j["remotefunction"];

      // Check if "name" field exists and is a string, and its value must be "lockprogram"
      if (!remotefunction.contains("name") || !remotefunction["name"].is_string() || remotefunction["name"] != "lockprogram") {
        AWARN << "Missing or invalid 'name' field, or its value is not 'lockprogram'";
        return false;
      }

      // Check if "output" field exists and is an object
      if (!remotefunction.contains("output") || !remotefunction["output"].is_object()) {
        AWARN << "Missing or invalid 'output' field";
        return false;
      }
      json output = remotefunction["output"];

      // Check if "type" field exists and is a string
      if (!output.contains("type") || !output["type"].is_string() || output["type"] != "true") {
        AWARN << "Missing or invalid 'type' field or type != 'true'";
        return false;
      }
    }
    // Catch nlohmann::json related exceptions (e.g., parsing errors, access errors, etc.)
    catch (const nlohmann::json::exception &e) {
      AERROR << "JSON parsing or access error: " << e.what();
      return false;
    }
    // Catch other standard exceptions
    catch (const std::exception &e) {
      AERROR << "Exception: " << e.what();
      return false;
    }
    // Catch all other unknown exceptions
    catch (...) {
      AERROR << "Unknown exception occurred";
      return false;
    }

  } else {
    AERROR << "Request failed: " << res.error();
    return false;
  }

  return true;
}
}  // namespace led_monitor
}  // namespace century