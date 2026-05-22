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
#include <unordered_map>
#include <unordered_set>

#include "modules/led_monitor/cpphttplib/httplib.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/led_monitor/proto/led_monitor_config.pb.h"

namespace century {
namespace led_monitor {
  /**
   * @brief get the SHA1 hash of the input string using boost library
   * 
   * @param input which is the string to be hashed
   * @return return the hashed string
   */
std::string Sha1Boost(const std::string& input);

/*
 * 操作步骤：
  * 1, Init
  * 2, Connect
  * 3, getVerificationCode
  * 4, userLogin
  * 5, PlayProgram
  * 6, Stop
  * 7, userLogout
  * 8, Disconnect
 */
class LedController {
public:
  explicit LedController(const std::string& ip);
  ~LedController() {}
  bool Init(const LedMonitorConfig& config);
  bool Process(const std::shared_ptr<planning::ADCTrajectory>& msg);
  bool Connect();
  bool PlayProgram(const int program);
  bool Stop();

  // LedController status.
  enum class Status {
    DISCONNECTED,
    CONNECTED,
    ERROR,
  };

private:
  void ShowHttpLogInfo(const httplib::Result& res);
  void ParseRawData(const std::string &msg);
  bool GetVerificationCode();
  bool UserLogin();
  bool UserLogOut();
  bool LockProgram(const int program);
  std::string CreateUserLoginJson();
  std::string CreateLockProgramJson(const int program);
  inline const std::string & VerificationCode() const {
    return verification_code_;
  }
private:
  std::unique_ptr<httplib::Client> http_client_ = nullptr;
  std::string  verification_code_;
  std::string  session_id_;
  std::string  ip_;
  Status status_ = Status::DISCONNECTED;
  bool is_front_ = false;
  std::unordered_set<int> valid_display_types_;
  std::unordered_map<int, int> program_map_;
  std::unique_ptr<cyber::Timer> timer_;
  std::atomic<int> current_program_{-1};
};
}  // namespace led_monitor
}  // namespace century
