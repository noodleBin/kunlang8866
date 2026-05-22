/******************************************************************************
 * Copyright 2024 The century Authors. All Rights Reserved.
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

#include "modules/monitor/common/monitor_utils.h"

#include <sstream>
#include <string>

#include "cyber/common/log.h"
namespace century {
namespace monitor {

bool KillProcessByName(const std::string& process_name) {
  std::stringstream ss;
  ss << "ps -ef | grep '" << process_name
     << "' |grep -v grep |awk '{print $2}'|"
     << "xargs kill -9";
  AINFO << "kill process cmd:" << ss.str();
  system(ss.str().c_str());
  return true;
}

}  // namespace monitor
}  // namespace century