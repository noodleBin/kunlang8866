/******************************************************************************
 * Copyright 2024 The Century Authors. All Rights Reserved.
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

#include "mcloud.h"
#include "modules/mcloud/common/cloud_gflags.h"

namespace century {
namespace mcloud {

MCloud::~MCloud() {
  AINFO << "[MCloud] destructor begin";
  cloud_mqtt_.Stop();
  // cloud_.Stop();
  client_.StopReceiving();
  AINFO << "[MCloud] destructor end";
}

bool MCloud::Init() {
  AINFO << "[MCloud] Init begin";
  client_.SetAttribute(FLAGS_cloud_server_ip, FLAGS_cloud_server_port);

  cloud_.RegisterClient(&client_, nullptr);
  client_.StartReceiving(cloud_);

  if (!cloud_mqtt_.Start()) {
    AWARN << "cloud mqtt startup failed, continue without mqtt";
  } else {
    AINFO << "[MCloud] cloud mqtt started";
  }
  cloud_.Start();
  AINFO << "[MCloud] Init end";
  return true;
}

bool MCloud::Proc() {

  return true;
}

}
}
