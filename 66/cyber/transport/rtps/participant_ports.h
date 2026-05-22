/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#ifndef CYBER_TRANSPORT_RTPS_PARTICIPANT_PORTS_H_
#define CYBER_TRANSPORT_RTPS_PARTICIPANT_PORTS_H_

#include "cyber/proto/cyber_conf.pb.h"

namespace century {
namespace cyber {
namespace transport {

constexpr int kDefaultTopologyParticipantPort = 11511;
constexpr int kDefaultTransportParticipantPort = 11512;

inline int NormalizeParticipantPort(uint32_t port, int default_port) {
  if (port == 0 || port > 65535) {
    return default_port;
  }
  return static_cast<int>(port);
}

inline int GetTopologyParticipantPort(const proto::CyberConfig& config) {
  if (!config.has_transport_conf() ||
      !config.transport_conf().has_participant_port()) {
    return kDefaultTopologyParticipantPort;
  }
  return NormalizeParticipantPort(
      config.transport_conf().participant_port().topology_manager_port(),
      kDefaultTopologyParticipantPort);
}

inline int GetTransportParticipantPort(const proto::CyberConfig& config) {
  if (!config.has_transport_conf() ||
      !config.transport_conf().has_participant_port()) {
    return kDefaultTransportParticipantPort;
  }
  return NormalizeParticipantPort(
      config.transport_conf().participant_port().transport_port(),
      kDefaultTransportParticipantPort);
}

}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_RTPS_PARTICIPANT_PORTS_H_
