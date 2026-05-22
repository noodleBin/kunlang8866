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

#ifndef CYBER_TRANSPORT_RTPS_PARTICIPANT_CONFIG_H_
#define CYBER_TRANSPORT_RTPS_PARTICIPANT_CONFIG_H_

#include <arpa/inet.h>

#include <cstdlib>
#include <string>

#include "cyber/proto/cyber_conf.pb.h"

namespace century {
namespace cyber {
namespace transport {

constexpr char kDefaultRtpsMulticastAddress[] = "239.255.0.1";
constexpr uint32_t kDefaultRtpsDomainId = 80;

inline bool IsValidIpv4Address(const std::string& address) {
  in_addr addr;
  return !address.empty() &&
         inet_pton(AF_INET, address.c_str(), &addr) == 1;
}

inline std::string GetRtpsMulticastAddress(const proto::CyberConfig& config) {
  if (!config.has_transport_conf() ||
      !config.transport_conf().has_participant_attr()) {
    return kDefaultRtpsMulticastAddress;
  }

  const auto& participant_attr = config.transport_conf().participant_attr();
  if (!participant_attr.has_multicast_address()) {
    return kDefaultRtpsMulticastAddress;
  }

  const std::string& multicast_address = participant_attr.multicast_address();
  if (!IsValidIpv4Address(multicast_address)) {
    return kDefaultRtpsMulticastAddress;
  }
  return multicast_address;
}

inline uint32_t GetRtpsDomainId(const proto::CyberConfig& config) {
  uint32_t domain_id = kDefaultRtpsDomainId;
  if (config.has_transport_conf() &&
      config.transport_conf().has_participant_attr()) {
    domain_id = config.transport_conf().participant_attr().domain_id();
  }

  const char* value = ::getenv("CYBER_DOMAIN_ID");
  if (value != nullptr) {
    domain_id = static_cast<uint32_t>(std::stoi(value));
  }
  return domain_id;
}

}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_RTPS_PARTICIPANT_CONFIG_H_
