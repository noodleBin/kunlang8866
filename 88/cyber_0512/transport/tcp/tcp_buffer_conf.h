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

#ifndef CYBER_TRANSPORT_TCP_TCP_BUFFER_CONF_H_
#define CYBER_TRANSPORT_TCP_TCP_BUFFER_CONF_H_

#include <string>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/global_data.h"
#include "cyber/proto/role_attributes.pb.h"
#include "cyber/proto/tcp_buffer_conf.pb.h"
#include "cyber/transport/tcp/tcp_socket.h"

namespace century {
namespace cyber {
namespace transport {
namespace tcpbuf {

constexpr int kDefaultTcpBufSize = 256 * 1024;     // 256KB
constexpr int kMaxTcpBufSize = 16 * 1024 * 1024;   // 16MB
constexpr int kBufScale = 4;

inline bool LoadTcpBufferConf(proto::TcpBufferConf* conf) {
  if (nullptr == conf) {
    return false;
  }
  std::string conf_path("conf/tcp_buffer.conf");
  auto cfg_file = common::GetAbsolutePath(common::WorkRoot(), conf_path);
  if (!common::PathExists(cfg_file) ||
      !common::GetProtoFromFile(cfg_file, conf)) {
    return false;
  }
  return true;
}

inline int ResolveBufSize(const proto::TcpBufferConf& conf,
                          const std::string& channel_name) {
  int min_buf = conf.has_default_buf()
                    ? static_cast<int>(conf.default_buf())
                    : kDefaultTcpBufSize;
  int max_buf = conf.has_max_buf() ? static_cast<int>(conf.max_buf())
                                   : kMaxTcpBufSize;
  if (min_buf <= 0) {
    min_buf = kDefaultTcpBufSize;
  }
  if (max_buf <= 0) {
    max_buf = kMaxTcpBufSize;
  }

  uint64_t msg_size = 0;
  for (const auto& ch : conf.channel_conf()) {
    if (ch.channel_name() == channel_name) {
      if (ch.has_max_msg_size()) {
        msg_size = ch.max_msg_size();
      }
      break;
    }
  }
  if (0 == msg_size) {
    return min_buf;
  }
  uint64_t scaled = msg_size * static_cast<uint64_t>(kBufScale);
  if (scaled > static_cast<uint64_t>(max_buf)) {
    return max_buf;
  }
  if (scaled < static_cast<uint64_t>(min_buf)) {
    return min_buf;
  }
  return static_cast<int>(scaled);
}

inline int ComputeBufSize(const RoleAttributes& attr) {
  proto::TcpBufferConf conf;
  if (LoadTcpBufferConf(&conf)) {
    return ResolveBufSize(conf, attr.channel_name());
  }
  return kDefaultTcpBufSize;
}

inline TcpSocketOptions MakeTcpReceiverOptions(const RoleAttributes& attr) {
  TcpSocketOptions options;
  options.nodelay = true;
  options.quickack = true;
  options.rcvbuf = ComputeBufSize(attr);
  return options;
}

inline TcpSocketOptions MakeTcpTransmitterOptions(const RoleAttributes& attr) {
  TcpSocketOptions options;
  options.keepalive = true;
  options.nodelay = true;
  options.user_timeout_ms = 2000;
  int buf = ComputeBufSize(attr);
  options.sndbuf = buf;
  options.rcvbuf = buf;
  return options;
}

}  // namespace tcpbuf
}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_TCP_TCP_BUFFER_CONF_H_
