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

#ifndef CYBER_TRANSPORT_TCP_TCP_SOCKET_H_
#define CYBER_TRANSPORT_TCP_TCP_SOCKET_H_

#include <sys/uio.h>

#include <cstddef>
#include <cstdint>

namespace century {
namespace cyber {
namespace transport {

struct TcpSocketOptions {
  int sndbuf = 0;
  int rcvbuf = 0;
  bool keepalive = false;
  bool nodelay = false;
  bool quickack = false;
  int user_timeout_ms = 0;
};

class TcpSocket {
 public:
  TcpSocket() = delete;

  static bool SetNonBlocking(int fd);
  static void Close(int fd);
  static void Tune(int fd, const TcpSocketOptions& options);
  static bool SendVector(int fd, const iovec* vec, int vec_count,
                         ssize_t* sent);
  static bool Read(int fd, char* buf, std::size_t len,
                   std::size_t* read_total, std::size_t* budget);
};

}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_TCP_TCP_SOCKET_H_
