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

#include "cyber/transport/tcp/tcp_socket.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <limits>

#include "cyber/common/log.h"

namespace century {
namespace cyber {
namespace transport {

bool TcpSocket::SetNonBlocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    AERROR << "tcp get flags failed.";
    return false;
  }
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    AERROR << "tcp set nonblocking failed.";
    return false;
  }
  return true;
}

void TcpSocket::Close(int fd) {
  if (fd < 0) {
    return;
  }
  shutdown(fd, SHUT_RDWR);
  close(fd);
}

void TcpSocket::Tune(int fd, const TcpSocketOptions& options) {
  if (options.sndbuf > 0) {
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &options.sndbuf,
                     sizeof(options.sndbuf));
  }
  if (options.rcvbuf > 0) {
    (void)setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &options.rcvbuf,
                     sizeof(options.rcvbuf));
  }
  if (options.keepalive) {
    int flag = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));
  }
  if (options.nodelay) {
    int flag = 1;
    (void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
  }
#ifdef TCP_QUICKACK
  if (options.quickack) {
    int flag = 1;
    (void)setsockopt(fd, IPPROTO_TCP, TCP_QUICKACK, &flag, sizeof(flag));
  }
#endif
#ifdef TCP_USER_TIMEOUT
  if (options.user_timeout_ms > 0) {
    int timeout = options.user_timeout_ms;
    (void)setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &timeout,
                     sizeof(timeout));
  }
#endif
}

bool TcpSocket::SendVector(int fd, const iovec* vec, int vec_count,
                           ssize_t* sent) {
  if (nullptr == sent) {
    return false;
  }
  *sent = 0;
  if (nullptr == vec || vec_count <= 0) {
    return true;
  }
  int flags = 0;
#ifdef MSG_NOSIGNAL
  flags = MSG_NOSIGNAL;
#endif
  while (true) {
    msghdr msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.msg_iov = const_cast<iovec*>(vec);
    msg.msg_iovlen = static_cast<size_t>(vec_count);
    ssize_t n = sendmsg(fd, &msg, flags);
    if (n < 0) {
      if (EINTR == errno) {
        continue;
      }
      if (EAGAIN == errno || EWOULDBLOCK == errno) {
        return true;
      }
      return false;
    }
    if (0 == n) {
      return false;
    }
    *sent = n;
    return true;
  }
}

bool TcpSocket::Read(int fd, char* buf, std::size_t len,
                     std::size_t* read_total, std::size_t* budget) {
  if (nullptr == read_total || nullptr == buf) {
    return false;
  }
  std::size_t local_budget = std::numeric_limits<std::size_t>::max();
  if (nullptr == budget) {
    budget = &local_budget;
  }
  while (*read_total < len && *budget > 0) {
    std::size_t to_read = len - *read_total;
    if (to_read > *budget) {
      to_read = *budget;
    }
    ssize_t n = recv(fd, buf + *read_total, to_read, 0);
    if (0 == n) {
      return false;
    }
    if (n < 0) {
      if (EINTR == errno) {
        continue;
      }
      if (EAGAIN == errno || EWOULDBLOCK == errno) {
        return true;
      }
      return false;
    }
    *read_total += static_cast<std::size_t>(n);
    *budget -= static_cast<std::size_t>(n);
  }
  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace century
