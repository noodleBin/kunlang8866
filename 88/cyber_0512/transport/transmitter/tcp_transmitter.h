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

#ifndef CYBER_TRANSPORT_TRANSMITTER_TCP_TRANSMITTER_H_
#define CYBER_TRANSPORT_TRANSMITTER_TCP_TRANSMITTER_H_

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/task/task.h"
#include "cyber/transport/tcp/tcp_buffer_conf.h"
#include "cyber/transport/tcp/tcp_socket.h"
#include "cyber/transport/transmitter/transmitter.h"

namespace century {
namespace cyber {
namespace transport {

template <typename M>
class TcpTransmitter : public Transmitter<M> {
 public:
  using MessagePtr = std::shared_ptr<M>;

  explicit TcpTransmitter(const RoleAttributes& attr);
  virtual ~TcpTransmitter();

  void Enable() override;
  void Disable() override;
  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

 private:
  struct SendItem {
    std::shared_ptr<std::string> header;
    std::shared_ptr<std::string> payload;
    std::size_t header_offset = 0;
    std::size_t payload_offset = 0;
  };

  struct Connection {
    std::mutex mutex;
    int fd = -1;
    bool connecting = false;
    bool registered = false;
    uint32_t generation = 0;
    uint32_t cur_events = 0;
    std::size_t queued_bytes = 0;
    std::deque<SendItem> queue;
  };
  using ConnectionPtr = std::shared_ptr<Connection>;

  static constexpr std::size_t kMaxQueueBytes = 8 * 1024 * 1024;
  static constexpr int kEpollWaitTimeoutMs = 100;
  static constexpr int kMinRetryMs = 100;
  static constexpr int kMaxRetryMs = 2000;

  struct RetryState {
    std::chrono::steady_clock::time_point next_retry;
    std::chrono::milliseconds backoff{0};
  };

  bool EnsureIoThread();
  void IoLoop();
  void WakeupIo();
  void TryConnectPending();
  bool ConnectNonBlocking(const RoleAttributes& opposite_attr, int* socket_fd,
                          bool* connecting);
  static uint64_t MakeToken(int fd, uint32_t generation);
  static int TokenFd(uint64_t token);
  static uint32_t TokenGeneration(uint64_t token);
  void UpdateConnectionEventsLocked(uint64_t, Connection* conn,
                                    bool want_write);
  void CloseConnection(const ConnectionPtr& conn);
  void CloseConnectionById(uint64_t id, const ConnectionPtr& conn);
  std::size_t RemainingBytes(const SendItem& item) const;
  bool FlushConnectionLocked(uint64_t id, Connection* conn);
  bool EnqueueMessageLocked(Connection* conn,
                            const std::shared_ptr<std::string>& header,
                            const std::shared_ptr<std::string>& payload,
                            bool* disconnect);
  std::string MakeEndpointKey(const RoleAttributes& opposite_attr) const;

  std::mutex mutex_;
  std::mutex io_mutex_;
  std::unordered_map<uint64_t, RoleAttributes> peers_;
  std::unordered_map<std::string, uint64_t> endpoint_to_id_;
  std::unordered_map<uint64_t, ConnectionPtr> connections_;
  std::unordered_map<int, uint64_t> fd_to_id_;
  std::unordered_map<uint64_t, RoleAttributes> pending_;
  std::unordered_map<uint64_t, RetryState> retry_states_;
  std::mutex epoll_mutex_;
  std::atomic<uint32_t> next_generation_{1};

  int epoll_fd_ = -1;
  int event_fd_ = -1;
  std::thread io_thread_;
  std::atomic<bool> running_ = {false};
  std::atomic<bool> wakeup_pending_{false};
};

template <typename M>
TcpTransmitter<M>::TcpTransmitter(const RoleAttributes& attr)
    : Transmitter<M>(attr) {}

template <typename M>
TcpTransmitter<M>::~TcpTransmitter() {
  Disable();
}

template <typename M>
void TcpTransmitter<M>::Enable() {
  this->enabled_ = true;
  EnsureIoThread();
}

template <typename M>
void TcpTransmitter<M>::Disable() {
  this->enabled_ = false;
  running_.store(false);
  WakeupIo();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }

  std::vector<ConnectionPtr> conns;
  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    conns.reserve(connections_.size());
    for (auto& item : connections_) {
      conns.push_back(item.second);
    }
    connections_.clear();
    endpoint_to_id_.clear();
    fd_to_id_.clear();
    pending_.clear();
    peers_.clear();
    retry_states_.clear();
  }
  for (const auto& conn : conns) {
    CloseConnection(conn);
  }

  {
    std::lock_guard<std::mutex> epoll_lock(epoll_mutex_);
    if (event_fd_ >= 0) {
      close(event_fd_);
      event_fd_ = -1;
    }
    if (epoll_fd_ >= 0) {
      close(epoll_fd_);
      epoll_fd_ = -1;
    }
  }
}

template <typename M>
void TcpTransmitter<M>::Enable(const RoleAttributes& opposite_attr) {
  if (!this->enabled_) {
    this->enabled_ = true;
  }
  if (!opposite_attr.has_socket_addr() ||
      !opposite_attr.socket_addr().has_port() ||
      0 == opposite_attr.socket_addr().port()) {
    return;
  }

  EnsureIoThread();

  const uint64_t key = opposite_attr.id();
  const std::string endpoint = MakeEndpointKey(opposite_attr);
  if (endpoint.empty()) {
    return;
  }
  uint64_t old_id = 0;
  ConnectionPtr old_conn;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto ep_iter = endpoint_to_id_.find(endpoint);
    if (ep_iter != endpoint_to_id_.end() && ep_iter->second != key) {
      old_id = ep_iter->second;
      auto old_iter = connections_.find(old_id);
      if (old_iter != connections_.end()) {
        old_conn = old_iter->second;
      }
    }
    endpoint_to_id_[endpoint] = key;
    peers_[key] = opposite_attr;
    auto conn_iter = connections_.find(key);
    if (conn_iter == connections_.end()) {
      connections_[key] = std::make_shared<Connection>();
      pending_[key] = opposite_attr;
    } else {
      auto conn = conn_iter->second;
      if (!conn) {
        connections_[key] = std::make_shared<Connection>();
        pending_[key] = opposite_attr;
      } else {
        std::lock_guard<std::mutex> conn_lock(conn->mutex);
        if (conn->fd < 0) {
          pending_[key] = opposite_attr;
        } else {
          return;
        }
      }
    }
  }

  if (old_id != 0 && old_id != key) {
    if (old_conn) {
      CloseConnectionById(old_id, old_conn);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.erase(old_id);
    peers_.erase(old_id);
    retry_states_.erase(old_id);
  }

  WakeupIo();
}

template <typename M>
void TcpTransmitter<M>::Disable(const RoleAttributes& opposite_attr) {
  const uint64_t key = opposite_attr.id();
  const std::string endpoint = MakeEndpointKey(opposite_attr);
  ConnectionPtr conn;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!endpoint.empty()) {
      auto ep_iter = endpoint_to_id_.find(endpoint);
      if (ep_iter != endpoint_to_id_.end() && ep_iter->second == key) {
        endpoint_to_id_.erase(ep_iter);
      }
    }
    auto iter = connections_.find(key);
    if (iter != connections_.end()) {
      conn = iter->second;
    }
  }
  if (conn) {
    CloseConnectionById(key, conn);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.erase(key);
    peers_.erase(key);
    retry_states_.erase(key);
  }
}

template <typename M>
bool TcpTransmitter<M>::Transmit(const MessagePtr& msg,
                                 const MessageInfo& msg_info) {
  const auto start_time = std::chrono::steady_clock::now();
  bool queued = false;
  std::size_t payload_size = 0;
  struct CostGuard {
    std::chrono::steady_clock::time_point start;
    const bool* queued;
    const std::size_t* payload_size;
    ~CostGuard() {
      const auto end_time = std::chrono::steady_clock::now();
      const auto cost_us = std::chrono::duration_cast<std::chrono::microseconds>(
                               end_time - start)
                               .count();
      AINFO << "tcp transmit cost(us): " << cost_us
            << " payload_bytes: " << *payload_size
            << " queued: " << (*queued ? 1 : 0);
    }
  } guard{start_time, &queued, &payload_size};

  if (!this->enabled_) {
    ADEBUG << "not enable.";
    return false;
  }

  EnsureIoThread();

  auto payload = std::make_shared<std::string>();
  RETURN_VAL_IF(!message::SerializeToString(*msg, payload.get()), false);
  payload_size = payload->size();

  MessageInfo msg_info_copy(msg_info);
  msg_info_copy.set_channel_id(this->attr_.channel_id());

  uint32_t msg_size = static_cast<uint32_t>(payload->size());
  uint32_t msg_size_net = htonl(msg_size);
  auto header = std::make_shared<std::string>();
  header->resize(sizeof(msg_size_net) + MessageInfo::kSize);
  char* header_buf = &(*header)[0];
  std::memcpy(header_buf, &msg_size_net, sizeof(msg_size_net));
  RETURN_VAL_IF(
      !msg_info_copy.SerializeTo(header_buf + sizeof(msg_size_net),
                                 MessageInfo::kSize),
      false);

  std::vector<std::pair<uint64_t, ConnectionPtr>> conns;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    conns.reserve(connections_.size());
    for (auto& item : connections_) {
      conns.emplace_back(item.first, item.second);
    }
  }

  std::vector<uint64_t> disconnect;
  for (auto& item : conns) {
    auto& conn = item.second;
    if (!conn) {
      continue;
    }
    bool disconnect_peer = false;
    bool want_write = false;
    {
      std::lock_guard<std::mutex> conn_lock(conn->mutex);
      if (EnqueueMessageLocked(conn.get(), header, payload,
                               &disconnect_peer)) {
        queued = true;
        want_write = (conn->fd >= 0 && !conn->connecting);
        if (want_write) {
          UpdateConnectionEventsLocked(item.first, conn.get(), true);
        }
      }
    }
    if (disconnect_peer) {
      disconnect.push_back(item.first);
    }
  }

  if (!disconnect.empty()) {
    for (const auto& id : disconnect) {
      ConnectionPtr conn;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto conn_iter = connections_.find(id);
        if (conn_iter == connections_.end()) {
          continue;
        }
        conn = conn_iter->second;
      }
      if (conn) {
        CloseConnectionById(id, conn);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      auto peer_iter = peers_.find(id);
      if (peer_iter != peers_.end()) {
        pending_[id] = peer_iter->second;
      }
    }
  }

  WakeupIo();
  return queued;
}

template <typename M>
bool TcpTransmitter<M>::EnsureIoThread() {
  if (running_.load()) {
    return true;
  }
  std::lock_guard<std::mutex> guard(io_mutex_);
  if (running_.load()) {
    return true;
  }
  if (epoll_fd_ < 0) {
    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
    if (epoll_fd_ < 0) {
      AERROR << "tcp epoll create failed.";
      return false;
    }
  }
  if (event_fd_ < 0) {
    event_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (event_fd_ < 0) {
      AERROR << "tcp eventfd create failed.";
      close(epoll_fd_);
      epoll_fd_ = -1;
      return false;
    }
    epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.u64 = MakeToken(event_fd_, 0);
    std::lock_guard<std::mutex> epoll_lock(epoll_mutex_);
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_, &ev) != 0) {
      AERROR << "tcp eventfd add epoll failed.";
      close(event_fd_);
      event_fd_ = -1;
      close(epoll_fd_);
      epoll_fd_ = -1;
      return false;
    }
  }
  running_.store(true);
  io_thread_ = std::thread(&TcpTransmitter<M>::IoLoop, this);
  return true;
}

template <typename M>
void TcpTransmitter<M>::WakeupIo() {
  if (event_fd_ < 0) {
    return;
  }
  if (wakeup_pending_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  uint64_t value = 1;
  ssize_t ret = write(event_fd_, &value, sizeof(value));
  if (ret < 0 && errno != EAGAIN) {
    ADEBUG << "tcp eventfd write failed.";
  }
}

template <typename M>
void TcpTransmitter<M>::IoLoop() {
  constexpr int kMaxEvents = 64;
  epoll_event events[kMaxEvents];

  while (running_.load()) {
    TryConnectPending();
    int ready = epoll_wait(epoll_fd_, events, kMaxEvents,
                           kEpollWaitTimeoutMs);
    if (!running_.load()) {
      break;
    }
    if (ready < 0) {
      if (EINTR == errno) {
        continue;
      }
      continue;
    }

    for (int i = 0; i < ready; ++i) {
      uint64_t token = events[i].data.u64;
      int fd = TokenFd(token);
      uint32_t generation = TokenGeneration(token);
      if (fd == event_fd_) {
        uint64_t value = 0;
        while (read(event_fd_, &value, sizeof(value)) > 0) {
        }
        wakeup_pending_.store(false, std::memory_order_release);
        continue;
      }

      uint64_t id = 0;
      ConnectionPtr conn;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto id_iter = fd_to_id_.find(fd);
        if (id_iter == fd_to_id_.end()) {
          continue;
        }
        id = id_iter->second;
        auto conn_iter = connections_.find(id);
        if (conn_iter == connections_.end()) {
          continue;
        }
        conn = conn_iter->second;
      }
      if (!conn) {
        continue;
      }

      bool need_close = false;
      bool want_write = false;
      {
        std::lock_guard<std::mutex> conn_lock(conn->mutex);
        if (conn->fd != fd || conn->generation != generation) {
          continue;
        }
        uint32_t ev = events[i].events;
        if (ev & (EPOLLERR | EPOLLHUP | EPOLLRDHUP)) {
          need_close = true;
        } else {
          if (conn->connecting) {
            int err = 0;
            socklen_t len = sizeof(err);
            if (getsockopt(conn->fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 ||
                err != 0) {
              need_close = true;
            } else {
              conn->connecting = false;
            }
          }
          if (!need_close && (ev & EPOLLOUT) && !conn->queue.empty()) {
            if (!FlushConnectionLocked(id, conn.get())) {
              need_close = true;
            }
          }
          if (!need_close) {
            want_write = conn->connecting || !conn->queue.empty();
            UpdateConnectionEventsLocked(id, conn.get(), want_write);
          }
        }
      }

      if (need_close) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          auto fd_iter = fd_to_id_.find(fd);
          if (fd_iter != fd_to_id_.end() && fd_iter->second == id) {
            fd_to_id_.erase(fd_iter);
          }
        }
        CloseConnection(conn);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          auto peer_iter = peers_.find(id);
          if (peer_iter != peers_.end()) {
            pending_[id] = peer_iter->second;
          }
        }
      }
    }
  }
}

template <typename M>
void TcpTransmitter<M>::TryConnectPending() {
  const auto now = std::chrono::steady_clock::now();
  std::vector<std::pair<uint64_t, RoleAttributes>> pending;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending.reserve(pending_.size());
    for (const auto& item : pending_) {
      auto state_iter = retry_states_.find(item.first);
      if (state_iter != retry_states_.end() &&
          now < state_iter->second.next_retry) {
        continue;
      }
      pending.emplace_back(item.first, item.second);
    }
  }

  for (const auto& item : pending) {
    int socket_fd = -1;
    bool connecting = false;
    if (!ConnectNonBlocking(item.second, &socket_fd, &connecting)) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto& state = retry_states_[item.first];
      if (0 == state.backoff.count()) {
        state.backoff = std::chrono::milliseconds(kMinRetryMs);
      } else {
        auto next_ms =
            std::min(state.backoff.count() * 2,
                     static_cast<int64_t>(kMaxRetryMs));
        state.backoff = std::chrono::milliseconds(next_ms);
      }
      state.next_retry = now + state.backoff;
      continue;
    }

    ConnectionPtr conn;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto pending_iter = pending_.find(item.first);
      if (pending_iter == pending_.end()) {
        conn.reset();
      } else {
        auto conn_iter = connections_.find(item.first);
        if (conn_iter == connections_.end() || !conn_iter->second) {
          conn = std::make_shared<Connection>();
          connections_[item.first] = conn;
        } else {
          conn = conn_iter->second;
        }
        pending_.erase(pending_iter);
        retry_states_.erase(item.first);
      }
    }

    if (!conn) {
      TcpSocket::Close(socket_fd);
      continue;
    }

    bool assigned = false;
    {
      std::lock_guard<std::mutex> conn_lock(conn->mutex);
      if (conn->fd < 0) {
        conn->fd = socket_fd;
        conn->connecting = connecting;
        conn->registered = false;
        conn->generation =
            next_generation_.fetch_add(1, std::memory_order_relaxed);
        conn->cur_events = 0;
        assigned = true;
      }
    }

    if (!assigned) {
      TcpSocket::Close(socket_fd);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      fd_to_id_[socket_fd] = item.first;
    }
    {
      std::lock_guard<std::mutex> conn_lock(conn->mutex);
      UpdateConnectionEventsLocked(
          item.first, conn.get(), conn->connecting || !conn->queue.empty());
    }
  }
}

template <typename M>
bool TcpTransmitter<M>::ConnectNonBlocking(const RoleAttributes& opposite_attr,
                                           int* socket_fd,
                                           bool* connecting) {
  if (nullptr == socket_fd || nullptr == connecting) {
    return false;
  }

  if (!opposite_attr.has_socket_addr()) {
    AWARN << "tcp receiver socket address not set.";
    return false;
  }

  const auto& addr = opposite_attr.socket_addr();
  if (!addr.has_port() || 0 == addr.port()) {
    AWARN << "tcp receiver port not set.";
    return false;
  }

  std::string ip = addr.ip();
  if (ip.empty()) {
    ip = opposite_attr.host_ip();
  }
  if (ip.empty()) {
    AWARN << "tcp receiver ip not set.";
    return false;
  }

  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    AERROR << "tcp socket create failed.";
    return false;
  }
  TcpSocketOptions options =
      tcpbuf::MakeTcpTransmitterOptions(this->attr_);
  TcpSocket::Tune(fd, options);
  if (!TcpSocket::SetNonBlocking(fd)) {
    TcpSocket::Close(fd);
    return false;
  }

  sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(static_cast<uint16_t>(addr.port()));
  if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
    AERROR << "tcp invalid receiver ip: " << ip;
    TcpSocket::Close(fd);
    return false;
  }

  int ret = connect(fd, reinterpret_cast<sockaddr*>(&server_addr),
                    sizeof(server_addr));
  if (0 == ret) {
    *socket_fd = fd;
    *connecting = false;
    return true;
  }
  if (ret < 0 && EINPROGRESS == errno) {
    *socket_fd = fd;
    *connecting = true;
    return true;
  }

  TcpSocket::Close(fd);
  return false;
}

template <typename M>
void TcpTransmitter<M>::UpdateConnectionEventsLocked(uint64_t,
                                                     Connection* conn,
                                                     bool want_write) {
  if (nullptr == conn || conn->fd < 0) {
    return;
  }
  std::lock_guard<std::mutex> epoll_lock(epoll_mutex_);
  if (epoll_fd_ < 0) {
    return;
  }
  uint32_t events = EPOLLERR | EPOLLHUP | EPOLLRDHUP | EPOLLIN;
  if (want_write) {
    events |= EPOLLOUT;
  }
  if (conn->registered && events == conn->cur_events) {
    return;
  }
  epoll_event ev;
  ev.data.u64 = MakeToken(conn->fd, conn->generation);
  ev.events = events;
  if (!conn->registered) {
    if (0 == epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, conn->fd, &ev)) {
      conn->registered = true;
      conn->cur_events = events;
    }
  } else {
    if (0 == epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, conn->fd, &ev)) {
      conn->cur_events = events;
    }
  }
}

template <typename M>
void TcpTransmitter<M>::CloseConnection(const ConnectionPtr& conn) {
  if (!conn) {
    return;
  }
  int fd = -1;
  bool registered = false;
  {
    std::lock_guard<std::mutex> conn_lock(conn->mutex);
    fd = conn->fd;
    registered = conn->registered;
    conn->fd = -1;
    conn->connecting = false;
    conn->registered = false;
    conn->generation = 0;
    conn->cur_events = 0;
    conn->queued_bytes = 0;
    conn->queue.clear();
  }
  if (fd >= 0) {
    {
      std::lock_guard<std::mutex> epoll_lock(epoll_mutex_);
      if (epoll_fd_ >= 0 && registered) {
        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
      }
    }
    TcpSocket::Close(fd);
  }
}

template <typename M>
void TcpTransmitter<M>::CloseConnectionById(uint64_t id,
                                            const ConnectionPtr& conn) {
  if (!conn) {
    return;
  }
  int fd = -1;
  bool registered = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = connections_.find(id);
    if (iter == connections_.end() || iter->second != conn) {
      return;
    }
    std::lock_guard<std::mutex> conn_lock(conn->mutex);
    fd = conn->fd;
    registered = conn->registered;
    conn->fd = -1;
    conn->connecting = false;
    conn->registered = false;
    conn->generation = 0;
    conn->cur_events = 0;
    conn->queued_bytes = 0;
    conn->queue.clear();
    connections_.erase(iter);
    if (fd >= 0) {
      auto fd_iter = fd_to_id_.find(fd);
      if (fd_iter != fd_to_id_.end() && fd_iter->second == id) {
        fd_to_id_.erase(fd_iter);
      }
    }
  }
  if (fd >= 0) {
    {
      std::lock_guard<std::mutex> epoll_lock(epoll_mutex_);
      if (epoll_fd_ >= 0 && registered) {
        epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
      }
    }
    TcpSocket::Close(fd);
  }
}

template <typename M>
uint64_t TcpTransmitter<M>::MakeToken(int fd, uint32_t generation) {
  return (static_cast<uint64_t>(generation) << 32) |
         static_cast<uint32_t>(fd);
}

template <typename M>
int TcpTransmitter<M>::TokenFd(uint64_t token) {
  return static_cast<int>(token & 0xFFFFFFFFu);
}

template <typename M>
uint32_t TcpTransmitter<M>::TokenGeneration(uint64_t token) {
  return static_cast<uint32_t>(token >> 32);
}

template <typename M>
std::size_t TcpTransmitter<M>::RemainingBytes(const SendItem& item) const {
  std::size_t header_remaining = 0;
  std::size_t payload_remaining = 0;
  if (item.header && item.header_offset < item.header->size()) {
    header_remaining = item.header->size() - item.header_offset;
  }
  if (item.payload && item.payload_offset < item.payload->size()) {
    payload_remaining = item.payload->size() - item.payload_offset;
  }
  return header_remaining + payload_remaining;
}

template <typename M>
bool TcpTransmitter<M>::FlushConnectionLocked(uint64_t id, Connection* conn) {
  if (nullptr == conn || conn->fd < 0) {
    return false;
  }
  while (!conn->queue.empty()) {
    SendItem& item = conn->queue.front();
    std::size_t header_remaining =
        item.header ? item.header->size() - item.header_offset : 0;
    std::size_t payload_remaining =
        item.payload ? item.payload->size() - item.payload_offset : 0;
    if (0 == header_remaining && 0 == payload_remaining) {
      conn->queue.pop_front();
      continue;
    }

    const std::size_t before_remaining = header_remaining + payload_remaining;
    iovec vec[2];
    int vec_count = 0;
    if (header_remaining > 0) {
      vec[vec_count].iov_base =
          const_cast<char*>(item.header->data() + item.header_offset);
      vec[vec_count].iov_len = header_remaining;
      ++vec_count;
    }
    if (payload_remaining > 0) {
      vec[vec_count].iov_base =
          const_cast<char*>(item.payload->data() + item.payload_offset);
      vec[vec_count].iov_len = payload_remaining;
      ++vec_count;
    }

    ssize_t sent = 0;
    if (!TcpSocket::SendVector(conn->fd, vec, vec_count, &sent)) {
      return false;
    }
    if (0 == sent) {
      break;
    }
    std::size_t sent_left = static_cast<std::size_t>(sent);
    if (header_remaining > 0) {
      std::size_t advance = std::min(header_remaining, sent_left);
      item.header_offset += advance;
      sent_left -= advance;
    }
    if (sent_left > 0 && payload_remaining > 0) {
      item.payload_offset += sent_left;
    }
    const std::size_t after_remaining = RemainingBytes(item);
    const std::size_t delta = before_remaining - after_remaining;
    if (delta > conn->queued_bytes) {
      conn->queued_bytes = 0;
    } else {
      conn->queued_bytes -= delta;
    }
    if (0 == RemainingBytes(item)) {
      conn->queue.pop_front();
    }
  }
  (void)id;
  return true;
}

template <typename M>
bool TcpTransmitter<M>::EnqueueMessageLocked(
    Connection* conn, const std::shared_ptr<std::string>& header,
    const std::shared_ptr<std::string>& payload, bool* disconnect) {
  if (disconnect != nullptr) {
    *disconnect = false;
  }
  if (nullptr == conn) {
    return false;
  }
  if (nullptr == header || nullptr == payload) {
    return false;
  }
  const std::size_t message_size = header->size() + payload->size();
  if (message_size > kMaxQueueBytes) {
    return false;
  }

  if (conn->queued_bytes + message_size > kMaxQueueBytes) {
    if (!conn->queue.empty()) {
      SendItem& front = conn->queue.front();
      const bool in_flight =
          front.header_offset > 0 || front.payload_offset > 0;
      if (in_flight) {
        std::size_t front_remaining = RemainingBytes(front);
        conn->queue.resize(1);
        conn->queued_bytes = front_remaining;
        if (conn->queued_bytes + message_size > kMaxQueueBytes) {
          return false;
        }
      } else {
        conn->queue.clear();
        conn->queued_bytes = 0;
      }
    }
  }

  conn->queue.push_back({header, payload, 0, 0});
  conn->queued_bytes += message_size;

  return true;
}

template <typename M>
std::string TcpTransmitter<M>::MakeEndpointKey(
    const RoleAttributes& opposite_attr) const {
  const uint64_t channel_id = opposite_attr.channel_id();
  std::string ip = opposite_attr.host_ip();
  if (ip.empty() && opposite_attr.has_socket_addr()) {
    ip = opposite_attr.socket_addr().ip();
  }
  if (ip.empty()) {
    return std::string();
  }
  return std::to_string(channel_id) + "@" + ip;
}

}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_TRANSMITTER_TCP_TRANSMITTER_H_
