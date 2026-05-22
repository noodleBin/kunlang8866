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

#ifndef CYBER_TRANSPORT_RECEIVER_TCP_RECEIVER_H_
#define CYBER_TRANSPORT_RECEIVER_TCP_RECEIVER_H_

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/transport/common/proxy_manager.h"
#include "cyber/transport/receiver/shm_receiver.h"
#include "cyber/transport/tcp/tcp_buffer_conf.h"
#include "cyber/transport/tcp/tcp_socket.h"
#include "cyber/transport/transmitter/shm_transmitter.h"
#include "cyber/transport/receiver/receiver.h"
#include "cyber/service_discovery/topology_manager.h"
#include "cyber/proto/topology_change.pb.h"

namespace century {
namespace cyber {
namespace transport {

template <typename M>
class TcpReceiver : public Receiver<M> {
 public:
  TcpReceiver(const RoleAttributes& attr,
              const typename Receiver<M>::MessageListener& msg_listener);
  virtual ~TcpReceiver();

  void Enable() override;
  void Disable() override;
  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

 private:
  static constexpr uint32_t kMaxTcpMessageSize = 4 * 1024 * 1024;
  static const std::size_t kHeaderSize;
  static constexpr int kEpollWaitTimeoutMs = 100;
  static constexpr std::size_t kReadQuotaBytes = 4 * 1024 * 1024;

  struct Connection {
    int fd = -1;
    bool registered = false;
    std::size_t header_offset = 0;
    std::string header_buf;
    uint32_t payload_size = 0;
    std::size_t payload_offset = 0;
    std::string payload;
    bool drop_payload = false;
    WritableBlock shm_block;
    bool use_shm_block = false;
    MessageInfo msg_info;
    bool has_msg_info = false;
  };
  using ConnectionPtr = std::shared_ptr<Connection>;

  bool SetupSocket();
  bool SetupEpoll();
  bool TryPromoteToProxyLocked();
  bool ActivateProxyLocked();
  void ReleaseProxyLock();
  void DisableShmReceiverLocked();
  void EnableShmSelfLocked();
  void OnProxyAcquiredFromWatch();
  void ReannounceSelf();
  void IoLoop();
  void WakeupIo();
  void HandleAccept();
  bool HandleRead(const ConnectionPtr& conn);
  bool ParseHeader(Connection* conn);
  void ResetForNextMessage(Connection* conn);
  void CloseConnectionLocked(int fd);
  void CloseAllConnectionsLocked();

  std::atomic<bool> running_ = {false};
  std::atomic<bool> is_proxy_{false};
  int listen_fd_ = -1;
  int epoll_fd_ = -1;
  int event_fd_ = -1;

  std::thread io_thread_;
  std::mutex mutex_;
  std::mutex proxy_mutex_;
  std::unordered_map<int, ConnectionPtr> connections_;
  std::shared_ptr<ShmTransmitter<M>> shm_transmitter_;
  std::shared_ptr<ShmReceiver<M>> shm_receiver_;
  std::unordered_map<uint64_t, RoleAttributes> shm_opposites_;
};

template <typename M>
const std::size_t TcpReceiver<M>::kHeaderSize =
    sizeof(uint32_t) + MessageInfo::kSize;

template <typename M>
TcpReceiver<M>::TcpReceiver(
    const RoleAttributes& attr,
    const typename Receiver<M>::MessageListener& msg_listener)
    : Receiver<M>(attr, msg_listener) {}

template <typename M>
TcpReceiver<M>::~TcpReceiver() {
  Disable();
}

template <typename M>
void TcpReceiver<M>::Enable() {
  if (this->enabled_) {
    return;
  }
  bool proxy_ready = false;
  {
    std::lock_guard<std::mutex> guard(proxy_mutex_);
    proxy_ready = TryPromoteToProxyLocked();
    if (!proxy_ready) {
      shm_receiver_ = std::make_shared<ShmReceiver<M>>(this->attr_,
                                                       this->msg_listener_);
    }
  }
  if (!proxy_ready) {
    ProxyManager::Instance()->StartWatch(
        this->attr_.channel_id(), "tcp",
        [this]() { OnProxyAcquiredFromWatch(); });
  }
  this->enabled_ = true;
}

template <typename M>
void TcpReceiver<M>::Disable() {
  if (!this->enabled_) {
    return;
  }

  ProxyManager::Instance()->StopWatch(this->attr_.channel_id(), "tcp");

  running_.store(false);
  WakeupIo();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  {
    std::lock_guard<std::mutex> guard(proxy_mutex_);
    if (shm_transmitter_) {
      shm_transmitter_->Disable();
      shm_transmitter_.reset();
    }
    if (shm_receiver_) {
      DisableShmReceiverLocked();
      shm_receiver_.reset();
    }
  }

  if (event_fd_ >= 0) {
    close(event_fd_);
    event_fd_ = -1;
  }
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
    epoll_fd_ = -1;
  }

  ReleaseProxyLock();
  this->enabled_ = false;
}

template <typename M>
void TcpReceiver<M>::Enable(const RoleAttributes& opposite_attr) {
  if (!this->enabled_) {
    Enable();
  }
  {
    std::lock_guard<std::mutex> guard(proxy_mutex_);
    if (!is_proxy_.load()) {
      TryPromoteToProxyLocked();
    }
    if (!is_proxy_.load()) {
      if (shm_receiver_) {
        shm_receiver_->Enable(opposite_attr);
        shm_opposites_[opposite_attr.id()] = opposite_attr;
      }
      return;
    }
  }
  (void)opposite_attr;
}

template <typename M>
void TcpReceiver<M>::Disable(const RoleAttributes& opposite_attr) {
  {
    std::lock_guard<std::mutex> guard(proxy_mutex_);
    if (!is_proxy_.load()) {
      if (shm_receiver_) {
        shm_receiver_->Disable(opposite_attr);
        shm_opposites_.erase(opposite_attr.id());
      }
      return;
    }
  }
  (void)opposite_attr;
}

template <typename M>
bool TcpReceiver<M>::TryPromoteToProxyLocked() {
  if (is_proxy_.load()) {
    return true;
  }
  if (!ProxyManager::Instance()->TryAcquire(this->attr_.channel_id(), "tcp")) {
    return false;
  }
  if (!ActivateProxyLocked()) {
    ProxyManager::Instance()->Release(this->attr_.channel_id(), "tcp");
    return false;
  }
  is_proxy_.store(true);
  return true;
}

template <typename M>
bool TcpReceiver<M>::ActivateProxyLocked() {
  if (!SetupSocket()) {
    return false;
  }
  if (!SetupEpoll()) {
    if (listen_fd_ >= 0) {
      TcpSocket::Close(listen_fd_);
      listen_fd_ = -1;
    }
    return false;
  }
  EnableShmSelfLocked();
  if (!shm_transmitter_) {
    shm_transmitter_ = std::make_shared<ShmTransmitter<M>>(this->attr_);
  }
  shm_transmitter_->Enable();
  if (!running_.load()) {
    running_.store(true);
    io_thread_ = std::thread(&TcpReceiver<M>::IoLoop, this);
  }
  ReannounceSelf();
  return true;
}

template <typename M>
void TcpReceiver<M>::ReleaseProxyLock() {
  ProxyManager::Instance()->Release(this->attr_.channel_id(), "tcp");
  is_proxy_.store(false);
}

template <typename M>
void TcpReceiver<M>::DisableShmReceiverLocked() {
  if (!shm_receiver_) {
    return;
  }
  for (const auto& item : shm_opposites_) {
    shm_receiver_->Disable(item.second);
  }
  shm_opposites_.clear();
  shm_receiver_->Disable();
}

template <typename M>
void TcpReceiver<M>::EnableShmSelfLocked() {
  if (!shm_receiver_) {
    shm_receiver_ = std::make_shared<ShmReceiver<M>>(this->attr_,
                                                     this->msg_listener_);
  } else {
    DisableShmReceiverLocked();
  }
  shm_receiver_->Enable();
}

template <typename M>
void TcpReceiver<M>::OnProxyAcquiredFromWatch() {
  std::lock_guard<std::mutex> guard(proxy_mutex_);
  if (is_proxy_.load()) {
    return;
  }
  if (!ActivateProxyLocked()) {
    ProxyManager::Instance()->Release(this->attr_.channel_id(), "tcp");
    return;
  }
  is_proxy_.store(true);
}

template <typename M>
void TcpReceiver<M>::ReannounceSelf() {
  auto channel_manager =
      service_discovery::TopologyManager::Instance()->channel_manager();
  if (!channel_manager) {
    return;
  }
  channel_manager->Leave(this->attr_, proto::RoleType::ROLE_READER);
  channel_manager->Join(this->attr_, proto::RoleType::ROLE_READER, true);
}

template <typename M>
bool TcpReceiver<M>::SetupSocket() {
  if (listen_fd_ >= 0) {
    return true;
  }

  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    AERROR << "tcp socket create failed.";
    return false;
  }

  TcpSocketOptions options =
      tcpbuf::MakeTcpReceiverOptions(this->attr_);
  TcpSocket::Tune(fd, options);

  int reuse = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
    AERROR << "tcp setsockopt SO_REUSEADDR failed.";
    TcpSocket::Close(fd);
    return false;
  }

  sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  uint16_t port = 0;
  if (this->attr_.has_socket_addr() && this->attr_.socket_addr().has_port()) {
    port = static_cast<uint16_t>(this->attr_.socket_addr().port());
  }
  server_addr.sin_port = htons(port);

  if (bind(fd, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) <
      0) {
    AERROR << "tcp bind failed.";
    TcpSocket::Close(fd);
    return false;
  }

  if (listen(fd, SOMAXCONN) < 0) {
    AERROR << "tcp listen failed.";
    TcpSocket::Close(fd);
    return false;
  }

  if (!TcpSocket::SetNonBlocking(fd)) {
    TcpSocket::Close(fd);
    return false;
  }

  sockaddr_in real_addr;
  socklen_t addr_len = sizeof(real_addr);
  if (getsockname(fd, reinterpret_cast<sockaddr*>(&real_addr), &addr_len) != 0) {
    AERROR << "tcp getsockname failed.";
    TcpSocket::Close(fd);
    return false;
  }

  std::string ip = this->attr_.host_ip();
  if (ip.empty()) {
    ip = common::GlobalData::Instance()->HostIp();
  }
  auto* socket_addr = this->attr_.mutable_socket_addr();
  socket_addr->set_ip(ip);
  socket_addr->set_port(ntohs(real_addr.sin_port));

  listen_fd_ = fd;
  return true;
}

template <typename M>
bool TcpReceiver<M>::SetupEpoll() {
  if (epoll_fd_ >= 0) {
    return true;
  }
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ < 0) {
    AERROR << "tcp epoll create failed.";
    return false;
  }

  event_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (event_fd_ < 0) {
    AERROR << "tcp eventfd create failed.";
    close(epoll_fd_);
    epoll_fd_ = -1;
    return false;
  }

  epoll_event ev;
  ev.events = EPOLLIN;
  ev.data.fd = event_fd_;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_, &ev) != 0) {
    AERROR << "tcp eventfd add epoll failed.";
    close(event_fd_);
    event_fd_ = -1;
    close(epoll_fd_);
    epoll_fd_ = -1;
    return false;
  }

  ev.events = EPOLLIN | EPOLLERR | EPOLLHUP | EPOLLRDHUP;
  ev.data.fd = listen_fd_;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, listen_fd_, &ev) != 0) {
    AERROR << "tcp listen fd add epoll failed.";
    close(event_fd_);
    event_fd_ = -1;
    close(epoll_fd_);
    epoll_fd_ = -1;
    return false;
  }

  return true;
}

template <typename M>
void TcpReceiver<M>::WakeupIo() {
  if (event_fd_ < 0) {
    return;
  }
  uint64_t value = 1;
  ssize_t ret = write(event_fd_, &value, sizeof(value));
  if (ret < 0 && errno != EAGAIN) {
    ADEBUG << "tcp eventfd write failed.";
  }
}

template <typename M>
void TcpReceiver<M>::IoLoop() {
  constexpr int kMaxEvents = 64;
  epoll_event events[kMaxEvents];

  while (running_.load()) {
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
      int fd = events[i].data.fd;
      if (fd == event_fd_) {
        uint64_t value = 0;
        while (read(event_fd_, &value, sizeof(value)) > 0) {
        }
        continue;
      }

      if (fd == listen_fd_) {
        HandleAccept();
        continue;
      }

      uint32_t ev = events[i].events;
      if (ev & (EPOLLERR | EPOLLHUP | EPOLLRDHUP)) {
        std::lock_guard<std::mutex> lock(mutex_);
        CloseConnectionLocked(fd);
        continue;
      }

      if (ev & EPOLLIN) {
        ConnectionPtr conn;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          auto conn_iter = connections_.find(fd);
          if (conn_iter == connections_.end()) {
            continue;
          }
          conn = conn_iter->second;
        }
        if (conn && !HandleRead(conn)) {
          std::lock_guard<std::mutex> lock(mutex_);
          CloseConnectionLocked(fd);
        }
      }
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  CloseAllConnectionsLocked();
  if (listen_fd_ >= 0) {
    close(listen_fd_);
    listen_fd_ = -1;
  }
}

template <typename M>
void TcpReceiver<M>::HandleAccept() {
  while (true) {
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd =
        accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_addr),
               &client_len);
    if (client_fd < 0) {
      if (EAGAIN == errno || EWOULDBLOCK == errno) {
        break;
      }
      continue;
    }

    TcpSocketOptions options =
        tcpbuf::MakeTcpReceiverOptions(this->attr_);
    TcpSocket::Tune(client_fd, options);

    if (!TcpSocket::SetNonBlocking(client_fd)) {
      TcpSocket::Close(client_fd);
      continue;
    }

    epoll_event ev;
    ev.events = EPOLLIN | EPOLLERR | EPOLLHUP | EPOLLRDHUP;
    ev.data.fd = client_fd;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, client_fd, &ev) != 0) {
      TcpSocket::Close(client_fd);
      continue;
    }

    ConnectionPtr conn = std::make_shared<Connection>();
    conn->fd = client_fd;
    conn->registered = true;
    conn->header_buf.resize(kHeaderSize);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      connections_[client_fd] = std::move(conn);
    }
  }
}

template <typename M>
bool TcpReceiver<M>::HandleRead(const ConnectionPtr& conn) {
  if (!conn) {
    return false;
  }

  const int fd = conn->fd;
  bool alive = true;
  std::size_t budget = kReadQuotaBytes;
  while (alive && budget > 0) {
    if (conn->header_offset < kHeaderSize) {
      if (!TcpSocket::Read(fd, &conn->header_buf[0], kHeaderSize,
                           &conn->header_offset, &budget)) {
        alive = false;
        break;
      }
      if (conn->header_offset < kHeaderSize) {
        break;
      }
      if (!ParseHeader(conn.get())) {
        alive = false;
        break;
      }
    }

    if (conn->payload_offset < conn->payload_size) {
      if (conn->drop_payload) {
        constexpr std::size_t kDiscardChunk = 64 * 1024;
        char discard_buf[kDiscardChunk];
        while (conn->payload_offset < conn->payload_size && budget > 0) {
          std::size_t to_read = conn->payload_size - conn->payload_offset;
          if (to_read > kDiscardChunk) {
            to_read = kDiscardChunk;
          }
          if (to_read > budget) {
            to_read = budget;
          }
          ssize_t n = recv(fd, discard_buf, to_read, 0);
          if (0 == n) {
            alive = false;
            break;
          }
          if (n < 0) {
            if (EINTR == errno) {
              continue;
            }
            if (EAGAIN == errno || EWOULDBLOCK == errno) {
              break;
            }
            alive = false;
            break;
          }
          conn->payload_offset += static_cast<std::size_t>(n);
          budget -= static_cast<std::size_t>(n);
        }
        if (!alive || conn->payload_offset < conn->payload_size) {
          break;
        }
      } else {
        char* payload_buf = nullptr;
        if (conn->use_shm_block) {
          payload_buf = reinterpret_cast<char*>(conn->shm_block.buf);
        } else {
          payload_buf = &conn->payload[0];
        }
        if (!TcpSocket::Read(fd, payload_buf, conn->payload_size,
                             &conn->payload_offset, &budget)) {
          alive = false;
          break;
        }
        if (conn->payload_offset < conn->payload_size) {
          break;
        }
      }
    }

    if (conn->payload_offset >= conn->payload_size && conn->has_msg_info) {
      if (conn->drop_payload) {
        ResetForNextMessage(conn.get());
        continue;
      }
      if (conn->use_shm_block) {
        if (shm_transmitter_) {
          shm_transmitter_->CommitWritableBlock(conn->shm_block,
                                                conn->payload_size,
                                                conn->msg_info);
        }
      } else if (shm_transmitter_) {
        shm_transmitter_->TransmitSerialized(conn->payload, conn->msg_info);
      } else {
        auto msg = std::make_shared<M>();
        if (message::ParseFromString(conn->payload, msg.get())) {
          this->OnNewMessage(msg, conn->msg_info);
        }
      }
      ResetForNextMessage(conn.get());
    }
  }
  return alive;
}

template <typename M>
bool TcpReceiver<M>::ParseHeader(Connection* conn) {
  if (nullptr == conn || conn->header_buf.size() < kHeaderSize) {
    return false;
  }
  uint32_t msg_size_net = 0;
  std::memcpy(&msg_size_net, conn->header_buf.data(), sizeof(msg_size_net));
  uint32_t msg_size = ntohl(msg_size_net);
  if (0 == msg_size || msg_size > kMaxTcpMessageSize) {
    AERROR << "tcp invalid message size: " << msg_size;
    return false;
  }

  MessageInfo msg_info;
  if (!msg_info.DeserializeFrom(
          conn->header_buf.data() + sizeof(msg_size_net), MessageInfo::kSize)) {
    AERROR << "tcp deserialize message info failed.";
    return false;
  }

  conn->payload_size = msg_size;
  conn->payload_offset = 0;
  conn->use_shm_block = false;
  conn->drop_payload = false;
  if (shm_transmitter_ &&
      shm_transmitter_->AcquireWritableBlock(msg_size, &conn->shm_block)) {
    conn->use_shm_block = true;
  } else if (shm_transmitter_) {
    conn->drop_payload = true;
  } else {
    conn->payload.resize(msg_size);
  }
  conn->msg_info = msg_info;
  conn->has_msg_info = true;
  return true;
}

template <typename M>
void TcpReceiver<M>::ResetForNextMessage(Connection* conn) {
  if (nullptr == conn) {
    return;
  }
  conn->header_offset = 0;
  conn->payload_size = 0;
  conn->payload_offset = 0;
  conn->use_shm_block = false;
  conn->drop_payload = false;
  conn->has_msg_info = false;
}

template <typename M>
void TcpReceiver<M>::CloseConnectionLocked(int fd) {
  auto iter = connections_.find(fd);
  if (iter == connections_.end()) {
    return;
  }
  ConnectionPtr conn = iter->second;
  if (conn && conn->use_shm_block && shm_transmitter_) {
    shm_transmitter_->ReleaseWritableBlock(conn->shm_block);
  }
  if (conn && conn->registered && epoll_fd_ >= 0) {
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
  }
  TcpSocket::Close(fd);
  if (conn) {
    conn->fd = -1;
    conn->registered = false;
  }
  connections_.erase(iter);
}

template <typename M>
void TcpReceiver<M>::CloseAllConnectionsLocked() {
  for (auto& item : connections_) {
    ConnectionPtr conn = item.second;
    if (conn && conn->use_shm_block && shm_transmitter_) {
      shm_transmitter_->ReleaseWritableBlock(conn->shm_block);
    }
    if (conn && conn->registered && epoll_fd_ >= 0) {
      epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, item.first, nullptr);
    }
    TcpSocket::Close(item.first);
    if (conn) {
      conn->fd = -1;
      conn->registered = false;
    }
  }
  connections_.clear();
}

}  // namespace transport
}  // namespace cyber
}  // namespace century

#endif  // CYBER_TRANSPORT_RECEIVER_TCP_RECEIVER_H_
