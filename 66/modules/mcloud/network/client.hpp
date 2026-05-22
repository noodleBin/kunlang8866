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

#pragma once

#include <algorithm>
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include "cyber/cyber.h"
#include "modules/mcloud/common/cloud_gflags.h"
#include "modules/mcloud/network/data_handler.h"

namespace century {
namespace mcloud {

class TcpClient {
 public:
  TcpClient()
      : server_ip_("0.0.0.0"),
        server_port_(0),
        sockfd_(-1),
        stop_(true),
        is_connected_(false) {}

  TcpClient(const std::string& ip, int port)
      : server_ip_(ip),
        server_port_(port),
        sockfd_(-1),
        stop_(true),
        is_connected_(false) {}

  ~TcpClient() { StopReceiving(); }

  bool SetAttribute(const std::string& ip, int port) {
    server_ip_ = ip;
    server_port_ = port;
    return true;
  }

  bool ConnectToServer() {
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
      sockfd_ = -1;
      return false;
    }

    if (!ConfigureSocketTimeouts(sockfd_)) {
      AERROR << "Failed to configure socket timeouts.";
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    sockaddr_in serverAddr;
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(server_port_);
    if (1 != inet_pton(AF_INET, server_ip_.c_str(), &serverAddr.sin_addr)) {
      AERROR << "Invalid server ip: " << server_ip_;
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    const int original_flags = fcntl(sockfd_, F_GETFL, 0);
    if (original_flags < 0) {
      AERROR << "Failed to get socket flags.";
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    if (fcntl(sockfd_, F_SETFL, original_flags | O_NONBLOCK) < 0) {
      AERROR << "Failed to set socket non-blocking mode.";
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    const int connect_result =
        connect(sockfd_, reinterpret_cast<struct sockaddr*>(&serverAddr),
                sizeof(serverAddr));
    if (connect_result < 0 && EINPROGRESS != errno) {
      AERROR << "Connect failed immediately, errno=" << errno;
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    if (connect_result < 0) {
      fd_set write_fds;
      FD_ZERO(&write_fds);
      FD_SET(sockfd_, &write_fds);

      timeval timeout;
      timeout.tv_sec = FLAGS_cloud_connect_timeout_ms / 1000;
      timeout.tv_usec = (FLAGS_cloud_connect_timeout_ms % 1000) * 1000;

      const int ready = select(sockfd_ + 1, nullptr, &write_fds, nullptr,
                               &timeout);
      if (ready <= 0) {
        if (0 == ready) {
          AERROR << "Connect timeout after "
                 << FLAGS_cloud_connect_timeout_ms << " ms.";
        } else {
          AERROR << "Select failed while connecting, errno=" << errno;
        }
        close(sockfd_);
        sockfd_ = -1;
        return false;
      }

      int socket_error = 0;
      socklen_t error_len = sizeof(socket_error);
      if (getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &socket_error,
                     &error_len) < 0 ||
          0 != socket_error) {
        AERROR << "Connect failed, socket error="
               << (0 != socket_error ? socket_error : errno);
        close(sockfd_);
        sockfd_ = -1;
        return false;
      }
    }

    if (fcntl(sockfd_, F_SETFL, original_flags) < 0) {
      AERROR << "Failed to restore socket flags.";
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    is_connected_ = true;
    AINFO << "Connected to server.";
    return true;
  }

  void Disconnect() {
    if (sockfd_ != -1) {
      close(sockfd_);
      sockfd_ = -1;
    }
    is_connected_ = false;
  }

  void StartReceiving(IDataHandler& dataHandler) {
    stop_ = false;
    std::thread([this, &dataHandler]() {
      while (!stop_) {
        if (!is_connected_) {
          AINFO << "Connection lost, attempting to reconnect...";
          Reconnect();
          if (!is_connected_) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(FLAGS_cloud_reconnect_delay_ms));
            continue;
          }
        }

        std::vector<uint8_t> buffer;
        std::vector<uint8_t> temp_buffer(
            static_cast<size_t>(FLAGS_cloud_rw_block_size));
        ssize_t bytesRead =
            recv(sockfd_, temp_buffer.data(), temp_buffer.size(), 0);
        if (0 >= bytesRead) {
          if (bytesRead < 0 &&
              (EAGAIN == errno || EWOULDBLOCK == errno)) {
            AERROR << "Waiting response timed out after "
                   << FLAGS_cloud_response_timeout_ms
                   << " ms. Attempting to reconnect.";
          }
          AERROR << "Connection error or server disconnected. Attempting to "
                    "reconnect.";
          Disconnect();
          continue;
        }

        buffer.insert(buffer.end(), temp_buffer.begin(),
                      temp_buffer.begin() + bytesRead);
        dataHandler.HandleData(buffer);
      }

      Disconnect();
    }).detach();
  }

  bool SendMessage(const std::vector<uint8_t>& message) {
    if (!is_connected_) {
      AERROR << "Error: Not connected to the server.";
      return false;
    }

    std::lock_guard<std::mutex> lock(send_mutex_);
    if (sockfd_ == -1) {
      AERROR << "Error: Not connected to the server.";
      return false;
    }

    ssize_t totalBytesSent = 0;
    ssize_t messageSize = message.size();
    const size_t rw_block_size =
        static_cast<size_t>(FLAGS_cloud_rw_block_size);

    while (totalBytesSent < messageSize) {
      const size_t bytes_to_send = std::min(
          rw_block_size, static_cast<size_t>(messageSize - totalBytesSent));
      ssize_t bytesSent = send(sockfd_, message.data() + totalBytesSent,
                               bytes_to_send, 0);
      if (bytesSent <= 0) {
        AERROR << "Error: Failed to send data.";
        return false;
      }
      totalBytesSent += bytesSent;
    }

    return true;
  }

  void StopReceiving() {
    stop_ = true;
    Disconnect();
  }

  bool IsConnect() { return is_connected_; }

 private:
  bool ConfigureSocketTimeouts(int sockfd) {
    const int response_timeout_ms = FLAGS_cloud_response_timeout_ms;
    timeval recv_timeout;
    recv_timeout.tv_sec = response_timeout_ms / 1000;
    recv_timeout.tv_usec = (response_timeout_ms % 1000) * 1000;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout,
                   sizeof(recv_timeout)) < 0) {
      return false;
    }
    return true;
  }

  bool Reconnect() {
    for (int attempt = 1; attempt <= FLAGS_cloud_max_reconnect_attempts;
         ++attempt) {
      AERROR << "Reconnect attempt " << attempt << " of "
            << FLAGS_cloud_max_reconnect_attempts;
      if (ConnectToServer()) {
        AINFO << "Reconnected to server.";
        return true;
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(FLAGS_cloud_reconnect_delay_ms));
    }
    AERROR << "Failed to reconnect after "
           << FLAGS_cloud_max_reconnect_attempts << " attempts.";
    return false;
  }

 private:
  std::string server_ip_;
  int server_port_;
  int sockfd_;
  std::atomic<bool> stop_;
  std::mutex send_mutex_;
  std::atomic<bool> is_connected_;
};

}  // namespace mcloud
}  // namespace century
