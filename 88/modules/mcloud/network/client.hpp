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

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include "cyber/cyber.h"
#include "modules/mcloud/network/data_handler.h"

namespace century {
namespace mcloud {

constexpr size_t buffer_size = 1024;
constexpr int max_reconnect_attempts = 5;
constexpr int reconnect_delay_ms = 2000;

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

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(server_port_);
    inet_pton(AF_INET, server_ip_.c_str(), &serverAddr.sin_addr);
    
    if (connect(sockfd_, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) <
        0) {
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
                std::chrono::milliseconds(reconnect_delay_ms));
            continue;
          }
        }

        std::vector<uint8_t> buffer;
        uint8_t tempBuffer[buffer_size];
        ssize_t bytesRead = recv(sockfd_, tempBuffer, sizeof(tempBuffer), 0);
        if (bytesRead <= 0) {
          AERROR << "Connection error or server disconnected. Attempting to "
                    "reconnect.";
          Disconnect();
          continue;
        }

        buffer.insert(buffer.end(), tempBuffer, tempBuffer + bytesRead);
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

    while (totalBytesSent < messageSize) {
      ssize_t bytesSent = send(sockfd_, message.data() + totalBytesSent,
                               messageSize - totalBytesSent, 0);
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
  bool Reconnect() {
    for (int attempt = 1; attempt <= max_reconnect_attempts; ++attempt) {
      AERROR << "Reconnect attempt " << attempt << " of "
            << max_reconnect_attempts;
      if (ConnectToServer()) {
        AINFO << "Reconnected to server.";
        return true;
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds(reconnect_delay_ms));
    }
    AERROR << "Failed to reconnect after " << max_reconnect_attempts
           << " attempts.";
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
