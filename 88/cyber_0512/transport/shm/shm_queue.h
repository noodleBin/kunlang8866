
/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

template <typename T>
using CallbackFunc = std::function<void(std::shared_ptr<T>)>;

constexpr size_t DEFAULT_CAPACITY = 5;

namespace century {
namespace cyber {
namespace transport {

template <typename T>
class shm_queue {
 public:
  shm_queue(const std::string& name);
  shm_queue(const std::string& name, const CallbackFunc<T>& cb);
  ~shm_queue();
  bool Write(const T* data);
  void Notify();
  void Listen();
  T* GetLatestMessage();
  T* GetNewObject();

 private:
  shm_queue(const shm_queue&) = delete;
  shm_queue& operator=(const shm_queue&) = delete;

  bool CreateAndMapMemory();
  void UnmapAndUnlinkMemory();
  bool Fetch(T*& buffer);

 private:
  std::string name_;
  size_t capacity_;
  bool start_ = false;

  int notify_fd_;
  int data_fd_;

  struct alignas(64) Notification {
    std::atomic<uint64_t> producer_seq;
  }* notify_;

  T* data_array_ = nullptr;
  CallbackFunc<T> callback_;
  std::unique_ptr<std::thread> listen_thread_;
  std::atomic<uint64_t> local_consumer_seq_;
};

template <typename T>
shm_queue<T>::shm_queue(const std::string& name)
    : name_(name), capacity_(DEFAULT_CAPACITY), notify_fd_(-1), data_fd_(-1) {
  if (!CreateAndMapMemory()) {
    throw std::runtime_error("Failed to create shared memory queue");
  }
  local_consumer_seq_.store(notify_->producer_seq.load() + 1);
  //  listen_thread_.reset(new std::thread(&shm_queue::Listen, this));
}

template <typename T>
shm_queue<T>::shm_queue(const std::string& name, const CallbackFunc<T>& cb)
    : name_(name), capacity_(DEFAULT_CAPACITY), notify_fd_(-1), data_fd_(-1) {
  if (!CreateAndMapMemory()) {
    throw std::runtime_error("Failed to create shared memory queue");
  }
  callback_ = cb;
  local_consumer_seq_.store(notify_->producer_seq.load() + 1);
  listen_thread_.reset(new std::thread(&shm_queue::Listen, this));
}

template <typename T>
shm_queue<T>::~shm_queue() {
  start_ = false;
  if (listen_thread_ && listen_thread_->joinable()) {
    listen_thread_->join();
  }
  UnmapAndUnlinkMemory();
}

template <typename T>
bool shm_queue<T>::CreateAndMapMemory() {
  std::hash<std::string> hasher;

  std::string notify_name = name_ + "_notif";
  bool is_new = false;
  std::string hash_notify_name = std::to_string(hasher(notify_name));
  notify_fd_ = shm_open(hash_notify_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  if (notify_fd_ != -1) {
    is_new = true;
  } else if (errno == EEXIST) {
    notify_fd_ = shm_open(hash_notify_name.c_str(), O_RDWR, 0666);
    if (notify_fd_ == -1) return false;
  } else {
    return false;
  }

  if (is_new && ftruncate(notify_fd_, sizeof(Notification)) == -1) return false;

  notify_ = static_cast<Notification*>(mmap(nullptr, sizeof(Notification),
                                            PROT_READ | PROT_WRITE, MAP_SHARED,
                                            notify_fd_, 0));
  if (notify_ == MAP_FAILED) return false;

  if (is_new) notify_->producer_seq.store(0);

  std::string data_name = name_ + "_data";
  std::string hash_data_name = std::to_string(hasher(data_name));

  data_fd_ = shm_open(hash_data_name.c_str(), O_CREAT | O_RDWR, 0666);
  if (data_fd_ == -1) return false;

  const size_t data_size = capacity_ * sizeof(T);
  if (ftruncate(data_fd_, data_size) == -1) return false;

  void* ptr =
      mmap(nullptr, data_size, PROT_READ | PROT_WRITE, MAP_SHARED, data_fd_, 0);
  if (ptr == MAP_FAILED) return false;

  data_array_ = static_cast<T*>(ptr);
  return true;
}

template <typename T>
void shm_queue<T>::UnmapAndUnlinkMemory() {
  if (notify_) {
    munmap(notify_, sizeof(Notification));
    notify_ = nullptr;
  }

  if (data_array_) {
    munmap(data_array_, capacity_ * sizeof(T));
    data_array_ = nullptr;
  }
}

template <typename T>
bool shm_queue<T>::Write(const T* data) {
  const uint64_t next_seq =
      notify_->producer_seq.load(std::memory_order_relaxed) + 1;
  const size_t index = next_seq % capacity_;
  data_array_[index] = *data;
  std::atomic_thread_fence(std::memory_order_release);
  notify_->producer_seq.store(next_seq, std::memory_order_release);
  return true;
}

template <typename T>
void shm_queue<T>::Notify() {
  const uint64_t next_seq =
      notify_->producer_seq.load(std::memory_order_relaxed) + 1;
  notify_->producer_seq.store(next_seq, std::memory_order_release);
}

//  template <typename T>
//  void shm_queue<T>::Listen() {
//    start_ = true;
//    T* data;
//    while (start_) {
//      if (Fetch(data)) {
//        if (callback_) {
//          callback_(data);
//        }
//      }
//      std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    }
//  }
template <typename T>
void shm_queue<T>::Listen() {
  start_ = true;
  T* data;
  while (start_) {
    if (Fetch(data)) {
      if (callback_) {
        callback_(std::shared_ptr<T>(data, [](T*) {}));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

template <typename T>
bool shm_queue<T>::Fetch(T*& buffer) {
  uint64_t prod_seq = notify_->producer_seq.load(std::memory_order_acquire);
  uint64_t cons_seq = local_consumer_seq_.load(std::memory_order_relaxed);

  if (cons_seq > prod_seq && (cons_seq - prod_seq < UINT64_MAX / 2)) {
    return false;
  }

  const size_t index = cons_seq % capacity_;
  buffer = &data_array_[index];

  local_consumer_seq_.store(cons_seq + 1, std::memory_order_release);
  return true;
}

template <typename T>
T* shm_queue<T>::GetLatestMessage() {
  uint64_t prod_seq = notify_->producer_seq.load(std::memory_order_acquire);
  const size_t index = prod_seq % capacity_;
  return &data_array_[index];
}

template <typename T>
T* shm_queue<T>::GetNewObject() {
  const uint64_t next_seq =
      notify_->producer_seq.load(std::memory_order_relaxed) + 1;
  const size_t index = next_seq % capacity_;
  return &data_array_[index];
}

}  // namespace transport
}  // namespace cyber
}  // namespace century