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

#include "cyber/transport/common/proxy_manager.h"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <chrono>
#include <vector>

namespace century {
namespace cyber {
namespace transport {

ProxyManager* ProxyManager::Instance() {
  static ProxyManager instance;
  return &instance;
}

ProxyManager::~ProxyManager() {
  std::vector<std::thread> threads;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    threads.reserve(entries_.size());
    for (auto& item : entries_) {
      item.second->running = false;
      if (item.second->thread.joinable()) {
        threads.emplace_back(std::move(item.second->thread));
      }
    }
  }
  for (auto& thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& item : entries_) {
      ReleaseLocked(item.second.get());
    }
    entries_.clear();
  }
}

bool ProxyManager::TryAcquire(uint64_t channel_id,
                              const std::string& proto) {
  const auto key = MakeKey(channel_id, proto);
  const auto path = MakePath(channel_id, proto);
  std::lock_guard<std::mutex> lock(mutex_);
  auto& entry = entries_[key];
  if (!entry) {
    entry = std::unique_ptr<Entry>(new Entry());
  }
  if (entry->is_owner) {
    return true;
  }
  return TryAcquireLocked(entry.get(), path);
}

void ProxyManager::Release(uint64_t channel_id, const std::string& proto) {
  const auto key = MakeKey(channel_id, proto);
  std::lock_guard<std::mutex> lock(mutex_);
  auto iter = entries_.find(key);
  if (entries_.end() == iter) {
    return;
  }
  ReleaseLocked(iter->second.get());
}

bool ProxyManager::IsOwner(uint64_t channel_id, const std::string& proto) {
  const auto key = MakeKey(channel_id, proto);
  std::lock_guard<std::mutex> lock(mutex_);
  auto iter = entries_.find(key);
  if (entries_.end() == iter) {
    return false;
  }
  return iter->second->is_owner;
}

bool ProxyManager::StartWatch(uint64_t channel_id, const std::string& proto,
                              const PromoteCallback& on_promote) {
  const auto key = MakeKey(channel_id, proto);
  std::lock_guard<std::mutex> lock(mutex_);
  auto& entry = entries_[key];
  if (!entry) {
    entry = std::unique_ptr<Entry>(new Entry());
  }
  if (entry->running) {
    return false;
  }
  entry->running = true;
  entry->on_promote = on_promote;
  entry->thread =
      std::thread(&ProxyManager::WatchLoop, this, key, channel_id, proto);
  return true;
}

void ProxyManager::StopWatch(uint64_t channel_id, const std::string& proto) {
  const auto key = MakeKey(channel_id, proto);
  std::thread thread;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = entries_.find(key);
    if (entries_.end() == iter) {
      return;
    }
    iter->second->running = false;
    thread = std::move(iter->second->thread);
  }
  if (thread.joinable()) {
    thread.join();
  }
}

bool ProxyManager::TryAcquireLocked(Entry* entry, const std::string& path) {
  if (nullptr == entry) {
    return false;
  }
  if (entry->fd < 0) {
    int fd = open(path.c_str(), O_CREAT | O_RDWR, 0644);
    if (fd < 0) {
      return false;
    }
    entry->fd = fd;
  }
  if (flock(entry->fd, LOCK_EX | LOCK_NB) != 0) {
    return false;
  }
  entry->is_owner = true;
  return true;
}

void ProxyManager::ReleaseLocked(Entry* entry) {
  if (nullptr == entry || entry->fd < 0) {
    return;
  }
  (void)flock(entry->fd, LOCK_UN);
  close(entry->fd);
  entry->fd = -1;
  entry->is_owner = false;
}

void ProxyManager::WatchLoop(const Key& key, uint64_t channel_id,
                             const std::string& proto) {
  const auto path = MakePath(channel_id, proto);
  constexpr auto kRetry = std::chrono::milliseconds(200);
  while (true) {
    PromoteCallback cb;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto iter = entries_.find(key);
      if (entries_.end() == iter) {
        return;
      }
      Entry* entry = iter->second.get();
      if (!entry->running) {
        return;
      }
      if (entry->is_owner) {
        entry->running = false;
        return;
      }
      if (TryAcquireLocked(entry, path)) {
        entry->running = false;
        cb = entry->on_promote;
      }
    }
    if (cb) {
      cb();
      return;
    }
    std::this_thread::sleep_for(kRetry);
  }
}

ProxyManager::Key ProxyManager::MakeKey(uint64_t channel_id,
                                        const std::string& proto) const {
  return proto + ":" + std::to_string(channel_id);
}

std::string ProxyManager::MakePath(uint64_t channel_id,
                                   const std::string& proto) const {
  return "/tmp/cyber_" + proto + "_proxy_" + std::to_string(channel_id);
}

}  // namespace transport
}  // namespace cyber
}  // namespace century
