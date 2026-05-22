/******************************************************************************der
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

#include "shm_queue.h"

shm_queue::shm_queue(const std::string& name, size_t capacity, size_t block_size)
    : name_(name), capacity_(capacity), block_size_(block_size),
      notif_fd_(-1), data_fd_(-1), notif_(nullptr), data_ptr_(nullptr) {
    if (!CreateAndMapMemory()) {
        std::cerr << "Failed to create and map shared memory." << std::endl;
        exit(EXIT_FAILURE);
    }
}

shm_queue::~shm_queue() {
    UnmapAndUnlinkMemory();
}

bool shm_queue::CreateAndMapMemory() {
    std::string notif_name = "/" + name_ + "_notif";
    notif_fd_ = shm_open(notif_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (notif_fd_ == -1) {
        perror("shm_open notif");
        return false;
    }
    if (ftruncate(notif_fd_, sizeof(Notification)) == -1) {
        perror("ftruncate notif");
        return false;
    }
    notif_ = static_cast<Notification*>(mmap(nullptr, sizeof(Notification),
                                             PROT_READ | PROT_WRITE, MAP_SHARED, notif_fd_, 0));
    if (notif_ == MAP_FAILED) {
        perror("mmap notif");
        return false;
    }

    notif_->producer_seq.store(0, std::memory_order_release);
    notif_->consumer_seq.store(0, std::memory_order_release);

    std::string data_name = "/" + name_ + "_data";
    size_t data_size = capacity_ * block_size_;
    data_fd_ = shm_open(data_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (data_fd_ == -1) {
        perror("shm_open data");
        return false;
    }
    if (ftruncate(data_fd_, data_size) == -1) {
        perror("ftruncate data");
        return false;
    }
    data_ptr_ = mmap(nullptr, data_size, PROT_READ | PROT_WRITE, MAP_SHARED, data_fd_, 0);
    if (data_ptr_ == MAP_FAILED) {
        perror("mmap data");
        return false;
    }

    return true;
}

void shm_queue::UnmapAndUnlinkMemory() {
    if (notif_) {
        munmap(notif_, sizeof(Notification));
        notif_ = nullptr;
    }
    if (notif_fd_ != -1) {
        close(notif_fd_);
        notif_fd_ = -1;
    }
    if (data_ptr_) {
        munmap(data_ptr_, capacity_ * block_size_);
        data_ptr_ = nullptr;
    }
    if (data_fd_ != -1) {
        close(data_fd_);
        data_fd_ = -1;
    }

    std::string notif_name = "/" + name_ + "_notif";
    std::string data_name = "/" + name_ + "_data";
    shm_unlink(notif_name.c_str());
    shm_unlink(data_name.c_str());
}

bool shm_queue::Enqueue(const void* data, size_t size) {
    if (size > block_size_ - sizeof(size_t)) {
        std::cerr << "Data size exceeds block size." << std::endl;
        return false;
    }

    uint64_t prod_seq = notif_->producer_seq.load(std::memory_order_acquire);
    uint64_t cons_seq = notif_->consumer_seq.load(std::memory_order_acquire);

    if (prod_seq - cons_seq >= capacity_) {
        return false;
    }

    size_t index = prod_seq % capacity_;
    char* dest = static_cast<char*>(data_ptr_) + index * block_size_;

    *reinterpret_cast<size_t*>(dest) = size;

    memcpy(dest + sizeof(size_t), data, size);

    notif_->producer_seq.store(prod_seq + 1, std::memory_order_release);

    return true;
}

bool shm_queue::Dequeue(void* buffer, size_t buffer_size, size_t& data_size) {
    uint64_t prod_seq = notif_->producer_seq.load(std::memory_order_acquire);
    uint64_t cons_seq = notif_->consumer_seq.load(std::memory_order_acquire);

    if (cons_seq == prod_seq) {
        return false;
    }

    size_t index = cons_seq % capacity_;
    char* src = static_cast<char*>(data_ptr_) + index * block_size_;

    data_size = *reinterpret_cast<size_t*>(src);
    if (data_size > buffer_size) {
        std::cerr << "Buffer size is smaller than actual data size." << std::endl;
        return false;
    }

    memcpy(buffer, src + sizeof(size_t), data_size);

    notif_->consumer_seq.store(cons_seq + 1, std::memory_order_release);

    return true;
}
