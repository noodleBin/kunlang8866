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

#ifndef SHM_QUEUE_H
#define SHM_QUEUE_H

#include <atomic>
#include <cstdint>
#include <string>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class shm_queue {
public:
    shm_queue(const std::string& name, size_t capacity = 10, size_t block_size = 4 * 1024 * 1024);

    ~shm_queue();

    bool Enqueue(const void* data, size_t size);

    bool Dequeue(void* buffer, size_t buffer_size, size_t& data_size);

private:
    shm_queue(const shm_queue&) = delete;
    shm_queue& operator=(const shm_queue&) = delete;

    bool CreateAndMapMemory();
    void UnmapAndUnlinkMemory();

    std::string name_;      
    size_t capacity_;            
    size_t block_size_;          

    int notif_fd_;               
    int data_fd_;              

    struct Notification {
        std::atomic<uint64_t> producer_seq;
        std::atomic<uint64_t> consumer_seq;
    } *notif_;                  

    void* data_ptr_;             
};

#endif // SHM_QUEUE_H
