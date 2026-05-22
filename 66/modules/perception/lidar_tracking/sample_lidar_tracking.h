#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <list>
#include <memory>
#include <iostream>
#include "modules/perception/lidar_tracking/lidar_tracking_component.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "cyber/common/file.h"
namespace century {
namespace perception {
namespace lidar {

static std::string perception_module_name;
static std::string localization_module_name;


template <typename T>
class ProducerConsumerQueue {
public:
  static ProducerConsumerQueue* GetInstance() {
    static ProducerConsumerQueue produce_consumer_;
    return &produce_consumer_;
  }

  void Init(const uint32_t& wait_seconds) {
    wait_seconds_ = wait_seconds;
  }

  void Produce(T value) {
    std::unique_lock<std::mutex> lock(mutex_);
  //  AINFO << "produce" << std::endl;
    queue_.push_back(value);
    lock.unlock();
    not_empty_.notify_one();
  }

  std::list<T> Consume() {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait_for(lock, std::chrono::seconds(wait_seconds_), [this] { return !queue_.empty(); });
    std::list<T> value;
    value.splice(value.end(), queue_);
   // AINFO << "consume" << std::endl;
    lock.unlock();
    return value;
  }

private:
  ProducerConsumerQueue() = default;
  ~ProducerConsumerQueue() = default;
  std::list<T> queue_;
  const size_t capacity_ = 100;
  std::mutex mutex_;
  std::condition_variable not_empty_;
  uint32_t wait_seconds_ = 5;
};

template <typename T>
class WriteOnceReadAllQueue {
public:
  static WriteOnceReadAllQueue* GetInstance() {
    static WriteOnceReadAllQueue produce_consumer_;
    return &produce_consumer_;
  }
   
  void Push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
  //  AINFO << "push" << std::endl;
    queue_.push_back(value);
  }

  void ReadAll(std::list<T>& out_queue) {
    std::lock_guard<std::mutex> lock(mutex_);
  //  AINFO << "read all" << std::endl;
    out_queue.swap(queue_);
    return ;
  }

private:
  WriteOnceReadAllQueue() = default;
  ~WriteOnceReadAllQueue() = default;
  mutable std::mutex mutex_; 
  std::list<T> queue_; 
};


template <typename MsgTypePtr>
void ReceiveMessageCallback(const MsgTypePtr& msg) {
  if(!msg) {
    return;
  }
  if(msg->header().module_name() == perception_module_name) {
    ProducerConsumerQueue<MsgTypePtr>::GetInstance()->Produce(msg);
   // AINFO << msg->DebugString() << std::endl;
  } else if (msg->header().module_name() == localization_module_name) {
    WriteOnceReadAllQueue<MsgTypePtr>::GetInstance()->Push(msg);
   // AINFO << msg->DebugString() << std::endl;
  }
}
}
}
}
