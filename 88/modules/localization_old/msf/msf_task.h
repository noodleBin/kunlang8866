#pragma once
#include <deque>
#include <memory>
#include <utility>

#include "glog/logging.h"
#include "third_party/mmath/thread_pool.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/localization/common/loc_time.h"
namespace century {
namespace loc {

class Task {
 public:
  enum TaskType {
    IMU_Input,
    Lidar_Meas,
    Chassis_Meas,
    Ins_Meas_Pose,
    Ins_Meas_Velocity,
  };
  using WorkItem = std::function<void()>;

  explicit Task(const double& time);
  ~Task();

  void SetWorkItem(const WorkItem& work_item);
  Task::WorkItem& GetWorkItem();
  double GetTimeStamp();

 private:
  WorkItem work_item_;
  std::mutex mutex_;
  double timestamp_;
};

class TaskPool {
 public:
  explicit TaskPool(const double& max_delay);
  ~TaskPool();
  void Commit(const std::shared_ptr<Task> task,
              const Task::TaskType& task_type);

 private:
  void DoWork();
  void Schedule(const double time_flag, std::mutex* mtx,
                std::deque<std::shared_ptr<Task>>* queue);
  std::condition_variable condition_;
  std::mutex mutex_queue_;
  std::mutex mutex_odom_;
  std::mutex mutex_lidar_;
  std::mutex mutex_chassis_;
  std::mutex mutex_ins_pose_;
  std::mutex mutex_ins_velocity_;
  std::deque<std::shared_ptr<Task>> task_queue_;
  std::deque<std::shared_ptr<Task>> tasks_odom_;
  std::deque<std::shared_ptr<Task>> tasks_lidar_;
  std::deque<std::shared_ptr<Task>> tasks_chassis_;
  std::deque<std::shared_ptr<Task>> tasks_ins_pose_;
  std::deque<std::shared_ptr<Task>> tasks_ins_velocity_;
  std::unique_ptr<std::thread> work_thread_;
  std::unique_ptr<mmath::ThreadPool> task_thread_;
  bool running_ = true;
  double max_hisory_span_;
  century::RuntimeCounter timer_;
};

}  // namespace loc
}  // namespace century
