#include "modules/localization/msf/msf_task.h"

namespace century {
namespace loc {

Task::Task(const double& time) : timestamp_(time) {}

Task::~Task() {}

void Task::SetWorkItem(const WorkItem& work_item) { work_item_ = work_item; }
Task::WorkItem& Task::GetWorkItem() { return work_item_; }

double Task::GetTimeStamp() { return timestamp_; }

TaskPool::TaskPool(const double& max_delay) : max_hisory_span_(max_delay) {
  task_thread_ = std::make_unique<mmath::ThreadPool>(1);

  work_thread_ = std::make_unique<std::thread>([this]() { this->DoWork(); });
  if (nullptr == work_thread_) {
    AERROR << "Init work thread error!";
  }
}

TaskPool::~TaskPool() {
  {
    std::unique_lock<std::mutex> lock(mutex_queue_);
    running_ = false;
  }
  condition_.notify_all();
  work_thread_->join();
}

void TaskPool::Commit(std::shared_ptr<Task> task,
                      const Task::TaskType& task_type) {
  switch (task_type) {
    case Task::IMU_Input: {
      std::unique_lock<std::mutex> lock(mutex_queue_);
      CHECK(task);
      task_queue_.emplace_back(std::move(task));
      ADEBUG << "commit task_queue_ size: " << task_queue_.size();
      lock.unlock();
      condition_.notify_one();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      break;
    }
    case Task::Lidar_Meas: {
      std::unique_lock<std::mutex> lock(mutex_lidar_);
      CHECK(task);
      tasks_lidar_.emplace_back(std::move(task));
      AINFO << "commit tasks_lidar_ size: " << tasks_lidar_.size();
      break;
    }
    case Task::Chassis_Meas: {
      std::unique_lock<std::mutex> lock(mutex_chassis_);
      CHECK(task);
      tasks_chassis_.emplace_back(std::move(task));
      ADEBUG << "commit tasks_chassis_ size: " << tasks_chassis_.size();
      break;
    }
    case Task::Ins_Meas_Pose: {
      std::unique_lock<std::mutex> lock(mutex_ins_pose_);
      CHECK(task);
      tasks_ins_pose_.emplace_back(std::move(task));
      ADEBUG << "commit tasks_ins_pose_ size: " << tasks_ins_pose_.size();
      break;
    }
    case Task::Ins_Meas_Velocity: {
      std::unique_lock<std::mutex> lock(mutex_ins_velocity_);
      CHECK(task);
      tasks_ins_velocity_.emplace_back(std::move(task));
      ADEBUG << "commit tasks_ins_velocity_ size: " << tasks_ins_velocity_.size();
      break;
    }
    default:
      AERROR << "un-supported task type.";
      break;
  }
}

void TaskPool::Schedule(const double time_flag, std::mutex* mtx,
                        std::deque<std::shared_ptr<Task>>* queue_ptr) {
  std::unique_lock<std::mutex> lock(*mtx);
  while (!queue_ptr->empty() &&
         (time_flag - queue_ptr->front()->GetTimeStamp()) > max_hisory_span_) {
    queue_ptr->pop_front();
  }
  while (!queue_ptr->empty() &&
         time_flag >= queue_ptr->front()->GetTimeStamp()) {
    task_queue_.emplace_back(std::move(queue_ptr->front()));
    queue_ptr->pop_front();
  }
}

void TaskPool::DoWork() {
  for (;;) {
    std::shared_ptr<Task> task;
    {
      std::unique_lock<std::mutex> lock(mutex_queue_);
      condition_.wait(lock,
                      [this]() { return !task_queue_.empty() || !running_; });

      if (!task_queue_.empty()) {
        double time_flag = task_queue_.back()->GetTimeStamp();
        Schedule(time_flag, &mutex_lidar_, &tasks_lidar_);
        Schedule(time_flag, &mutex_chassis_, &tasks_chassis_);
        Schedule(time_flag, &mutex_ins_pose_, &tasks_ins_pose_);
        Schedule(time_flag, &mutex_ins_velocity_, &tasks_ins_velocity_);
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      } else if (!running_) {
        return;
      }
    }
    // timer_.StartCounter("DoWork");
    CHECK(task);
    task_thread_->enqueue(task->GetWorkItem());
    // timer_.EndCounter("DoWork");
  }
}

}  // namespace loc
}  // namespace century
