#include "amcl_recorder.h"
#include "glog/logging.h"

namespace landmark_loc{
void AmclRecorder::AddData(const Eigen::Vector3d dr, const Eigen::Vector3d amcl, const double ts,
                           const std::vector<Eigen::Vector3d> particle, const Eigen::Vector3d cov, 
                           const uint32_t semantic_count, const uint32_t cluster_count) {
  if(!inited_) {
    //LOG(WARNING) << "AmclRecorder not inited properly.";
    return;
  }
  auto cur_package = record_.add_package();
  cur_package->set_semantic_count(semantic_count);
  cur_package->set_cluster_count(cluster_count);
  cur_package->set_ts(ts);
  auto* dr_pose = new century::perception::landmark_loc::AmclPoint();
  dr_pose->set_x(dr(0));
  dr_pose->set_y(dr(1));
  dr_pose->set_z(dr(2));
  cur_package->set_allocated_dr(dr_pose);
 
  auto* amcl_pose =  new century::perception::landmark_loc::AmclPoint();
  amcl_pose->set_x(amcl(0));
  amcl_pose->set_y(amcl(1));
  amcl_pose->set_z(amcl(2));
  cur_package->set_allocated_amcl_point (amcl_pose);

  auto* covariance =  new century::perception::landmark_loc::AmclCov();
  covariance->set_x_x(cov(0));
  covariance->set_y_y(cov(1));
  covariance->set_yaw_yaw(cov(2));
  cur_package->set_allocated_amcl_cov(covariance);

  auto* amcl_particle =  new century::perception::landmark_loc::AmclParticle();
  for(const auto& it : particle) {
    auto* particle_point = amcl_particle->add_point();
    particle_point->set_x(it(0));
    particle_point->set_y(it(1));
    particle_point->set_z(it(2));
  }
  cur_package->set_allocated_particles(amcl_particle);

  if(seq_++ >= item_count_) {
    const std::string file_name = save_path_ + pose_label.at(AMCL) + std::to_string(ts) + ".pb.bin";
    LOG(INFO) << "save amcl record to " << file_name;
    century::cyber::common::SetProtoToBinaryFile(record_, file_name);
    record_.Clear();
    seq_ = 0;
  }

  return;
}

bool AmclRecorder::EnsureDirectory(const std::string& path) {

  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    if (info.st_mode & S_IFDIR) {
     // AINFO << "Folder already exists: " << path << std::endl;
      return true;
    } else {
      LOG(INFO) << "Path exists but is not a directory: " << path << std::endl;
      return false;
    }
  } else {
    if (mkdir(path.c_str(), 0755) == 0) {
      LOG(INFO) << "Folder created successfully: " << path << std::endl;
      return true;
    } else {
      LOG(INFO) << "Failed to create folder: " << path << std::endl;
      return false;
    }
  }
}

AmclRecorder::~AmclRecorder() {
  stop_thread_ = true;
  std::unique_lock<std::mutex> lock(mutex_);
  lock.unlock();
  not_empty_.notify_one();
  if(recorder_thread_) {
    recorder_thread_->join();
    delete recorder_thread_;
  }
}

AmclRecorder::AmclRecorder() {
  for(const auto& label : pose_label) {
    inited_ = EnsureDirectory(save_path_ + label);
    if(!inited_) {
      LOG(ERROR) << "AmclRecorder init failed.";
      return;
    }
  }
  result_.reset(new std::array<century::perception::landmark_loc::AmclPredictedPoses,2>);
  recorder_thread_ = new std::thread(&AmclRecorder::DoWork, this);
}

void AmclRecorder::DoWork() {
  LOG(INFO) <<"Recorder thread start";
  while (!stop_thread_) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock);
    if(stop_thread_) {
      break;
    }
    auto result_dump = result_dump_;
    auto dump_ts = dump_ts_;
    lock.unlock();
    if(result_dump_) {
      const std::string file_predicted = save_path_ + pose_label.at(PREDICTED) + std::to_string(dump_ts) + ".pb.bin";
      
      if(result_dump_->at(PREDICTED).pose_size() > 0) {
        LOG(INFO) << "save predicted record to " << file_predicted;
        century::cyber::common::SetProtoToBinaryFile(result_dump_->at(0), file_predicted);
      }
     
      const std::string file_dr = save_path_ + pose_label.at(DR) + std::to_string(dump_ts) + ".pb.bin";
      if(result_dump_->at(DR).pose_size() > 0) {
        LOG(INFO) << "save dr record to " << file_dr;
        century::cyber::common::SetProtoToBinaryFile(result_dump_->at(1), file_dr);
      }
     
      result_dump_.reset();
    }
  }
}

void AmclRecorder::AddResult(const Eigen::Vector3d predicted_pose, const double ts, const int indice) {
  if(!inited_) {
   // LOG(WARNING) << "AmclRecorder not inited properly.";
    return;
  }

  auto* cur_pose = result_->at(indice).add_pose();
  cur_pose->set_ts(ts);
  auto* pose = new century::perception::landmark_loc::AmclPoint();
  pose->set_x(predicted_pose(0));
  pose->set_y(predicted_pose(1));
  pose->set_z(predicted_pose(2));
  cur_pose->set_allocated_position(pose);

  if(result_seq_++ >= result_count_) {
    std::unique_lock<std::mutex> lock(mutex_);
    result_dump_ = result_;
    dump_ts_ = ts;
    lock.unlock();
    not_empty_.notify_one();
    result_seq_ = 0;
    result_.reset(new std::array<century::perception::landmark_loc::AmclPredictedPoses,2>);
  }


  return;
}
}  // namespace landmark_loc