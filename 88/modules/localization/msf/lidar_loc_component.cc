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
#include "modules/localization/msf/lidar_loc_component.h"

#include "lidar_loc_component.h"

namespace century {
namespace loc {

LidarLocComponent::LidarLocComponent() {}
LidarLocComponent::~LidarLocComponent() {
#if SAVE_SHOW_DATA_LIDAR
  if (lidar_pose_file_ && lidar_pose_file_->is_open()) {
      lidar_pose_file_->close();
  }
#endif
}

bool LidarLocComponent::Init() {
  if (!InitConfig()) {
    AERROR << "InitConfig failed!";
    return false;
  }

  if (!InitParams()) {
    AERROR << "InitParams failed!";
    return false;
  }

  if (!InitReaderWriter()) {
    AERROR << "InitReaderWriter failed!";
    return false;
  }

  return true;
}

bool LidarLocComponent::InitConfig() {
  // set config
  AINFO << "config_file_path_: " << config_file_path_;

  if (!cyber::common::GetProtoFromFile(config_file_path_, &config_.lidar_cfg)) {
    AERROR << "Load lidar config failed!";
    return false;
  }
  AINFO << "-----Load lidar config success!-----";
  config_.loc_config_path = config_.lidar_cfg.loc_config_path();
  YAML::Node config_node = YAML::LoadFile(config_.loc_config_path);
  config_.ndt_map_path = config_node["ndt_map_path"].as<std::string>();
  config_.left_lidar_topic = config_node["left_lidar_topic"].as<std::string>();
  config_.right_lidar_topic =
      config_node["right_lidar_topic"].as<std::string>();
  config_.lidar_loc_topic = config_node["lidar_loc_topic"].as<std::string>();
  config_.msf_loc_topic = config_node["msf_loc_topic"].as<std::string>();
  config_.left_lidar_trans_topic =
      config_node["left_lidar_trans_topic"].as<std::string>();
  config_.right_lidar_trans_topic =
      config_node["right_lidar_trans_topic"].as<std::string>();
  config_.lidar_calib_file = config_node["lidar_calib_file"].as<std::string>();
  config_.is_sim = config_.lidar_cfg.is_sim();
  config_.sync_time_thresh = config_.lidar_cfg.sync_time_thresh();
  AINFO << "loc_config_path: " << config_.loc_config_path;
  AINFO << "left_lidar_topic: " << config_.left_lidar_topic;
  AINFO << "right_lidar_topic: " << config_.right_lidar_topic;
  AINFO << "msf_loc_topic: " << config_.msf_loc_topic;

  // parse config
  config_parser_ = std::make_unique<common::io::ConfigParser>();
  message_parser_ = std::make_unique<common::io::MessageParser>();
  left_feature_extractor_ = std::make_unique<LidarFeatureMultiThread>();
  right_feature_extractor_ = std::make_unique<LidarFeatureMultiThread>();
  lidar_locator_ = std::make_unique<localization::ndt::LidarLocatorNdt>();

  shm_cloud_left_ = std::make_unique<cyber::transport::shm_queue<PointCloudShm>>(config_.left_lidar_topic);
  shm_cloud_right_ = std::make_unique<cyber::transport::shm_queue<PointCloudShm>>(config_.right_lidar_topic);

  // lidar
  if (!config_parser_->LoadLidarConfig(
          config_.lidar_calib_file, &config_.Tx_veh_lidar1,
          &config_.Tx_imu_lidar1, &config_.Tx_veh_lidar2,
          &config_.Tx_imu_lidar2)) {
    AERROR << "LoadLidarConfig failed!";
    return false;
  }

  // ndt map
  if (!config_parser_->LoadNdtMapConfig(config_.ndt_map_path,
                                        &ndt_map_config_)) {
    AERROR << "Load ndt map config failed!";
    return false;
  } else {
    AINFO << "-----Load ndt map config success!-----";
    AINFO << "ndt map path: " << config_.ndt_map_path;
    AINFO << "ndt map version: " << ndt_map_config_.coordinate_type;
    AINFO << "map node size(x, y): " << ndt_map_config_.map_node_size_x << ", "
          << ndt_map_config_.map_node_size_y;
    AINFO << "map range(min_x, min_y, max_x, max_y): "
          << ndt_map_config_.map_min_x << ", " << ndt_map_config_.map_min_y
          << ", " << ndt_map_config_.map_max_x << ", "
          << ndt_map_config_.map_max_y;
    AINFO << "map resolution: " << ndt_map_config_.map_resolution;
    AINFO << "map ground height offset: "
          << ndt_map_config_.map_ground_height_offset;
    AINFO << "--------------------------------------";
  }

  // try load zone id from local_map folder
  if (FLAGS_if_utm_zone_id_from_folder) {
    bool success = LoadZoneIdFromFolder(config_.ndt_map_path, &config_.zone_id);
    if (!success) {
      AWARN << "Can't load utm zone id from map folder, use default value.";
    }
  }
  AINFO << "utm zone id: " << config_.zone_id;

  lidar_locator_->SetMapFolderPath(config_.ndt_map_path);
  lidar_locator_->SetLidarHeight(config_.lidar_height);
  lidar_locator_->SetOnlineCloudResolution(
      static_cast<float>(ndt_map_config_.map_resolution));

  return true;
}

bool LidarLocComponent::InitParams() {
  // set interpolater
  raw_imu_interpolater_ = std::make_unique<Vector6dLinearInterpolater>(
      config_.interpolater_max_history_us, config_.interpolater_max_predict_us);
  linear_vel_Mp_imu_interpolater_ =
      std::make_unique<Vector3dLinearInterpolater>(
          config_.interpolater_max_history_us,
          config_.interpolater_max_predict_us);
  angular_vel_Mp_imu_interpolater_ =
      std::make_unique<Vector3dLinearInterpolater>(
          config_.interpolater_max_history_us,
          config_.interpolater_max_predict_us);

  Tx_Mp_L1_interpolater_ = std::make_unique<SE3LinearInterpolater>(
      config_.interpolater_max_history_us, config_.interpolater_max_predict_us);
  Tx_Mp_L2_interpolater_ = std::make_unique<SE3LinearInterpolater>(
      config_.interpolater_max_history_us, config_.interpolater_max_predict_us);
  Tx_Mp_imu_interpolater_ = std::make_unique<SE3LinearInterpolater>(
      config_.interpolater_max_history_us, config_.interpolater_max_predict_us);
  Vel6_Mp_L1_interpolater_ = std::make_unique<Vector6dLinearInterpolater>(
      config_.interpolater_max_history_us, config_.interpolater_max_predict_us);
  Vel6_Mp_L2_interpolater_ = std::make_unique<Vector6dLinearInterpolater>(
      config_.interpolater_max_history_us, config_.interpolater_max_predict_us);
  left_lidar_loc_thread_ = std::make_unique<mmath::ThreadPool>(4);
  right_lidar_loc_thread_ = std::make_unique<mmath::ThreadPool>(4);
  lidar_matcher_timer_.reset(new cyber::Timer(
      LIDAR_FRAME_INTERVAL_NUM * LIDAR_FRAME_INTERVAL_MS,
      [this]() { this->LidarMatcherTimerCallback(); }, false));
  lidar_matcher_timer_->Start();
  return true;
}

bool LidarLocComponent::InitReaderWriter() {
  if (config_.is_sim) {
    left_lidar_listener_ = node_->CreateReader<drivers::PointCloudPacked>(
        config_.left_lidar_topic,
        std::bind(&LidarLocComponent::LeftLidarCallback, this,
                  std::placeholders::_1));
    right_lidar_listener_ = node_->CreateReader<drivers::PointCloudPacked>(
        config_.right_lidar_topic,
        std::bind(&LidarLocComponent::RightLidarCallback, this,
                  std::placeholders::_1));
  } else {
    left_lidar_trans_talker_ =
        node_->CreateWriter<loc::CloudFrame>(config_.left_lidar_trans_topic);
    ACHECK(left_lidar_trans_talker_);

    cyber::ReaderConfig reader_config_left;
    reader_config_left.pending_queue_size = LIDAR_QUEUE_SIZE;
    reader_config_left.channel_name = config_.left_lidar_trans_topic;
    shm_left_lidar_listener_ = node_->CreateReader<loc::CloudFrame>(
        reader_config_left, std::bind(&LidarLocComponent::ShmLeftLidarCallback,
                                      this, std::placeholders::_1));

    right_lidar_trans_talker_ =
        node_->CreateWriter<loc::CloudFrame>(config_.right_lidar_trans_topic);
    ACHECK(right_lidar_trans_talker_);

    cyber::ReaderConfig reader_config_right;
    reader_config_right.pending_queue_size = LIDAR_QUEUE_SIZE;
    reader_config_right.channel_name = config_.right_lidar_trans_topic;
    shm_right_lidar_listener_ = node_->CreateReader<loc::CloudFrame>(
        reader_config_right,
        std::bind(&LidarLocComponent::ShmRightLidarCallback, this,
                  std::placeholders::_1));
  }

  fusion_pose_listener_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          config_.msf_loc_topic,
          std::bind(&LidarLocComponent::FusionPoseCallback, this,
                    std::placeholders::_1));
  lidar_pose_talker_ =
      node_->CreateWriter<localization::PoseWithCov>(config_.lidar_loc_topic);
  ACHECK(lidar_pose_talker_);

#if SAVE_SHOW_DATA_LIDAR
  lidar_pose_file_ = std::make_unique<std::ofstream>("/century/data/bag/match_data/lidar_pose.bin", std::ios::binary | std::ios::app);
  if (!lidar_pose_file_->is_open()) {
      std::cout << "error opening lidar pose file: " << std::endl;
  }
#endif

  return true;
}

bool LidarLocComponent::Proc(
    const std::shared_ptr<drivers::gnss::Imu> &rawimu_msg) {
  if (nullptr == rawimu_msg) {
    return false;
  }

  // AINFO << std::fixed << "##raw imu timestamp: " <<
  // rawimu_msg->header().timestamp_sec()
  //       << " host time: " << cyber::Time::Now().ToSecond();

  if (cyber_likely(!config_.is_sim) && get_fuison_) {
    ReadShmLeftLidarData();
    ReadShmRightLidarData();
  }

  return true;
}

void LidarLocComponent::FusionPoseCallback(
    const std::shared_ptr<localization::LocalizationEstimate> &msg) {
  if (nullptr == msg) {
    return;
  }

  if (false == get_fuison_) {
    get_fuison_ = true;
  }

  double timestamp;
  mmath::SE3 Tx_Mp_imu;
  Eigen::Vector3d linear_vel;
  message_parser_->ParseFusionMsg(msg, &timestamp, &Tx_Mp_imu, &linear_vel);

  TimeUs time = timestamp * kSecond;
  // AINFO << std::fixed << "## fuison_msg timestamp(s): " << timestamp;
  // AINFO << "fuison_pose(imu) trans:" <<
  // Tx_Mp_imu.getTranslation().transpose().x() << "," << 
  // Tx_Mp_imu.getTranslation().transpose().y() << "," << 
  // Tx_Mp_imu.getTranslation().transpose().z() << ", ypr:" << 
  // Tx_Mp_imu.getSO3().getEulerYPR().z() * mmath::kRadToDeg << "," <<
  // Tx_Mp_imu.getSO3().getEulerYPR().y() * mmath::kRadToDeg << "," <<
  // Tx_Mp_imu.getSO3().getEulerYPR().x() * mmath::kRadToDeg;

  if (Tx_Mp_imu_interpolater_ != nullptr) {
    std::unique_lock<std::mutex> lock(interpolater_mutex_);
    Tx_Mp_imu_interpolater_->insert(time, Tx_Mp_imu);
  }
}

void LidarLocComponent::LeftLidarFeatureExtract(
    const double timestamp, const std::shared_ptr<loc::PointCloudXYZIRT> &pcd) {
  auto line_pcd = std::make_shared<loc::PointCloudXYZI>();
  auto plane_pcd = std::make_shared<loc::PointCloudXYZI>();
  {
    RuntimeCounter feature_extractor_runtime;
    feature_extractor_runtime.StartCounter("LeftLidarFeatureExtract");

    left_feature_extractor_->AddPointCloud(*pcd);
    left_feature_extractor_->GetFeaturePointCloud(line_pcd.get(),
                                                  plane_pcd.get());

    feature_extractor_runtime.EndCounter("LeftLidarFeatureExtract");
    AINFO << "line pcd size:" << line_pcd->size()
          << " plane pcd size: " << plane_pcd->size();
  }

  {
    std::lock_guard<std::mutex> lock(left_lidar_buffer_mutex);
    LeftLidar_line_buffer_.emplace_back(std::make_pair(timestamp, line_pcd));
    LeftLidar_plane_buffer_.emplace_back(std::make_pair(timestamp, plane_pcd));
    pcl::io::savePCDFileBinary("/century/data/left_lidar_line_feature/" +
                                   std::to_string(timestamp) + ".pcd",
                               *LeftLidar_line_buffer_.back().second);
    pcl::io::savePCDFileBinary("/century/data/left_lidar_plane_feature/" +
                                   std::to_string(timestamp) + ".pcd",
                               *LeftLidar_plane_buffer_.back().second);

    while (LeftLidar_line_buffer_.size() > LIDAR_BUFFER_SIZE) {
      LeftLidar_line_buffer_.pop_front();
    }

    while (LeftLidar_plane_buffer_.size() > LIDAR_BUFFER_SIZE) {
      LeftLidar_plane_buffer_.pop_front();
    }
    AINFO << "LeftLidar_plane_buffer_ size: " << LeftLidar_plane_buffer_.size();
  }

  left_feature_extract_mutex_.unlock();
}

void LidarLocComponent::RightLidarFeatureExtract(
    const double timestamp, const std::shared_ptr<loc::PointCloudXYZIRT> &pcd) {
  auto line_pcd = std::make_shared<loc::PointCloudXYZI>();
  auto plane_pcd = std::make_shared<loc::PointCloudXYZI>();
  {
    RuntimeCounter feature_extractor_runtime;
    feature_extractor_runtime.StartCounter("RightLidarFeatureExtract");

    right_feature_extractor_->AddPointCloud(*pcd);
    right_feature_extractor_->GetFeaturePointCloud(line_pcd.get(),
                                                   plane_pcd.get());

    feature_extractor_runtime.EndCounter("RightLidarFeatureExtract");

    AINFO << "line pcd size:" << line_pcd->size()
          << " plane pcd size: " << plane_pcd->size();
  }

  {
    std::lock_guard<std::mutex> lock(right_lidar_buffer_mutex);
    RightLidar_line_buffer_.emplace_back(std::make_pair(timestamp, line_pcd));
    RightLidar_plane_buffer_.emplace_back(std::make_pair(timestamp, plane_pcd));
    pcl::io::savePCDFileBinary("/century/data/right_lidar_line_feature/" +
                                   std::to_string(timestamp) + ".pcd",
                               *RightLidar_line_buffer_.back().second);
    pcl::io::savePCDFileBinary("/century/data/right_lidar_plane_feature/" +
                                   std::to_string(timestamp) + ".pcd",
                               *RightLidar_plane_buffer_.back().second);

    while (RightLidar_line_buffer_.size() > LIDAR_BUFFER_SIZE) {
      RightLidar_line_buffer_.pop_front();
    }

    while (RightLidar_plane_buffer_.size() > LIDAR_BUFFER_SIZE) {
      RightLidar_plane_buffer_.pop_front();
    }
    AINFO << "RightLidar_plane_buffer_ size: "
          << RightLidar_plane_buffer_.size();
  }

  right_feature_extract_mutex_.unlock();
}

void LidarLocComponent::LeftLidarCallback(
    const std::shared_ptr<drivers::PointCloudPacked> &lidar_msg) {
  if (nullptr == lidar_msg) {
    return;
  }
  // parse msg
  if (lidar_msg->height() < 1 || lidar_msg->width() < 1) {
    AWARN << "Receive bad pointcloud:width-" << lidar_msg->width()
          << " ,height-" << lidar_msg->height() << " ,size-"
          << lidar_msg->point_size();
    return;
  } else if (lidar_msg->height() > 1 && lidar_msg->width() > 1) {
    ADEBUG << "Receive organized-pointcloud:width-" << lidar_msg->width()
           << " ,height-" << lidar_msg->height() << " ,size-"
           << lidar_msg->point_size();
  } else {
    ADEBUG << "Receive un-organized-pointcloud:width-" << lidar_msg->width()
           << " ,height-" << lidar_msg->height() << " ,size-"
           << lidar_msg->point_size();
  }

  double timestamp =
      message_parser_->GetMessageStamp(lidar_msg, config_.sync_time_thresh);
  AINFO << std::fixed << "## left lidar_msg timestamp(s): " << timestamp
        << " @host time: " << cyber::Time::Now().ToSecond();

  auto pcd_pcl_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  message_parser_->ParseLidarMsg(*lidar_msg, pcd_pcl_xyz);

#if SAVE_SHOW_DATA_LIDAR 
  auto pcd_pcl_xyzi = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  message_parser_->ParseLidarMsg(*lidar_msg, pcd_pcl_xyzi);
#endif

  {
    std::lock_guard<std::mutex> lock(left_lidar_buffer_mutex);
    LeftLidar_common_buffer_.emplace_back(
        std::make_pair(timestamp, pcd_pcl_xyz));

    while (LeftLidar_common_buffer_.size() > LIDAR_BUFFER_SIZE) {
      LeftLidar_common_buffer_.pop_front();
    }

#if SAVE_SHOW_DATA_LIDAR
    LeftLidar_common_buffer_show_.emplace_back(
        std::make_pair(timestamp, pcd_pcl_xyzi));

    while (LeftLidar_common_buffer_show_.size() > LIDAR_BUFFER_SIZE) {
      LeftLidar_common_buffer_show_.pop_front();
    }
#endif
  }
}

void LidarLocComponent::RightLidarCallback(
    const std::shared_ptr<drivers::PointCloudPacked> &lidar_msg) {
  if (nullptr == lidar_msg) {
    return;
  }

  if (lidar_msg->height() < 1 || lidar_msg->width() < 1) {
    AWARN << "Receive bad pointcloud:width-" << lidar_msg->width()
          << " ,height-" << lidar_msg->height() << " ,size-"
          << lidar_msg->point_size();
    return;
  } else if (lidar_msg->height() > 1 && lidar_msg->width() > 1) {
    ADEBUG << "Receive organized-pointcloud:width-" << lidar_msg->width()
           << " ,height-" << lidar_msg->height() << " ,size-"
           << lidar_msg->point_size();
  } else {
    ADEBUG << "Receive un-organized-pointcloud:width-" << lidar_msg->width()
           << " ,height-" << lidar_msg->height() << " ,size-"
           << lidar_msg->point_size();
  }

  double timestamp =
      message_parser_->GetMessageStamp(lidar_msg, config_.sync_time_thresh);
  AINFO << std::fixed << "## right lidar_msg timestamp(s): " << timestamp
        << " @host time: " << cyber::Time::Now().ToSecond();

  auto pcd_pcl_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  message_parser_->ParseLidarMsg(*lidar_msg, pcd_pcl_xyz);

#if SAVE_SHOW_DATA_LIDAR
  auto pcd_pcl_xyzi = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  message_parser_->ParseLidarMsg(*lidar_msg, pcd_pcl_xyzi);
#endif

  {
    std::lock_guard<std::mutex> lock(right_lidar_buffer_mutex);
    RightLidar_common_buffer_.emplace_back(
        std::make_pair(timestamp, pcd_pcl_xyz));

    while (RightLidar_common_buffer_.size() > LIDAR_BUFFER_SIZE) {
      RightLidar_common_buffer_.pop_front();
    }

#if SAVE_SHOW_DATA_LIDAR
    RightLidar_common_buffer_show_.emplace_back(
        std::make_pair(timestamp, pcd_pcl_xyzi));

    while (RightLidar_common_buffer_show_.size() > LIDAR_BUFFER_SIZE) {
      RightLidar_common_buffer_show_.pop_front();
    }
#endif
  }
  // right_lidar_runtime.EndCounter("RightLidarLoc");
}

void LidarLocComponent::ShmLeftLidarCallback(
    const std::shared_ptr<loc::CloudFrame> &shmlidar_msg) {
  if (nullptr == shmlidar_msg) {
    return;
  }

  timer_.StartCounter("ShmLeftLidar");

  static int get_index = 0;
  ++get_index;
  if (0 == (get_index % LIDAR_FRAME_INTERVAL_NUM)) {
    get_index = 0;
  } else {
    return;
  }

  // parse msg
  double timestamp = shmlidar_msg->timestamp_;
  AINFO << std::fixed << "## shmleftlidar_msg timestamp(s): " << timestamp;
  double measuretime = shmlidar_msg->measuretime_;
  AINFO << std::fixed << "## shmleftlidar_msg measuretime(s): " << measuretime;
  // TimeUs time = timestamp * kSecond;

  if (!CheckInputPcd(shmlidar_msg->cloud_ptr_)) {
    AERROR << "ShmLeftLidarCallback: CheckInputPcd failed!";
    return;
  }

  if (right_feature_extract_mutex_.try_lock()) {
    right_lidar_loc_thread_->enqueue(
        &LidarLocComponent::LeftLidarFeatureExtract, this, timestamp,
        shmlidar_msg->cloud_ptr_);
  }
}

void LidarLocComponent::ShmRightLidarCallback(
    const std::shared_ptr<loc::CloudFrame> &shmlidar_msg) {
  if (nullptr == shmlidar_msg) {
    return;
  }
  timer_.StartCounter("ShmRightLidar");

  static int get_index = 0;
  ++get_index;
  if (0 == (get_index % LIDAR_FRAME_INTERVAL_NUM)) {
    get_index = 0;
  } else {
    return;
  }

  // parse msg
  double timestamp = shmlidar_msg->timestamp_;
  AINFO << std::fixed << "## shmrightlidar_msg timestamp(s): " << timestamp;
  double measuretime = shmlidar_msg->measuretime_;
  AINFO << std::fixed << "## shmrightlidar_msg measuretime(s): " << measuretime;
  // TimeUs time = timestamp * kSecond;

  if (!CheckInputPcd(shmlidar_msg->cloud_ptr_)) {
    AERROR << "ShmLeftLidarCallback: CheckInputPcd failed!";
    return;
  }

  if (right_feature_extract_mutex_.try_lock()) {
    right_lidar_loc_thread_->enqueue(
        &LidarLocComponent::RightLidarFeatureExtract, this, timestamp,
        shmlidar_msg->cloud_ptr_);
  }
}

void LidarLocComponent::LidarMatcherTimerCallback() {
  std::pair<double, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
      left_lidar_frame, right_lidar_frame;

#if SAVE_SHOW_DATA_LIDAR
  std::pair<double, std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>
      left_lidar_frame_show, right_lidar_frame_show;
#endif

  {
    std::lock_guard<std::mutex> lock1(left_lidar_buffer_mutex);
    std::lock_guard<std::mutex> lock2(right_lidar_buffer_mutex);

    while (!LeftLidar_common_buffer_.empty() &&
           !RightLidar_common_buffer_.empty()) {
      if (LeftLidar_common_buffer_.front().first <
          RightLidar_common_buffer_.front().first - 0.01) {
        LeftLidar_common_buffer_.pop_front();
#if SAVE_SHOW_DATA_LIDAR
        LeftLidar_common_buffer_show_.pop_front();
#endif
      } else if (LeftLidar_common_buffer_.front().first >
                 RightLidar_common_buffer_.front().first + 0.01) {
        RightLidar_common_buffer_.pop_front();
#if SAVE_SHOW_DATA_LIDAR
        RightLidar_common_buffer_show_.pop_front();
#endif
      } else {
        break;
      }
    }

    if (LeftLidar_common_buffer_.empty() || RightLidar_common_buffer_.empty()) {
      AWARN << "LidarMatcherTimerCallback:buffer is empty!";
      return;
    }

    left_lidar_frame = LeftLidar_common_buffer_.front();
    right_lidar_frame = RightLidar_common_buffer_.front();
    LeftLidar_common_buffer_.pop_front();
    RightLidar_common_buffer_.pop_front();

#if SAVE_SHOW_DATA_LIDAR
    left_lidar_frame_show = LeftLidar_common_buffer_show_.front();
    right_lidar_frame_show = RightLidar_common_buffer_show_.front();
    LeftLidar_common_buffer_show_.pop_front();
    RightLidar_common_buffer_show_.pop_front();
#endif
  }

  auto left_transformed_frame =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto right_transformed_frame =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

#if SAVE_SHOW_DATA_LIDAR
  auto left_transformed_frame_show =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto right_transformed_frame_show =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
#endif

  double timestamp_s = left_lidar_frame.first;
  if (std::fabs(left_lidar_frame.first - right_lidar_frame.first) <
      0.01) {  // TODO
    pcl::transformPointCloud(*left_lidar_frame.second, *left_transformed_frame,
                             config_.Tx_veh_lidar1.getTransformationMatrix());
    pcl::transformPointCloud(*right_lidar_frame.second,
                             *right_transformed_frame,
                             config_.Tx_veh_lidar2.getTransformationMatrix());

#if SAVE_SHOW_DATA_LIDAR
    pcl::transformPointCloud(*left_lidar_frame_show.second, *left_transformed_frame_show,
                             config_.Tx_veh_lidar1.getTransformationMatrix());
    pcl::transformPointCloud(*right_lidar_frame_show.second,
                             *right_transformed_frame_show,
                             config_.Tx_veh_lidar2.getTransformationMatrix());
#endif
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_veh_frame(
      new pcl::PointCloud<pcl::PointXYZ>());
  *merged_veh_frame = *left_transformed_frame + *right_transformed_frame;

#if SAVE_SHOW_DATA_LIDAR
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_veh_frame_show(
      new pcl::PointCloud<pcl::PointXYZI>());
  *merged_veh_frame_show = *left_transformed_frame_show + *right_transformed_frame_show;
#endif

  // get initial guess
  mmath::SE3 guess_pose;
  if (!GetInitGuessPose(FromSeconds(timestamp_s), &guess_pose)) {
    static int cnt = 0;
    if (0 == ((++cnt) % 100)) {
      AINFO << "LidarMatcherTimerCallback: Waiting for initial guess pose...";
    }
    return;
  }
  
  static unsigned int frame_idx = 0;
  if (!lidar_locator_->IsInitialized()) {
    lidar_locator_->Init(Eigen::Affine3d(guess_pose.getTransformationMatrix()),
                         config_.resolution_id, config_.zone_id);
    return;
  }

  AINFO << "ndt time:" << std::fixed << left_lidar_frame.first << " " << right_lidar_frame.first;
  lidar_locator_->Update(frame_idx++,
                         Eigen::Affine3d(guess_pose.getTransformationMatrix()),
                         merged_veh_frame);

  auto lidar_pose_ = lidar_locator_->GetPose();
  auto fitness_score_ = lidar_locator_->GetFitnessScore();
  auto is_converged_ = lidar_locator_->HasConverged();
  PublishLidarPose(timestamp_s, mmath::SE3(lidar_pose_.matrix()),
                   fitness_score_, is_converged_);
          
#if SAVE_SHOW_DATA_LIDAR
  std::string filename = "/century/data/bag/match_data/" + std::to_string(timestamp_s) + ".pcd";
  pcl::io::savePCDFileBinary(filename, *merged_veh_frame_show);
#endif

  AINFO << std::fixed << "Publish lidar pose timestamp(s): " << timestamp_s
        << " host time: " << cyber::Time::Now().ToSecond();
}

void LidarLocComponent::LidarFrameTransfer(
    const std::shared_ptr<loc::PointCloudXYZIRT> &msg1,
    std::shared_ptr<loc::PointCloudXYZI> &msg2) {
  CHECK_NOTNULL(msg1);
  CHECK_NOTNULL(msg2);
  if (msg1->height > 1 && msg1->width > 1) {
    for (unsigned int i = 0; i < msg1->height; ++i) {
      for (unsigned int j = 0; j < msg1->width; ++j) {
        pcl::PointXYZI pt;
        pt.x = msg1->points[i * msg1->width + j].x;
        pt.y = msg1->points[i * msg1->width + j].y;
        pt.z = msg1->points[i * msg1->width + j].z;
        pt.intensity = msg1->points[i * msg1->width + j].intensity;
        msg2->points.emplace_back(pt);
      }
    }
  } else {
    // AINFO << "Receiving un-organized-point-cloud, width " << msg1->width
    //       << " height " << msg1->height << "size " << msg1->points.size();
    for (size_t i = 0; i < msg1->points.size(); ++i) {
      pcl::PointXYZI pt;
      pt.x = msg1->points[i].x;
      pt.y = msg1->points[i].y;
      pt.z = msg1->points[i].z;
      pt.intensity = msg1->points[i].intensity;
      msg2->points.emplace_back(pt);
    }
  }
}

void LidarLocComponent::LidarFrameTransfer(
    const std::shared_ptr<loc::PointCloudXYZI> &msg, LidarFrame *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  if (msg->height > 1 && msg->width > 1) {
    for (unsigned int i = 0; i < msg->height; ++i) {
      for (unsigned int j = 0; j < msg->width; ++j) {
        Eigen::Vector3f pt3d;
        pt3d[0] = msg->points[i * msg->width + j].x;
        pt3d[1] = msg->points[i * msg->width + j].y;
        pt3d[2] = msg->points[i * msg->width + j].z;
        if (!std::isnan(pt3d[0]) || !std::isnan(pt3d[1]) ||
            !std::isnan(pt3d[2])) {
          unsigned char intensity = static_cast<unsigned char>(
              msg->points[i * msg->width + j].intensity);
          lidar_frame->pt_xs.emplace_back(pt3d[0]);
          lidar_frame->pt_ys.emplace_back(pt3d[1]);
          lidar_frame->pt_zs.emplace_back(pt3d[2]);
          lidar_frame->intensities.emplace_back(intensity);
        }
      }
    }
  } else {
    // AINFO << "Receiving un-organized-point-cloud, width " << msg->width
    //       << " height " << msg->height << "size " << msg->points.size();
    for (size_t i = 0; i < msg->points.size(); ++i) {
      Eigen::Vector3f pt3d;
      pt3d[0] = msg->points[i].x;
      pt3d[1] = msg->points[i].y;
      pt3d[2] = msg->points[i].z;
      if (!std::isnan(pt3d[0]) || !std::isnan(pt3d[1]) ||
          !std::isnan(pt3d[2])) {
        unsigned char intensity =
            static_cast<unsigned char>(msg->points[i].intensity);
        lidar_frame->pt_xs.emplace_back(pt3d[0]);
        lidar_frame->pt_ys.emplace_back(pt3d[1]);
        lidar_frame->pt_zs.emplace_back(pt3d[2]);
        lidar_frame->intensities.emplace_back(intensity);
      }
    }
  }

  // lidar_frame->measurement_time =
  //     cyber::Time(msg->measurement_time()).ToSecond();
}

bool LidarLocComponent::ReadShmLeftLidarData() {
  const auto data = shm_cloud_left_->GetLatestMessage();
  auto pcd_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  static double timestamp_ = 0;
  if (std::abs(timestamp_ - data->timestamp) < 0.01) return false;
  timestamp_ = data->timestamp;
  if (nullptr != data) {
    for (uint32_t i = 0; i < data->points_num; ++i) {
      pcl::PointXYZ pt;
      pt.x = data->points[i].x;
      pt.y = data->points[i].y;
      pt.z = data->points[i].z;

      if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
        continue;
      }
      pcd_xyz->emplace_back(pt);
    }
  } else {
    AWARN << "ReadShmLeftLidarData failed";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(left_lidar_buffer_mutex);
    LeftLidar_common_buffer_.emplace_back(std::make_pair(timestamp_, pcd_xyz));

    while (LeftLidar_common_buffer_.size() > LIDAR_BUFFER_SIZE) {
      LeftLidar_common_buffer_.pop_front();
    }
  }
    AINFO << std::fixed << "##readshm left lidar_msg timestamp(s): " << timestamp_
        << " @host time: " << cyber::Time::Now().ToSecond();
  return true;
}

bool LidarLocComponent::ReadShmRightLidarData() {
  const auto data = shm_cloud_right_->GetLatestMessage();
  auto pcd_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  static double timestamp_ = 0;
  if (std::abs(timestamp_ - data->timestamp) < 0.01) return false;
  timestamp_ = data->timestamp;
  if (nullptr != data) {
    for (uint32_t i = 0; i < data->points_num; ++i) {
      pcl::PointXYZ pt;
      pt.x = data->points[i].x;
      pt.y = data->points[i].y;
      pt.z = data->points[i].z;

      if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
        continue;
      }
      pcd_xyz->emplace_back(pt);
    }
  } else {
    AWARN << "ReadShmRightLidarData failed";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(right_lidar_buffer_mutex);
    RightLidar_common_buffer_.emplace_back(std::make_pair(timestamp_, pcd_xyz));

    while (RightLidar_common_buffer_.size() > LIDAR_BUFFER_SIZE) {
      RightLidar_common_buffer_.pop_front();
    }
  }
  AINFO << std::fixed << "##read right lidar_msg timestamp(s): " << timestamp_
        << " @host time: " << cyber::Time::Now().ToSecond();
  return true;
}

bool LidarLocComponent::CheckInputPcd(
    const std::shared_ptr<loc::PointCloudXYZIRT> &pcd) {
  std::size_t pcd_size = pcd->size();
  if (pcd_size < PCD_SIZE_MIN) {
    AERROR << "Bad input pcd size:" << pcd_size;
    return false;
  }
  return true;
}
bool LidarLocComponent::GetInitGuessPose(const TimeUs time,
                                         mmath::SE3 *guess_pose) {
  if (!guess_pose) {
    return false;
  }
  std::unique_lock<std::mutex> lock(interpolater_mutex_);
  if (Tx_Mp_imu_interpolater_->evaluate(time, guess_pose)) {
    Eigen::Vector3d guess_trans = guess_pose->getTranslation();
    ADEBUG << "[lidar loc] guess_pose: " << guess_trans.transpose();
    guess_trans.x() += UTM_offset_x;
    guess_trans.y() += UTM_offset_y;
    if (!IsInMapRange(guess_trans)) {
      AWARN << "[lidar loc] guess_pose not in map: " << guess_trans.transpose();
      return false;
    }
    guess_pose->setTranslation(guess_trans);
    ADEBUG << "Get Guess Pose at: " << guess_pose->log().transpose();
  } else {
    return false;
  }
  return true;
}

bool LidarLocComponent::LoadZoneIdFromFolder(const std::string &folder_path,
                                             int *zone_id) {
  std::string map_zone_id_folder;
  if (cyber::common::DirectoryExists(folder_path + "/map/000/north")) {
    map_zone_id_folder = folder_path + "/map/000/north";
  } else if (cyber::common::DirectoryExists(folder_path + "/map/000/south")) {
    map_zone_id_folder = folder_path + "/map/000/south";
  } else {
    return false;
  }

  auto folder_list = cyber::common::ListSubPaths(map_zone_id_folder);
  for (auto itr = folder_list.begin(); itr != folder_list.end(); ++itr) {
    *zone_id = std::stoi(*itr);
    return true;
  }
  return false;
}
bool LidarLocComponent::IsInMapRange(const Eigen::Vector3d &pose) {
  return pose.x() >= ndt_map_config_.map_min_x &&
         pose.y() >= ndt_map_config_.map_min_y &&
         pose.x() <= ndt_map_config_.map_max_x &&
         pose.y() <= ndt_map_config_.map_max_y;
}
void LidarLocComponent::PublishLidarPose(const double timestamp,
                                         const mmath::SE3 &Tx_Mp_L,
                                         const double fitness_score,
                                         const bool is_converged) {
  localization::PoseWithCov pose_msg;
  std::string frame_id = "base_link";
  Eigen::Vector3d trans = Tx_Mp_L.getTranslation();
  trans.x() -= UTM_offset_x;
  trans.y() -= UTM_offset_y;
  mmath::SE3 Tx_Mp_L_relative(Tx_Mp_L.getSO3(), trans);
  message_parser_->GetLidarPoseMsg(timestamp, frame_id, Tx_Mp_L_relative,
                                   fitness_score, is_converged, &pose_msg);
                                   
  AINFO << "LidarPose---->" << std::fixed << timestamp << " " 
        << Tx_Mp_L_relative.getTranslation().x() << " "
        << Tx_Mp_L_relative.getTranslation().y() << " "
        << Tx_Mp_L_relative.getTranslation().z() << " "
        << Tx_Mp_L_relative.getSO3().getEulerYPR().z() * mmath::kRadToDeg << " "
        << Tx_Mp_L_relative.getSO3().getEulerYPR().y() * mmath::kRadToDeg << " "
        << Tx_Mp_L_relative.getSO3().getEulerYPR().x() * mmath::kRadToDeg;

  lidar_pose_talker_->Write(pose_msg);

#if SAVE_SHOW_DATA_LIDAR
  if (!lidar_pose_file_ || !lidar_pose_file_->is_open()) {
      return;
  }

  LidarPoseData lidar_pose_data;
  lidar_pose_data.timestamp = timestamp;
  lidar_pose_data.x = Tx_Mp_L_relative.getTranslation().x();
  lidar_pose_data.y = Tx_Mp_L_relative.getTranslation().y();
  lidar_pose_data.z = Tx_Mp_L_relative.getTranslation().z();
  lidar_pose_data.roll = Tx_Mp_L_relative.getSO3().getEulerYPR().z();
  lidar_pose_data.pitch = Tx_Mp_L_relative.getSO3().getEulerYPR().y();
  lidar_pose_data.yaw = Tx_Mp_L_relative.getSO3().getEulerYPR().x();
  lidar_pose_file_->write(reinterpret_cast<const char*>(&lidar_pose_data), sizeof(LidarPoseData));
  lidar_pose_file_->flush();
#endif
}
}  // namespace loc
}  // namespace century
