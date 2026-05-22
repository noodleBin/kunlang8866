/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/onboard/component/lidar_segmentation_component.h"

#include <filesystem>
#include <string>

#include "cyber/time/clock.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/common/timer_util.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/pcl_util.h"

namespace fs = std::filesystem;

using ::century::cyber::Clock;

namespace century {
namespace perception {
namespace onboard {

std::atomic<uint32_t> LidarSegmentationComponent::seq_num_{0};

void LidarSegmentationComponent::DebugPCD(
    const std::string& base_path,
    const std::shared_ptr<PointCloudInType>& message) noexcept {
  std::string sensor_name = sensor_name_;
  std::string prefix = "lidar_";

  if (sensor_name.rfind(prefix, 0) == 0) {
    sensor_name = sensor_name.substr(prefix.length());
  }

  std::string dir_path = base_path + sensor_name;

  std::regex pattern(R"(1746599663\.\d+\.pcd)");
  std::vector<std::string> matched_files;

  for (const auto& entry : fs::directory_iterator(dir_path)) {
    const auto& path = entry.path();
    if (fs::is_regular_file(path)) {
      std::string filename = path.filename().string();
      if (std::regex_match(filename, pattern)) {
        matched_files.push_back(path.string());
      }
    }
  }

  if (matched_files.empty()) {
    std::cerr << "No matching PCD files found." << std::endl;
    return;
  }

  std::string pcd_file = matched_files[0];
  std::cout << "Loading PCD file: " << pcd_file << std::endl;

  pcl::PointCloud<lidar::PointXYZIR>::Ptr cloud(
      new pcl::PointCloud<lidar::PointXYZIR>);
  if (pcl::io::loadPCDFile<lidar::PointXYZIR>(pcd_file, *cloud) == -1) {
    std::cerr << "Failed to load PCD file!" << std::endl;
    return;
  }
}

bool LidarSegmentationComponent::Init() {
  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get config failed";
    return false;
  }
  auto output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  detector_name_ = comp_config.detector_name();

  lidar2imu_tf2_child_frame_id_ =
      comp_config.lidar2novatel_tf2_child_frame_id();
  enable_hdmap_ = comp_config.enable_hdmap();
  AERROR << ", lidar2imu_tf2_child_frame_id_: "
         << lidar2imu_tf2_child_frame_id_;

  lidar::BasePointCloudPreprocessor* preprocessor =
      lidar::BasePointCloudPreprocessorRegisterer::GetInstanceByName(
          detector_name_);
  CHECK_NOTNULL(preprocessor);
  ground_preprocessor_.reset(preprocessor);

  preprocessor_init_options_.sensor_name = sensor_name_;
  ACHECK(ground_preprocessor_->Init(preprocessor_init_options_))
      << "lidar preprocessor init error";

  lidar2vehicle_trans_.Init(lidar2imu_tf2_child_frame_id_);

  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  seg_pointcloud_writer_ = node_->CreateWriter<drivers::PointCloud>(
      output_channel_name_ + "/debug/seg_pointcloud");

  fps_counter_ = std::make_unique<FpsCounter>();

  AERROR << "\n Lidar Component Configs: " << comp_config.DebugString()
         << "\n sensor_name: " << sensor_name_
         << "\n detector_name: " << detector_name_
         << "\n output_channel_name: " << output_channel_name_
         << "\n thread id: " << std::this_thread::get_id();
  return true;
}

bool LidarSegmentationComponent::Proc(
    const std::shared_ptr<PointCloudInType>& message) {
  auto out_message = std::make_shared<LidarFrameMessage>();
  auto flag = false;
  if (flag) {
    std::string base_path =
        "/century/data/test_infer_pillar/"
        "20250507_duichang_3/lidar/";
    DebugPCD(base_path, message);
  }

  PERCEPTION_PERF_BLOCK_START();
  if (InternalProc(message, out_message)) {
    PublishSegmentedPointCloud(out_message);
  }
  PERCEPTION_PERF_BLOCK_END("PointCloud Segmentation");

  return true;
}

bool LidarSegmentationComponent::InitAlgorithmPlugin() {
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,
                                                          &sensor_info_));
  return true;
}

bool LidarSegmentationComponent::InternalProc(
    const std::shared_ptr<const PointCloudInType>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  uint32_t seq_num = seq_num_.fetch_add(1);
  const double timestamp = in_message->header().timestamp_sec();
  const double cur_time = Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  const int fps = static_cast<int>(fps_counter_->GetFps());
  AINFO << std::fixed << std::setprecision(16)
        << "FRAME_STATISTICS:Lidar:Start:msg_time[" << timestamp << "]:sensor["
        << sensor_name_ << "]:cur_time[" << cur_time << "]:cur_latency["
        << start_latency << "], fps: " << fps;

  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().timestamp_sec();
  out_message->seq_num_ = seq_num;
  out_message->process_stage_ = ProcessStage::UNKNOWN_STAGE;
  out_message->error_code_ = century::common::ErrorCode::OK;

  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;

  if (!is_query_lidar_pose_) {
    if (!QueryLidarPoseInfo(&pose_info_)) {
      AERROR << "Failed to obtain lidar extrinsics";
      return false;
    }
    is_query_lidar_pose_ = true;
  }

  frame->lidar2world_pose = pose_info_.lidar2vehicle;
  frame->novatel2world_pose = pose_info_.lidar2vehicle;
  frame->vehicle2imu_pose = pose_info_.imu2vehicle;

  lidar::PointCloudPreprocessorOptions preprocessor_options;
  preprocessor_options.sensor2novatel_extrinsics = frame->lidar2world_pose;
  preprocessor_options.sensor2vehicle_extrinsics = pose_info_.lidar2imu;

  auto flag = ground_preprocessor_->Preprocess(preprocessor_options, in_message,
                                               frame.get());
  if (!flag) {
    AERROR << "Failed to preprocess point cloud: " << sensor_name_;
    return false;
  }

  writer_->Write(out_message);
  AINFO << "Send lidar detect output message: " << sensor_name_ << ": "
        << frame->raw_cloud->size();

  return true;
}

void LidarSegmentationComponent::PublishSegmentedPointCloud(
    const std::shared_ptr<LidarFrameMessage>& out_message) noexcept {
  if (!out_message) {
    return;
  }
  drivers::PointCloud seg_point_clouds;
  seg_point_clouds.mutable_header()->set_sequence_num(out_message->seq_num_);
  seg_point_clouds.mutable_header()->set_timestamp_sec(out_message->timestamp_);
  seg_point_clouds.set_measurement_time(cyber::Time::Now().ToSecond());

  seg_pointcloud_writer_->Write(seg_point_clouds);
}

bool LidarSegmentationComponent::QueryLidarPoseInfo(
    LidarPoseInfo* pose_info) noexcept {
  if (!pose_info) {
    return false;
  }

  Eigen::Matrix4d lidar2imu;
  if (!lidar2vehicle_trans_.QueryStaticTF("imu", sensor_name_, &lidar2imu)) {
    AERROR << "Failed to query lidar2imu TF: " << sensor_name_;
    return false;
  }

  Eigen::Matrix4d imu2vehicle;
  if (!lidar2vehicle_trans_.QueryStaticTF("vehicle", "imu", &imu2vehicle)) {
    AERROR << "Failed to query imu2vehicle TF";
    return false;
  }

  Eigen::Matrix4d lidar2vehicle_mat = imu2vehicle * lidar2imu;

  if (lidar2vehicle_mat.hasNaN()) {
    AERROR << "Computed lidar2vehicle has NaN values!\n"
           << "lidar2imu:\n"
           << lidar2imu << "\n"
           << "imu2vehicle:\n"
           << imu2vehicle;
    return false;
  }

  pose_info->lidar2vehicle = Eigen::Affine3d(lidar2vehicle_mat);
  pose_info->imu2vehicle = Eigen::Affine3d(imu2vehicle);
  pose_info->lidar2imu = Eigen::Affine3d(lidar2imu);
  pose_info->novatel2world = pose_info->lidar2vehicle;

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace century
