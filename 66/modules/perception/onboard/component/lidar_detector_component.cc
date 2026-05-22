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

#include "modules/perception/onboard/component/lidar_detector_component.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "sys/time.h"

#include "cyber/base/thread_pool.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/base/singleton.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/common/timer_util.h"
#include "modules/perception/lidar/common/duplicated_object_filter.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_util.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"

using Clock = century::cyber::Clock;

namespace century {
namespace perception {
namespace onboard {

using century::cyber::Rate;
using century::cyber::Time;

using google::protobuf::Closure;
using google::protobuf::NewCallback;

namespace {
constexpr int kTaskPoolSize = 2;
constexpr int kThreadPoolSize = 4;
constexpr int kQueueSize = 2;
}  // namespace

LidarDetectorComponent::~LidarDetectorComponent() {
  pointcloud_msg_queue_.reset();
  merged_msg_queue_.reset();
  around_vehicle_queue_.reset();
}

bool LidarDetectorComponent::Init() {
  thread_pool_ = std::make_unique<lib::ThreadPool>(kThreadPoolSize);
  thread_pool_->Start();

  base::Singleton<std::mutex>::Instance();

  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get config failed";
    return false;
  }
  AINFO << "Perception Version: " << comp_config.version();
  AINFO << "Lidar Component Configs: " << comp_config.DebugString();
  use_viz_debug_ = comp_config.use_viz_debug();
  use_filter_bank_ = comp_config.use_filter_bank();
  use_pose_query_ = comp_config.use_pose_query();
  pose_query_offset_ = comp_config.pose_query_offset();
  use_point_interpolation_ = comp_config.use_point_interpolation();
  detector_name_ = comp_config.detector_name();
  cluster_name_ = comp_config.cluster_name();
  sensor_name_ = comp_config.sensor_name();
  enable_hdmap_ = comp_config.enable_hdmap();
  output_channel_name_ = comp_config.output_channel_name();
  around_ego_output_channel_name_ =
      comp_config.around_ego_output_channel_name();
  debug_output_channel_name_ = comp_config.debug_output_channel_name();
  debug_around_output_channel_name_ =
      comp_config.debug_around_output_channel_name();
  if (output_channel_name_.empty()) {
    AERROR << "output_channel_name is empty in lidar component config.";
    return false;
  }
  if (around_ego_output_channel_name_.empty()) {
    AERROR << "around_ego_output_channel_name is empty in lidar component "
           << "config.";
    return false;
  }
  if (debug_output_channel_name_.empty()) {
    AERROR << "debug_output_channel_name is empty in lidar component config.";
    return false;
  }
  if (debug_around_output_channel_name_.empty()) {
    AERROR << "debug_around_output_channel_name is empty in lidar component "
           << "config.";
    return false;
  }
  if (comp_config.localization_topic().empty()) {
    AERROR << "localization_topic is empty in lidar component config.";
    return false;
  }
  perception_writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_channel_name_);

  around_ego_perception_writer_ =
      node_->CreateWriter<PerceptionObstacles>(around_ego_output_channel_name_);
  perception_debug_writer_ = node_->CreateWriter<PerceptionObstacleDebugMsg>(
      debug_output_channel_name_);

  perception_debug_around_writer_ =
      node_->CreateWriter<PerceptionObstacleDebugMsg>(
          debug_around_output_channel_name_);

  auto& input_channel = comp_config.input_channel_name();
  lidar_readers_.reserve(input_channel.size());
  for (const auto& channel : input_channel) {
    input_channels_.push_back(channel);
    auto reader = node_->CreateReader<LidarFrameMessage>(channel);
    lidar_readers_.push_back(reader);
  }
  lidar2vehicle_trans_.Init("imu");

  pointcloud_msg_queue_ =
      std::make_shared<lib::FixedSizeConQueue<LidarMsgsVecPtr>>(kQueueSize);

  merged_msg_queue_ =
      std::make_shared<lib::FixedSizeConQueue<LidarFrameMessagePtr>>(
          kQueueSize);

  around_vehicle_queue_ =
      std::make_shared<lib::FixedSizeConQueue<LidarFrameMessagePtr>>(
          kQueueSize);

  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          comp_config.localization_topic());
  localization_reader_->SetHistoryDepth(250);
  use_map_manager_ = comp_config.use_map_manager() && enable_hdmap_;

  lidar::SceneManagerInitOptions scene_manager_init_options;
  ACHECK(lidar::SceneManager::Instance().Init(scene_manager_init_options));

  auto ret = InitAlgorithmPlugin();
  if (!ret) {
    AERROR << "Init algorithm plugin failed.";
    return false;
  }

  lidar::ObjectBuilderInitOptions builder_init_options;
  ACHECK(builder_.Init(builder_init_options));

  Closure* receive_msg_closure =
      NewCallback(this, &LidarDetectorComponent::StartReceiveMsg);
  Closure* process_lidar_msg_closure =
      NewCallback(this, &LidarDetectorComponent::ProcessLidarFrameMessage);
  Closure* sync_lidar_msg_closure =
      NewCallback(this, &LidarDetectorComponent::SyncPointCloudMsgTask);

  Closure* process_lidar_msg_around_vehicle_closure = NewCallback(
      this, &LidarDetectorComponent::ProcessLidarFrameAroundVehicle);

  thread_pool_->Add(receive_msg_closure);
  thread_pool_->Add(process_lidar_msg_closure);
  thread_pool_->Add(sync_lidar_msg_closure);
  thread_pool_->Add(process_lidar_msg_around_vehicle_closure);

  return true;
}
void LidarDetectorComponent::DumpWorldPcl(
    const LidarFrameMessagePtr& raw_cloud) {
  std::string folder_name =
      FLAGS_dump_world_pcl + "/" + std::to_string(raw_cloud->timestamp_);
  std::replace(folder_name.begin(), folder_name.end(), '.', '_');
  folder_name = folder_name + ".pcd";
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto& item : raw_cloud->lidar_frame_->raw_cloud->points()) {
    cloud->points.emplace_back(item.x, item.y, item.z, item.intensity);
  }
  cloud->width = raw_cloud->lidar_frame_->raw_cloud->points().size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (pcl::io::savePCDFileASCII(folder_name, *cloud) != 0) {
    AERROR << "Failed to save " << folder_name << std::endl;
  }

  return;
}
void LidarDetectorComponent::DumpWorldPcl(
    const double ts, const base::PointFCloudPtr& vehicle_cloud,
    const Eigen::Affine3d& map_vehicle_pose) const noexcept {
  if (FLAGS_dump_world_pcl.empty()) {
    return;
  }

  std::string folder_name = FLAGS_dump_world_pcl + std::to_string(ts);
  std::replace(folder_name.begin(), folder_name.end(), '.', '_');
  struct stat info;
  if (stat(folder_name.c_str(), &info) != 0) {
    if (mkdir(folder_name.c_str(), 0755) != 0) {
      AERROR << "Failed to create folder: " << folder_name << std::endl;
      return;
    }
  }

  std::ofstream log_file(folder_name + "/pcl.csv");
  if (log_file.is_open()) {
    for (const auto& item : vehicle_cloud->points()) {
      auto w_point = map_vehicle_pose * Eigen::Vector3d(item.x, item.y, item.z);
      log_file << std::fixed << w_point(0) << "," << w_point(1) << ","
               << w_point(2) << std::endl;
    }
    log_file.close();
  }
}

LidarFrameMessagePtr LidarDetectorComponent::SyncAndMergePointCloudMessages(
    double tolerance) noexcept {
  std::vector<LidarFrameMessagePtr> synchronized_msgs;
  LidarMsgsVecPtr msg_ptr;

  pointcloud_msg_queue_->Pop(&msg_ptr);
  if (!msg_ptr || msg_ptr->empty()) {
    AERROR << "No messages to synchronize.";
    return nullptr;
  }

  for (const auto& msg : *msg_ptr) {
    if (!msg || !msg->lidar_frame_) {
      AERROR << "Invalid message or cloud in synchronized messages.";
      continue;
    }
    synchronized_msgs.push_back(msg);
  }

  // Find the earliest timestamp among all queues
  double earliest_timestamp = std::numeric_limits<double>::max();
  for (const auto& msg_item : synchronized_msgs) {
    earliest_timestamp = std::min(earliest_timestamp, msg_item->timestamp_);
  }

  // Check if we have valid timestamps
  if (earliest_timestamp == std::numeric_limits<double>::max()) {
    AERROR << "No valid messages in the queues: " << synchronized_msgs.size();
    return nullptr;
  }

  for (auto& msg : synchronized_msgs) {
    double time_diff = std::abs(msg->timestamp_ - earliest_timestamp);
    if (time_diff <= tolerance) {
    } else {
      AWARN << " msg out of sync (time diff: " << time_diff << ").";
      return nullptr;
    }
  }

  // Check if all channels are synchronized
  if (synchronized_msgs.size() != lidar_readers_.size()) {
    AERROR << "Not all channels are synchronized. Expected: "
           << ", Got: " << synchronized_msgs.size();
    return nullptr;
  }

  // Merge synchronized messages
  auto merged_message = std::make_shared<LidarFrameMessage>();
  if (!merged_message->lidar_frame_) {
    merged_message->lidar_frame_ = lidar::LidarFramePool::Instance().Get();

    merged_message->lidar_frame_->cloud =
        base::PointFCloudPool::Instance().Get();

    merged_message->lidar_frame_->raw_cloud =
        base::PointFCloudPool::Instance().Get();

    merged_message->lidar_frame_->ego_cloud =
        base::PointFCloudPool::Instance().Get();

    merged_message->lidar_frame_->world_cloud =
        base::PointDCloudPool::Instance().Get();
  }

  for (auto& msg : synchronized_msgs) {
    if (!msg || !msg->lidar_frame_) {
      AERROR << "Invalid message or cloud in synchronized messages.";
      continue;
    }
    auto non_ground_indices = msg->lidar_frame_->non_ground_indices.indices;
    auto points_num = merged_message->lidar_frame_->raw_cloud->points().size();

    for (const auto& index : non_ground_indices) {
      merged_message->lidar_frame_->non_ground_indices.indices.push_back(
          points_num + index);
    }

    // Merge point cloud data
    *merged_message->lidar_frame_->cloud += *msg->lidar_frame_->cloud;

    *merged_message->lidar_frame_->raw_cloud += *msg->lidar_frame_->raw_cloud;
    *merged_message->lidar_frame_->ego_cloud += *msg->lidar_frame_->ego_cloud;

    // Merge segmented objects
    merged_message->lidar_frame_->segmented_objects.insert(
        merged_message->lidar_frame_->segmented_objects.end(),
        msg->lidar_frame_->segmented_objects.begin(),
        msg->lidar_frame_->segmented_objects.end());
  }

  merged_message->lidar_frame_->raw_cloud->set_timestamp(
      synchronized_msgs[0]->lidar_frame_->raw_cloud->get_timestamp());

  merged_message->lidar_frame_->cloud->set_timestamp(
      synchronized_msgs[0]->lidar_frame_->cloud->get_timestamp());

  merged_message->lidar_frame_->ego_cloud->set_timestamp(
      synchronized_msgs[0]->lidar_frame_->ego_cloud->get_timestamp());
  // Copy additional metadata from the first synchronized message
  merged_message->lidar_frame_->hdmap_struct =
      synchronized_msgs[0]->lidar_frame_->hdmap_struct;

  merged_message->timestamp_ = earliest_timestamp;
  merged_message->lidar_timestamp_ = synchronized_msgs[0]->lidar_timestamp_;
  merged_message->seq_num_ = synchronized_msgs[0]->seq_num_;
  merged_message->type_name_ = synchronized_msgs[0]->type_name_;
  merged_message->process_stage_ = synchronized_msgs[0]->process_stage_;
  merged_message->error_code_ = synchronized_msgs[0]->error_code_;

  AINFO << "Merged point cloud size: "
        << merged_message->lidar_frame_->raw_cloud->size() << ", from "
        << synchronized_msgs.size() << " sensors.";

  return merged_message;
}

void LidarDetectorComponent::SyncPointCloudMsgTask() noexcept {
  const double kTolerance = 0.1;
  while (century::cyber::OK()) {
    auto merged_msg = SyncAndMergePointCloudMessages(kTolerance);
    if (!merged_msg) {
      continue;
    }

    merged_msg_queue_->PushBack(merged_msg);
  }
}

void LidarDetectorComponent::ProcessLidarFrameAroundVehicle() noexcept {
  while (century::cyber::OK()) {
    LidarFrameMessagePtr frame_msg;
    around_vehicle_queue_->Pop(&frame_msg);

    if (!frame_msg || !frame_msg->lidar_frame_) {
      AERROR << "Invalid frame message or lidar frame.";
      continue;
    }

    auto& segmented_objects = frame_msg->lidar_frame_->segmented_objects;

    segmented_objects.erase(
        std::remove_if(segmented_objects.begin(), segmented_objects.end(),
                       [](const base::ObjectPtr& obj) {
                         return obj->type == base::ObjectType::UNKNOWN;
                       }),
        segmented_objects.end());

    std::vector<std::shared_ptr<base::Object>> det_objects = segmented_objects;

    lidar::ClusterOptions cluster_options;
    cluster_options.enable_hdmap_input = false;

    PERCEPTION_PERF_BLOCK_START();

    frame_msg->lidar_frame_->cloud = frame_msg->lidar_frame_->ego_cloud;

    cluster_processor_->Cluster(cluster_options, frame_msg->lidar_frame_.get());

    lidar::ObjectBuilderOptions builder_options;
    if (!builder_.Build(builder_options, frame_msg->lidar_frame_.get())) {
      AERROR << "Failed to build objects.";
      continue;
    }

    if (perception_debug_around_writer_->HasReader() && use_viz_debug_) {
      const auto& localization_msg = localization_reader_->GetLatestObserved();
      if (!localization_msg) {
        AERROR << "No localization message received.";
        continue;
      }
      PublishPerceptionObstacles(perception_debug_around_writer_, frame_msg,
                                 localization_msg->header().timestamp_sec());
    }

    frame_msg->lidar_frame_->segmented_objects.insert(
        frame_msg->lidar_frame_->segmented_objects.end(), det_objects.begin(),
        det_objects.end());
    PERCEPTION_PERF_BLOCK_END("Around Vehicle PointCloud Cluster");

    float duplicated_distance_thres = 1.0f;
    lidar::DuplicatedObjectFilter duplicate_filter(duplicated_distance_thres);

    for (const auto& iter : frame_msg->lidar_frame_->segmented_objects) {
      if (iter->type != base::ObjectType::UNKNOWN) {
        duplicate_filter.AddPoint(
            Eigen::Vector3d(iter->center(0), iter->center(1), iter->theta),
            iter->size(0), iter->size(1),
            iter->type == base::ObjectType::PEDESTRIAN);
      }
    }

    auto& vec = frame_msg->lidar_frame_->segmented_objects;
    vec.erase(
        std::remove_if(vec.begin(), vec.end(),
                       [&](century::perception::base::ObjectPtr iter) {
                         return ((iter->type == base::ObjectType::UNKNOWN) &&
                                 (duplicate_filter.IsDuplicated(Eigen::Vector2d(
                                     iter->center(0), iter->center(1)))));
                       }),
        vec.end());

    auto out_message = std::make_shared<SensorFrameMessage>();
    if (InternalProc(frame_msg, out_message)) {
      auto perception_obstacles = std::make_shared<PerceptionObstacles>();
      if (!MsgSerializer::SerializeMsg(
              out_message->timestamp_, out_message->lidar_timestamp_,
              out_message->seq_num_, out_message->frame_->objects,
              out_message->error_code_, perception_obstacles.get())) {
        AERROR << "Failed to serialize PerceptionObstacles object.";
        continue;
      }

      static int seq_num = 0;
      for (int i = 0; i < perception_obstacles->perception_obstacle_size();
           ++i) {
        perception_obstacles->mutable_perception_obstacle(i)->set_id(seq_num++);
      }

      auto header = perception_obstacles->mutable_header();
      header->set_timestamp_sec(frame_msg->timestamp_);

      auto& lidar2world_pose = frame_msg->lidar_frame_->lidar2world_pose;
      century::perception::Affine3dProto lidar2world_pose_proto;
      const auto& translation = lidar2world_pose.translation();
      lidar2world_pose_proto.set_tx(translation.x());
      lidar2world_pose_proto.set_ty(translation.y());
      lidar2world_pose_proto.set_tz(translation.z());

      Eigen::Quaterniond quat(lidar2world_pose.linear());
      lidar2world_pose_proto.set_qw(quat.w());
      lidar2world_pose_proto.set_qx(quat.x());
      lidar2world_pose_proto.set_qy(quat.y());
      lidar2world_pose_proto.set_qz(quat.z());
      perception_obstacles->mutable_lidar2world()->CopyFrom(
          lidar2world_pose_proto);
      around_ego_perception_writer_->Write(perception_obstacles);
    } else {
      AERROR << "Failed to process lidar frame message.";
    }
  }
}

void LidarDetectorComponent::StartReceiveMsg() noexcept {
  constexpr double kOutputRate = 20.0;
  Rate rate(kOutputRate);

  auto block_read_lidar_msg = [&](auto reader, auto msg_vec) -> bool {
    int retry_count = 20;
    while (retry_count--) {
      reader->Observe();
      auto lidar_frame = reader->GetLatestObserved();
      if (lidar_frame) {
        msg_vec->emplace_back(lidar_frame);
        reader->ClearData();
        AINFO << "Received message: "
              << " detector_name_: " << detector_name_ << std::fixed
              << std::setprecision(6)
              << ", topic name: " << reader->GetChannelName() << ", "
              << lidar_frame->timestamp_ << ", "
              << lidar_frame->lidar_frame_->cloud->size() << " points."
              << std::endl;
        return true;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }

    return false;
  };

  while (century::cyber::OK()) {
    LidarMsgsVecPtr msg_ptr =
        std::make_shared<std::vector<LidarFrameMessagePtr>>();
    msg_ptr->reserve(lidar_readers_.size());
    bool is_save{true};
    int count = 0;

    for (const auto& reader : lidar_readers_) {
      is_save = block_read_lidar_msg(reader, msg_ptr);
      if (!is_save) {
        AERROR << "No message received." << input_channels_[count] << std::endl;
        break;
      }
      count++;
    }

    if (is_save) {
      pointcloud_msg_queue_->PushBack(msg_ptr);
    }
  }
}

void LidarDetectorComponent::ProcessLidarFrameMessage() noexcept {
  while (century::cyber::OK()) {
    LidarFrameMessagePtr merged_msg;
    merged_msg_queue_->Pop(&merged_msg);
    PERCEPTION_PERF_BLOCK_START();
    localization_reader_->Observe();
    const auto& localization_msg = localization_reader_->GetLatestObserved();
    if (!localization_msg) {
      AERROR << "No localization message received.";
      continue;
    }

    Eigen::Matrix4d imu2vehicle_matrix;
    lidar2vehicle_trans_.QueryStaticTF("vehicle", "imu", &imu2vehicle_matrix);
    auto loc_begin = localization_reader_->Begin();
    auto loc_end = localization_reader_->End();
    merged_msg->lidar_frame_->localizetion_poses.clear();
    double last_timestamp = -1.0;
    for (auto& it = loc_begin; it != loc_end; ++it) {
      auto& world_pose = (*it)->pose();
      Eigen::Affine3d lidar2world_pose = Eigen::Affine3d();
      Eigen::Quaterniond vehicle2world_quaternion(
          world_pose.orientation().qw(), world_pose.orientation().qx(),
          world_pose.orientation().qy(), world_pose.orientation().qz());
      Eigen::Vector3d vehicle2world_translate(world_pose.position().x(),
                                              world_pose.position().y(),
                                              world_pose.position().z());
      lidar2world_pose.linear() = vehicle2world_quaternion.toRotationMatrix();
      lidar2world_pose.translation() = vehicle2world_translate;
      if (last_timestamp > 0 &&
          (*it)->header().timestamp_sec() >= last_timestamp) {
        AERROR << std::fixed << std::setprecision(6)
               << "skip duplicate timestamp: "
               << (*it)->header().timestamp_sec() << " : " << last_timestamp
               << std::endl;
        continue;
      }
      last_timestamp = (*it)->header().timestamp_sec();
      merged_msg->lidar_frame_->localizetion_poses.emplace_back(
          std::make_pair((*it)->header().timestamp_sec(),
                         lidar2world_pose * imu2vehicle_matrix));
    }
    AINFO << "localization size: "
          << merged_msg->lidar_frame_->localizetion_poses.size() << std::endl;

    auto& frame_timestamp = merged_msg->timestamp_;
    auto frame_timestamp_query = frame_timestamp + pose_query_offset_;
    Eigen::Affine3d frame_pose = Eigen::Affine3d::Identity();
    double pose_time = 0.0;
    if (use_pose_query_) {
      if (!lidar::GetNearest(merged_msg->lidar_frame_->localizetion_poses,
                             frame_timestamp_query, frame_pose, pose_time,
                             false)) {
        UpdateLidarPose(merged_msg, localization_msg);
      } else {
        AINFO << std::fixed << std::setprecision(6)
              << "USE INTER POSE: " << frame_timestamp_query << " : "
              << pose_time;
        merged_msg->lidar_frame_->lidar2world_pose =
            frame_pose * imu2vehicle_matrix;
        int points_num = merged_msg->lidar_frame_->cloud->size();
        if (use_point_interpolation_) {
          for (size_t i = 0; i < points_num; ++i) {
            auto& point =
                merged_msg->lidar_frame_->raw_cloud->mutable_points()->at(i);
            Eigen::Vector3d vec3d_lidar(point.x, point.y, point.z);
            auto point_timestamp =
                merged_msg->lidar_frame_->raw_cloud->points_timestamp().at(i);

            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            double point_pose_time = 0.0;
            if (!lidar::GetNearest(merged_msg->lidar_frame_->localizetion_poses,
                                   point_timestamp, pose, point_pose_time,
                                   false)) {
              AERROR << std::fixed << std::setprecision(6)
                     << "Failed to get point pose at time: " << point_timestamp
                     << " : " << point_pose_time;
            }
            Eigen::Vector3d vec3d_world =
                pose * Eigen::Affine3d(imu2vehicle_matrix) * vec3d_lidar;
            base::PointD world_point;
            world_point.x = vec3d_world(0);
            world_point.y = vec3d_world(1);
            world_point.z = vec3d_world(2);
            Eigen::Vector3d vec3d_local =
                Eigen::Affine3d(imu2vehicle_matrix.inverse()) *
                (frame_pose.inverse() * vec3d_world);
            point.x = vec3d_local(0);
            point.y = vec3d_local(1);
            point.z = vec3d_local(2);
          }
          merged_msg->lidar_frame_->cloud->clear();
          merged_msg->lidar_frame_->cloud->CopyPointCloud(
              *merged_msg->lidar_frame_->raw_cloud,
              merged_msg->lidar_frame_->non_ground_indices);
        }
      }
    } else {
      UpdateLidarPose(merged_msg, localization_msg);
    }

    merged_msg->lidar_frame_->segmented_objects.clear();

    auto detector_future = std::async(
        std::launch::async,
        std::bind(&LidarDetectorComponent::Detect, this, merged_msg));

    auto cluster_future = std::async(
        std::launch::async,
        std::bind(&LidarDetectorComponent::Cluster, this, merged_msg));
    detector_future.get();
    cluster_future.get();

    PERCEPTION_PERF_BLOCK_END("start PointCloud Cluster");

    RemoveHighTrailerObjects(merged_msg);

    PERCEPTION_PERF_BLOCK_END("End PointCloud Cluster");

    if (use_filter_bank_) {
      lidar::ObjectFilterOptions filter_options;
      filter_bank_.Filter(filter_options, merged_msg->lidar_frame_.get());
    }

    lidar::ObjectBuilderOptions builder_options;
    if (!builder_.Build(builder_options, merged_msg->lidar_frame_.get())) {
      AERROR << "Failed to build objects.";
    }

    PERCEPTION_PERF_BLOCK_END("PointCloud Detector Build");

    if (perception_debug_writer_->HasReader() && use_viz_debug_) {
      PublishPerceptionObstacles(perception_debug_writer_, merged_msg,
                                 localization_msg->header().timestamp_sec());
    }

    auto out_message = std::make_shared<SensorFrameMessage>();
    if (InternalProc(merged_msg, out_message)) {
      auto perception_obstacles = std::make_shared<PerceptionObstacles>();
      if (!MsgSerializer::SerializeMsg(
              out_message->timestamp_, out_message->lidar_timestamp_,
              out_message->seq_num_, out_message->frame_->objects,
              out_message->error_code_, perception_obstacles.get())) {
        AERROR << "Failed to serialize PerceptionObstacles object.";
        continue;
      }

      auto header = perception_obstacles->mutable_header();
      header->set_timestamp_sec(localization_msg->header().timestamp_sec());
      auto& lidar2world_pose = merged_msg->lidar_frame_->lidar2world_pose;
      century::perception::Affine3dProto lidar2world_pose_proto;
      const auto& translation = lidar2world_pose.translation();
      lidar2world_pose_proto.set_tx(translation.x());
      lidar2world_pose_proto.set_ty(translation.y());
      lidar2world_pose_proto.set_tz(translation.z());

      Eigen::Quaterniond quat(lidar2world_pose.linear());
      lidar2world_pose_proto.set_qw(quat.w());
      lidar2world_pose_proto.set_qx(quat.x());
      lidar2world_pose_proto.set_qy(quat.y());
      lidar2world_pose_proto.set_qz(quat.z());
      perception_obstacles->mutable_lidar2world()->CopyFrom(
          lidar2world_pose_proto);
      perception_writer_->Write(perception_obstacles);
      around_vehicle_queue_->PushBack(merged_msg);
    } else {
      AERROR << "Failed to process lidar frame message.";
    }
  }
}

void LidarDetectorComponent::Detect(LidarFrameMessagePtr frame) noexcept {
  lidar::LidarObstacleDetectionOptions detect_opts;
  struct timeval start, end;
  gettimeofday(&start, NULL);
  detect_opts.sensor_name = sensor_name_;
  lidar::LidarProcessResult ret =
      detector_->Process(detect_opts, frame->lidar_frame_.get());
  gettimeofday(&end, NULL);
  long seconds = end.tv_sec - start.tv_sec;
  long useconds = end.tv_usec - start.tv_usec;
  double elapsed_ms =
      ((seconds)*1000 + useconds / 1000.0) + (useconds % 1000) / 1000.0;
  AINFO << "dnn Time taken: " << elapsed_ms << " ms";
}

void LidarDetectorComponent::Cluster(LidarFrameMessagePtr frame) noexcept {
  struct timeval start, end;
  gettimeofday(&start, NULL);
  lidar::ClusterOptions cluster_options;
  cluster_processor_->Cluster(cluster_options, frame->lidar_frame_.get());
  gettimeofday(&end, NULL);
  long seconds = end.tv_sec - start.tv_sec;
  long useconds = end.tv_usec - start.tv_usec;
  double elapsed_ms =
      ((seconds)*1000 + useconds / 1000.0) + (useconds % 1000) / 1000.0;
  AINFO << "cluster Time taken: " << elapsed_ms << " ms and points size is "
        << frame->lidar_frame_->cloud->size() << std::endl;
}

bool LidarDetectorComponent::UpdateLidarPose(
    LidarFrameMessagePtr merged_msg,
    const std::shared_ptr<century::localization::LocalizationEstimate>&
        localization_msg) noexcept {
  auto& world_pose = localization_msg->pose();
  Eigen::Affine3d lidar2world_pose = Eigen::Affine3d();
  Eigen::Quaterniond vehicle2world_quaternion(
      world_pose.orientation().qw(), world_pose.orientation().qx(),
      world_pose.orientation().qy(), world_pose.orientation().qz());
  Eigen::Vector3d vehicle2world_translate(world_pose.position().x(),
                                          world_pose.position().y(),
                                          world_pose.position().z());
  lidar2world_pose.linear() = vehicle2world_quaternion.toRotationMatrix();
  lidar2world_pose.translation() = vehicle2world_translate;
  Eigen::Matrix4d imu2vehicle_matrix;
  lidar2vehicle_trans_.QueryStaticTF("vehicle", "imu", &imu2vehicle_matrix);
  merged_msg->lidar_frame_->lidar2world_pose =
      lidar2world_pose * imu2vehicle_matrix;

  // dump pcl for verify
  if (!FLAGS_dump_world_pcl.empty()) {
    DumpWorldPcl(merged_msg->timestamp_, merged_msg->lidar_frame_->cloud,
                 merged_msg->lidar_frame_->lidar2world_pose);
  }

  return true;
}

bool LidarDetectorComponent::InitAlgorithmPlugin() noexcept {
  lidar::BaseLidarObstacleDetection* detector =
      lidar::BaseLidarObstacleDetectionRegisterer::GetInstanceByName(
          detector_name_);
  CHECK_NOTNULL(detector);
  detector_.reset(detector);
  lidar::LidarObstacleDetectionInitOptions init_options;
  init_options.sensor_name = sensor_name_;
  init_options.enable_hdmap_input =
      FLAGS_obs_enable_hdmap_input && enable_hdmap_;

  lidar::BasePointCloudCluster* cluster =
      lidar::BasePointCloudClusterRegisterer::GetInstanceByName(cluster_name_);
  cluster_processor_.reset(cluster);
  lidar::ClusterInitOptions cluster_init_options;
  cluster_init_options.enable_hdmap_input =
      FLAGS_obs_enable_hdmap_input && enable_hdmap_;
  cluster_processor_->Init(cluster_init_options);

  ACHECK(detector_->Init(init_options))
      << "lidar obstacle detection init error";

  if (use_filter_bank_) {
    lidar::ObjectFilterInitOptions filter_bank_init_options;
    filter_bank_init_options.sensor_name = sensor_name_;
    std::cout << sensor_name_ << std::endl;
    ACHECK(filter_bank_.Init(filter_bank_init_options));
  }

  return true;
}

bool LidarDetectorComponent::InternalProc(
    const std::shared_ptr<const LidarFrameMessage>& in_message,
    const std::shared_ptr<SensorFrameMessage>& out_message) noexcept {
  auto& sensor_name = in_message->lidar_frame_->sensor_info.name;

  out_message->timestamp_ = in_message->timestamp_;
  out_message->lidar_timestamp_ = in_message->lidar_timestamp_;
  out_message->seq_num_ = in_message->seq_num_;
  out_message->process_stage_ = ProcessStage::LIDAR_RECOGNITION;
  out_message->sensor_id_ = sensor_name;

  if (in_message->error_code_ != century::common::ErrorCode::OK) {
    out_message->error_code_ = in_message->error_code_;
    AERROR << "Lidar recognition receive message with error code, skip it.";
    return true;
  }

  auto& lidar_frame = in_message->lidar_frame_;
  out_message->hdmap_ = lidar_frame->hdmap_struct;
  auto& frame = out_message->frame_;
  frame = base::FramePool::Instance().Get();
  frame->sensor_info = lidar_frame->sensor_info;
  frame->timestamp = in_message->timestamp_;
  frame->objects = lidar_frame->segmented_objects;
  frame->sensor2world_pose = lidar_frame->lidar2world_pose;
  frame->lidar_frame_supplement.on_use = true;
  frame->lidar_frame_supplement.cloud_ptr = lidar_frame->cloud;

  const double end_timestamp = Clock::NowInSeconds();
  const double end_latency = (end_timestamp - in_message->timestamp_) * 1e3;
  AINFO << std::setprecision(16) << "FRAME_STATISTICS:Lidar:End:msg_time["
        << in_message->timestamp_ << "]:cur_time[" << end_timestamp
        << "]:cur_latency[" << end_latency << "]";
  return true;
}

void LidarDetectorComponent::PublishPerceptionObstacles(
    const std::shared_ptr<century::cyber::Writer<PerceptionObstacleDebugMsg>>&
        writer,
    const LidarFrameMessagePtr& merged_msg, double tm) noexcept {
  auto out_message = std::make_shared<SensorFrameMessage>();
  if (InternalProc(merged_msg, out_message)) {
    auto perception_obstacles = std::make_shared<PerceptionObstacles>();

    for (auto& obj : out_message->frame_->objects) {
      obj->track_id = obj->id;
    }

    if (!MsgSerializer::SerializeMsg(
            out_message->timestamp_, out_message->lidar_timestamp_,
            out_message->seq_num_, out_message->frame_->objects,
            out_message->error_code_, perception_obstacles.get())) {
      AERROR << "Failed to serialize PerceptionObstacles object.";
      return;
    }

    auto header = perception_obstacles->mutable_header();
    header->set_timestamp_sec(tm);
    auto perception_obstacle_debug_msg =
        std::make_shared<PerceptionObstacleDebugMsg>();
    perception_obstacle_debug_msg->mutable_obstacles()->CopyFrom(
        *perception_obstacles);

    drivers::PointCloud point_clouds;

    point_clouds.mutable_header()->set_sequence_num(merged_msg->seq_num_);
    point_clouds.set_width(merged_msg->lidar_frame_->raw_cloud->width());
    point_clouds.set_height(merged_msg->lidar_frame_->raw_cloud->height());
    point_clouds.mutable_header()->set_timestamp_sec(merged_msg->timestamp_);
    point_clouds.set_measurement_time(cyber::Time::Now().ToSecond());
    point_clouds.set_is_dense(true);

    for (auto& point : merged_msg->lidar_frame_->raw_cloud->points()) {
      auto point_cloud = point_clouds.add_point();
      point_cloud->set_x(point.x);
      point_cloud->set_y(point.y);
      point_cloud->set_z(point.z);
      point_cloud->set_intensity(point.intensity);
    }

    drivers::PointCloud seg_point_clouds;
    seg_point_clouds.mutable_header()->set_sequence_num(merged_msg->seq_num_);
    seg_point_clouds.set_width(merged_msg->lidar_frame_->cloud->width());
    seg_point_clouds.set_height(merged_msg->lidar_frame_->cloud->height());
    seg_point_clouds.mutable_header()->set_timestamp_sec(
        merged_msg->timestamp_);
    seg_point_clouds.set_measurement_time(cyber::Time::Now().ToSecond());
    seg_point_clouds.set_is_dense(true);

    for (auto& point : merged_msg->lidar_frame_->cloud->points()) {
      auto point_cloud = seg_point_clouds.add_point();
      point_cloud->set_x(point.x);
      point_cloud->set_y(point.y);
      point_cloud->set_z(point.z);
      point_cloud->set_intensity(point.intensity);
    }

    perception_obstacle_debug_msg->mutable_raw_pointcloud()->CopyFrom(
        point_clouds);
    perception_obstacle_debug_msg->mutable_seg_pointcloud()->CopyFrom(
        seg_point_clouds);

    auto& lidar2world_pose = merged_msg->lidar_frame_->lidar2world_pose;
    century::perception::Affine3dProto lidar2world_pose_proto;
    const auto& translation = lidar2world_pose.translation();
    lidar2world_pose_proto.set_tx(translation.x());
    lidar2world_pose_proto.set_ty(translation.y());
    lidar2world_pose_proto.set_tz(translation.z());

    Eigen::Quaterniond quat(lidar2world_pose.linear());
    lidar2world_pose_proto.set_qw(quat.w());
    lidar2world_pose_proto.set_qx(quat.x());
    lidar2world_pose_proto.set_qy(quat.y());
    lidar2world_pose_proto.set_qz(quat.z());
    perception_obstacle_debug_msg->mutable_lidar2world()->CopyFrom(
        lidar2world_pose_proto);
    writer->Write(perception_obstacle_debug_msg);
  } else {
    AERROR << "Failed to process lidar frame message.";
  }
}

void LidarDetectorComponent::FilterPointCloudInBoundingBox(
    const LidarFrameMessagePtr& merged_msg) noexcept {
  auto non_ground_pc = merged_msg->lidar_frame_->cloud;
  auto objects = merged_msg->lidar_frame_->segmented_objects;
  constexpr float kRangeThreshold = 0.10f;
  std::vector<uint32_t> remove_pt_indices;
  remove_pt_indices.reserve(non_ground_pc->size());

  auto result_pc = base::PointFCloudPool::Instance().Get();
  result_pc->reserve(non_ground_pc->size());

  for (size_t i = 0; i < non_ground_pc->size(); ++i) {
    const auto& pt = non_ground_pc->at(i);
    bool is_point_inside_object = false;

    for (const auto& obj : objects) {
      if (obj->type != base::ObjectType::VEHICLE &&
          obj->type != base::ObjectType::STACKER) {
        continue;
      }

      Eigen::Vector3f center(obj->center(0), obj->center(1), obj->center(2));
      Eigen::Vector3f size = obj->size;
      float yaw = obj->theta;

      Eigen::Affine3f tf = Eigen::Affine3f::Identity();
      tf.translate(center);
      tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
      Eigen::Affine3f tf_inv = tf.inverse();

      Eigen::Vector3f raw_pt(pt.x, pt.y, pt.z);

      Eigen::Vector3f pt_local = tf_inv * raw_pt;

      float half_l = size(0) / 2 + kRangeThreshold;
      float half_w = size(1) / 2 + kRangeThreshold;
      float half_h = size(2) / 2 + kRangeThreshold;

      if (std::abs(pt_local.x()) <= half_l &&
          std::abs(pt_local.y()) <= half_w &&
          std::abs(pt_local.z()) <= half_h) {
        is_point_inside_object = true;
        break;
      }
    }

    if (!is_point_inside_object) {
      result_pc->push_back(pt);
    } else {
      remove_pt_indices.push_back(i);
    }
  }

  result_pc->resize(result_pc->size());
  *non_ground_pc = *result_pc;
}

void LidarDetectorComponent::RemoveHighTrailerObjects(
    const LidarFrameMessagePtr& merged_msg) noexcept {
  auto non_ground_pc = merged_msg->lidar_frame_->cloud;
  auto& objects = merged_msg->lidar_frame_->segmented_objects;

  constexpr float kMinHeightThreshold = 2.0f;
  constexpr float kRangeThreshold = 0.10f;

  std::vector<size_t> objects_to_remove;

  for (size_t obj_idx = 0; obj_idx < objects.size(); ++obj_idx) {
    const auto& obj = objects[obj_idx];

    if (obj->sub_type != base::ObjectSubType::TRAILER_FULL) {
      continue;
    }

    Eigen::Vector3f center(obj->center(0), obj->center(1), obj->center(2));
    Eigen::Vector3f size = obj->size;
    float yaw = obj->theta;

    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translate(center);
    tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    Eigen::Affine3f tf_inv = tf.inverse();

    float half_l = size(0) * 0.5f + kRangeThreshold;
    float half_w = size(1) * 0.5f + kRangeThreshold;
    float half_h = size(2) * 0.5f + kRangeThreshold;

    float min_z = std::numeric_limits<float>::max();
    bool has_points_inside = false;

    for (size_t i = 0; i < non_ground_pc->size(); ++i) {
      const auto& pt = non_ground_pc->at(i);
      Eigen::Vector3f raw_pt(pt.x, pt.y, pt.z);
      Eigen::Vector3f pt_local = tf_inv * raw_pt;

      if (std::abs(pt_local.x()) <= half_l &&
          std::abs(pt_local.y()) <= half_w &&
          std::abs(pt_local.z()) <= half_h) {
        has_points_inside = true;
        min_z = std::min(min_z, pt.z);
      }
    }

    if (has_points_inside && min_z > kMinHeightThreshold) {
      objects_to_remove.emplace_back(obj_idx);
      break;
    }
  }

  for (auto it = objects_to_remove.rbegin(); it != objects_to_remove.rend();
       ++it) {
    objects.erase(objects.begin() + *it);
  }
}

}  // namespace onboard
}  // namespace perception
}  // namespace century
