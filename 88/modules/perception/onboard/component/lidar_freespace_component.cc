/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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

#include "modules/perception/onboard/component/lidar_freespace_component.h"

#include <algorithm>

#include <Eigen/Geometry>

#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"

namespace century {
namespace perception {
namespace onboard {

bool LidarFreespaceComponent::Init() {
  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get config failed";
    return false;
  }

  if (comp_config.input_channel_name().empty()) {
    AERROR << "input_channel_name is empty in lidar freespace config.";
    return false;
  }
  input_channel_name_ = comp_config.input_channel_name(0);
  output_channel_name_ = comp_config.output_channel_name();
  debug_output_channel_name_ = comp_config.debug_output_channel_name();
  if (output_channel_name_.empty()) {
    AERROR << "output_channel_name is empty in lidar freespace config.";
    return false;
  }

  enable_freespace_mask_ = comp_config.enable_freespace_mask();
  freespace_options_.spec = {comp_config.freespace_mask_x_min(),
                             comp_config.freespace_mask_x_max(),
                             comp_config.freespace_mask_y_min(),
                             comp_config.freespace_mask_y_max(),
                             comp_config.freespace_mask_resolution()};
  freespace_options_.min_points_per_cell =
      std::max(1, comp_config.freespace_min_points_per_cell());
  freespace_options_.obstacle_inflate =
      std::max(0.0, comp_config.freespace_obstacle_inflate());
  freespace_options_.use_hdmap_road = comp_config.freespace_use_hdmap_road();
  freespace_options_.show_ray_source = comp_config.freespace_show_ray_source();
  freespace_options_.use_cuda = comp_config.freespace_use_cuda();
  freespace_options_.enable_temporal_filter =
      comp_config.freespace_enable_temporal_filter();
  freespace_options_.temporal_expand_alpha =
      std::max(0.0, std::min(1.0,
                             comp_config.freespace_temporal_expand_alpha()));
  freespace_options_.temporal_max_expand =
      std::max(0.0, comp_config.freespace_temporal_max_expand());
  freespace_options_.temporal_source_shift_reset =
      std::max(0.0, comp_config.freespace_temporal_source_shift_reset());

  perception_writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_channel_name_);
  if (!debug_output_channel_name_.empty()) {
    debug_writer_ =
        node_->CreateWriter<PerceptionObstacleDebugMsg>(
            debug_output_channel_name_);
  }
  lidar_reader_ = node_->CreateReader<LidarFrameMessage>(
      input_channel_name_,
      [this](const std::shared_ptr<LidarFrameMessage>& message) {
        Process(message);
      });
  return true;
}

void LidarFreespaceComponent::Process(
    const std::shared_ptr<LidarFrameMessage>& message) noexcept {
  if (nullptr == message || nullptr == message->lidar_frame_) {
    return;
  }

  auto perception_obstacles = std::make_shared<PerceptionObstacles>();
  const auto& frame = *message->lidar_frame_;
  if (!MsgSerializer::SerializeMsg(message->timestamp_,
                                   message->lidar_timestamp_,
                                   message->seq_num_,
                                   frame.segmented_objects,
                                   message->error_code_,
                                   perception_obstacles.get())) {
    AERROR << "Failed to serialize PerceptionObstacles object.";
    return;
  }

  auto* header = perception_obstacles->mutable_header();
  header->set_timestamp_sec(message->timestamp_);
  const auto& lidar2world_pose = frame.lidar2world_pose;
  Affine3dProto lidar2world_pose_proto;
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

  PerceptionObstacleDebugMsg freespace_debug_msg;
  if (enable_freespace_mask_) {
    lidar::BuildFreespaceMask(frame, freespace_options_,
                              &freespace_debug_msg,
                              &freespace_temporal_state_);
    lidar::CopyFreespaceToObstaclesWorld(freespace_debug_msg, lidar2world_pose,
                                         perception_obstacles.get());
  }
  perception_writer_->Write(perception_obstacles);

  if (nullptr != debug_writer_ && debug_writer_->HasReader()) {
    auto debug_msg =
        std::make_shared<PerceptionObstacleDebugMsg>(freespace_debug_msg);
    debug_msg->mutable_obstacles()->CopyFrom(*perception_obstacles);
    lidar::CopyFreespaceToObstaclesLocal(freespace_debug_msg,
                                         debug_msg->mutable_obstacles());
    debug_msg->mutable_lidar2world()->CopyFrom(lidar2world_pose_proto);
    debug_writer_->Write(debug_msg);
  }
}

}  // namespace onboard
}  // namespace perception
}  // namespace century
