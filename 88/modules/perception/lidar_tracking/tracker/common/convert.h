/******************************************************************************
 * Copyright 2023 The century Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "cyber/time/clock.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/lidar_tracking/tracker/common/common.h"
#include <Eigen/Core>

namespace century {
namespace perception {


Eigen::Vector4d ConvertToBb2x(const common::Point3D &point, const double yaw,  const double length, const double width) {
  std::array<Eigen::Vector2d, 4UL> corners;

  double halfLength = length / 2.0;
  double halfWidth = width / 2.0;

  double sinYaw = sin(yaw);
  double cosYaw = cos(yaw);

  double localCorners[4][2] = {
    { halfLength,  halfWidth},  
    { halfLength, -halfWidth},  
    {-halfLength, -halfWidth}, 
    {-halfLength,  halfWidth} 
  };

  for (int i = 0; i < corners.size(); ++i) {
    double localX = localCorners[i][0];
    double localY = localCorners[i][1];

    corners[i](0) = localX * cosYaw - localY * sinYaw;
    corners[i](1) = localX * sinYaw + localY * cosYaw;

    corners[i](0) += point.x();
    corners[i](1) += point.y();
  }


  double x_min = std::numeric_limits<double>::max();
  double y_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto& point : corners) {
      x_min = std::min(x_min, point.x());
      y_min = std::min(y_min, point.y());
      x_max = std::max(x_max, point.x());
      y_max = std::max(y_max, point.y());
  }

  return Eigen::Vector4d (x_min, y_min, x_max, y_max);
}

void FillInBb2xValue(century::perception::PerceptionObstacles *perception_msg) {
  for(int i=0; i < perception_msg->perception_obstacle_size(); i++) {
    Eigen::Vector4d bbox = century::perception::ConvertToBb2x( 
      perception_msg->perception_obstacle(i).position(),
      perception_msg->perception_obstacle(i).theta(),
      perception_msg->perception_obstacle(i).length(),
      perception_msg->perception_obstacle(i).width());
    century::perception::BBox2D* bbox2d = new century::perception::BBox2D();
    bbox2d->set_xmin(bbox(0));
    bbox2d->set_ymin(bbox(1));
    bbox2d->set_xmax(bbox(2));
    bbox2d->set_ymax(bbox(3));
    perception_msg->mutable_perception_obstacle(i)->set_allocated_bbox2d(bbox2d);
  }
}

void CopyFreespaceFields(const PerceptionObstacles &source,
                         PerceptionObstacles *target) {
  if (target == nullptr) {
    return;
  }

  target->clear_lidar2world();
  target->clear_freespace_mask_polygon();
  target->clear_freespace_left_boundary();
  target->clear_freespace_right_boundary();

  if (source.has_lidar2world()) {
    target->mutable_lidar2world()->CopyFrom(source.lidar2world());
  }
  for (const auto &polygon : source.freespace_mask_polygon()) {
    target->add_freespace_mask_polygon()->CopyFrom(polygon);
  }
  if (source.has_freespace_left_boundary()) {
    target->mutable_freespace_left_boundary()->CopyFrom(
        source.freespace_left_boundary());
  }
  if (source.has_freespace_right_boundary()) {
    target->mutable_freespace_right_boundary()->CopyFrom(
        source.freespace_right_boundary());
  }
}

bool ConvertSensorFrameMessage2Obstacles(
    const std::shared_ptr<onboard::SensorFrameMessage> &msg,
    PerceptionObstacles *obstacles) {
  auto header = obstacles->mutable_header();
  header->set_timestamp_sec(obstacles->header().timestamp_sec());
  header->set_module_name("perception_obstacle");
  header->set_sequence_num(msg->seq_num_);
  header->set_lidar_timestamp(msg->lidar_timestamp_);
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  obstacles->set_error_code(century::common::ErrorCode::OK);
  if (msg != nullptr && msg->frame_ != nullptr) {
    for (const auto &obj : msg->frame_->objects) {
      PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
      if (!ConvertObjectToPb(obj, obstacle)) {
        AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
        return false;
      }
    }
    FillInBb2xValue(obstacles);
  }
  return true;
}

bool ConvertLidarFrameMessage2Obstacles(
    const std::shared_ptr<onboard::LidarFrameMessage> &msg,
    PerceptionObstacles *obstacles) {
  static uint32_t seq_num = 0;
  auto header = obstacles->mutable_header();
  header->set_timestamp_sec(obstacles->header().timestamp_sec());
  header->set_module_name("perception_lidar");
  header->set_sequence_num(seq_num++);
  header->set_lidar_timestamp(msg->lidar_timestamp_);
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  obstacles->set_error_code(century::common::OK);

  int id = 0;
  for (const auto &obj : msg->lidar_frame_.get()->segmented_objects) {
    Eigen::Vector3d trans_point(obj->center(0), obj->center(1), obj->center(2));
    trans_point = msg->lidar_frame_.get()->lidar2world_pose * trans_point;
    obj->center(0) = trans_point[0];
    obj->center(1) = trans_point[1];
    obj->center(2) = trans_point[2];

    for (size_t i = 0; i < obj->polygon.size(); ++i) {
      auto &pt = obj->polygon[i];
      Eigen::Vector3d trans_point_polygon(pt.x, pt.y, pt.z);
      trans_point_polygon =
          msg->lidar_frame_.get()->lidar2world_pose * trans_point_polygon;
      pt.x = trans_point_polygon[0];
      pt.y = trans_point_polygon[1];
      pt.z = trans_point_polygon[2];
    }

    base::PointDCloud& cloud_world = (obj->lidar_supplement).cloud_world;
    cloud_world.clear();
    cloud_world.resize(obj->lidar_supplement.cloud.size());
    for (size_t i = 0; i < obj->lidar_supplement.cloud.size(); ++i) {
      Eigen::Vector3d pt(obj->lidar_supplement.cloud[i].x,
                         obj->lidar_supplement.cloud[i].y,
                         obj->lidar_supplement.cloud[i].z);
      Eigen::Vector3d pt_world = msg->lidar_frame_.get()->lidar2world_pose * pt;
      cloud_world[i].x = pt_world(0);
      cloud_world[i].y = pt_world(1);
      cloud_world[i].z = pt_world(2);
      cloud_world[i].intensity = obj->lidar_supplement.cloud[i].intensity;
    }

    for (size_t i = 0; i < obj->lidar_supplement.cloud.size(); ++i) {
      cloud_world.SetPointHeight(i,
                                 obj->lidar_supplement.cloud.points_height(i));
    }

    Eigen::Vector3d trans_anchor_point(obj->anchor_point(0),
                                       obj->anchor_point(1),
                                       obj->anchor_point(2));
    trans_anchor_point =
        msg->lidar_frame_.get()->lidar2world_pose * trans_anchor_point;
    obj->anchor_point(0) = trans_anchor_point[0];
    obj->anchor_point(1) = trans_anchor_point[1];
    obj->anchor_point(2) = trans_anchor_point[2];

    obj->track_id = id++;
  }

  for (const auto &obj : msg->lidar_frame_.get()->segmented_objects) {
    PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
    if (!ConvertObjectToPb(obj, obstacle)) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return false;
    }
  }
  return true;
}

}  // namespace perception
}  // namespace century
