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
#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/perception_obstacle_debug.pb.h"

namespace century {
namespace perception {
namespace lidar {

struct FreespaceGridSpec {
  double x_min = -20.0;
  double x_max = 50.0;
  double y_min = -20.0;
  double y_max = 20.0;
  double resolution = 0.3;
};

struct FreespaceBuildOptions {
  FreespaceGridSpec spec;
  int min_points_per_cell = 1;
  double obstacle_inflate = 0.0;
  bool use_hdmap_road = false;
  bool show_ray_source = false;
  bool use_cuda = true;
  bool enable_temporal_filter = false;
  double temporal_expand_alpha = 0.35;
  double temporal_max_expand = 1.2;
  double temporal_source_shift_reset = 2.0;
};

struct FreespaceTemporalFilterState {
  FreespaceGridSpec spec;
  double source_x = 0.0;
  double source_y = 0.0;
  std::vector<double> ray_ranges;
  std::vector<uint8_t> ray_valid;
  std::vector<double> pending_ray_ranges;
  std::vector<uint8_t> pending_ray_valid;
  std::vector<uint8_t> pending_change_type;
  std::vector<uint8_t> pending_change_count;
};

bool BuildFreespaceMask(const LidarFrame& frame,
                        const FreespaceBuildOptions& options,
                        PerceptionObstacleDebugMsg* debug_msg,
                        FreespaceTemporalFilterState* temporal_state = nullptr);

// Copy freespace polygon/boundary from lidar-frame debug_msg into obstacles
// WITHOUT any coordinate transform. Use for the debug topic consumed by
// visualizer which expects ego/lidar coordinates.
void CopyFreespaceToObstaclesLocal(
    const PerceptionObstacleDebugMsg& debug_msg,
    PerceptionObstacles* perception_obstacles);

// Copy freespace polygon/boundary from lidar-frame debug_msg into obstacles
// AND transform them into world coordinates. Use for the PerceptionObstacles
// message consumed by planning/control.
void CopyFreespaceToObstaclesWorld(
    const PerceptionObstacleDebugMsg& debug_msg,
    const Eigen::Affine3d& lidar2world_pose,
    PerceptionObstacles* perception_obstacles);

}  // namespace lidar
}  // namespace perception
}  // namespace century
