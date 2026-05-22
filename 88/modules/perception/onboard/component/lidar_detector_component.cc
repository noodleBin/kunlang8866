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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>
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
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/common/i_lib/core/i_constant.h"
#include "modules/perception/base/singleton.h"
#include "modules/perception/common/sensor_manager/camera_sensor_config.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/common/timer_util.h"
#include "modules/perception/base/singleton.h"
#include "modules/perception/lidar/common/duplicated_object_filter.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_util.h"
#include "modules/perception/lidar/lib/freespace/freespace_mask.h"
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
};

struct FreespaceGridContext {
  FreespaceGridSpec spec;
  int width = 0;
  int height = 0;
  double ground_z = 0.0;
  double occupancy_z_min = 0.0;
  double occupancy_z_max = 0.0;
  double self_x_min = 0.0;
  double self_x_max = 0.0;
  double self_y_min = 0.0;
  double self_y_max = 0.0;
  std::vector<uint8_t> occupied;
  std::vector<uint16_t> occupied_point_count;
  std::vector<uint8_t> map_allowed;

  bool IsInside(const double x, const double y) const {
    return x >= spec.x_min && x < spec.x_max && y >= spec.y_min &&
           y < spec.y_max;
  }

  void ToGrid(const double x, const double y, int* gx, int* gy) const {
    *gx = static_cast<int>(std::floor((x - spec.x_min) / spec.resolution));
    *gy = static_cast<int>(std::floor((y - spec.y_min) / spec.resolution));
  }

  void CellCenter(const int gx, const int gy, double* x, double* y) const {
    *x = spec.x_min + (static_cast<double>(gx) + 0.5) * spec.resolution;
    *y = spec.y_min + (static_cast<double>(gy) + 0.5) * spec.resolution;
  }

  bool IsInsideVehicle(const double x, const double y) const {
    return x >= self_x_min && x <= self_x_max && y >= self_y_min &&
           y <= self_y_max;
  }
};

struct FreespaceRayState {
  int ray_count = 720;
  double source_x = 0.0;
  double source_y = 0.0;
  double max_range = 0.0;
  double angular_step = 0.0;
  std::vector<double> ray_ranges;
  std::vector<uint8_t> ray_valid;
  std::vector<uint8_t> ray_hit_occupied;
  std::vector<double> ray_hit_occ_x;
  std::vector<double> ray_hit_occ_y;
};

inline bool IsFiniteValue(const double v) { return std::isfinite(v); }

inline double GetGroundZ(const lidar::LidarFrame& frame) {
  if (IsFiniteValue(frame.original_ground_z) &&
      std::fabs(frame.original_ground_z) < 5.0) {
    return frame.original_ground_z;
  }
  if (IsFiniteValue(frame.parsing_ground_height) &&
      std::fabs(frame.parsing_ground_height) < 5.0) {
    return frame.parsing_ground_height;
  }
  return 0.0;
}

inline int ToGridIndex(const int gx, const int gy, const int width) {
  return gy * width + gx;
}

bool PrepareFreespaceGridContext(const lidar::LidarFrame& frame,
                                 const FreespaceBuildOptions& options,
                                 FreespaceGridContext* context) {
  if (nullptr == context) {
    return false;
  }
  const auto& spec = options.spec;
  if (spec.resolution <= 1e-6 || spec.x_max <= spec.x_min ||
      spec.y_max <= spec.y_min) {
    AERROR << "Invalid freespace mask config. x[" << spec.x_min << ", "
           << spec.x_max << "] y[" << spec.y_min << ", " << spec.y_max
           << "] resolution: " << spec.resolution;
    return false;
  }

  context->width =
      static_cast<int>(std::ceil((spec.x_max - spec.x_min) / spec.resolution));
  context->height =
      static_cast<int>(std::ceil((spec.y_max - spec.y_min) / spec.resolution));
  if (context->width <= 0 || context->height <= 0) {
    return false;
  }

  context->spec = spec;
  context->ground_z = GetGroundZ(frame);
  const auto& vehicle_param =
      century::common::VehicleConfigHelper::GetConfig().vehicle_param();
  context->occupancy_z_min = context->ground_z - 1.0;
  context->occupancy_z_max = context->ground_z + vehicle_param.height() + 1.0;
  constexpr double kSelfFootprintMargin = 0.2;
  context->self_x_min =
      -vehicle_param.back_edge_to_center() - kSelfFootprintMargin;
  context->self_x_max =
      vehicle_param.front_edge_to_center() + kSelfFootprintMargin;
  context->self_y_min =
      -vehicle_param.right_edge_to_center() - kSelfFootprintMargin;
  context->self_y_max =
      vehicle_param.left_edge_to_center() + kSelfFootprintMargin;

  const size_t grid_size =
      static_cast<size_t>(context->width * context->height);
  context->occupied.assign(grid_size, 0);
  context->occupied_point_count.assign(grid_size, 0);
  context->map_allowed.assign(grid_size, 1);
  return true;
}

void MarkPointOccupancy(const lidar::LidarFrame& frame,
                        const int min_points_per_cell,
                        FreespaceGridContext* context) {
  if (nullptr == context || nullptr == frame.raw_cloud) {
    return;
  }
  const auto& non_ground_idx = frame.non_ground_indices.indices;
  for (const int pt_idx : non_ground_idx) {
    if (pt_idx < 0 || static_cast<size_t>(pt_idx) >= frame.raw_cloud->size()) {
      continue;
    }
    const auto& point = frame.raw_cloud->at(static_cast<size_t>(pt_idx));
    if (!IsFiniteValue(point.x) || !IsFiniteValue(point.y) ||
        !IsFiniteValue(point.z) ||
        !context->IsInside(point.x, point.y) ||
        point.z < context->occupancy_z_min ||
        point.z > context->occupancy_z_max ||
        context->IsInsideVehicle(point.x, point.y)) {
      continue;
    }
    int gx = 0;
    int gy = 0;
    context->ToGrid(point.x, point.y, &gx, &gy);
    if (gx < 0 || gx >= context->width || gy < 0 || gy >= context->height) {
      continue;
    }
    const size_t idx =
        static_cast<size_t>(ToGridIndex(gx, gy, context->width));
    if (context->occupied_point_count[idx] <
        std::numeric_limits<uint16_t>::max()) {
      ++context->occupied_point_count[idx];
    }
  }

  for (size_t idx = 0; idx < context->occupied_point_count.size(); ++idx) {
    if (context->occupied_point_count[idx] >= min_points_per_cell) {
      context->occupied[idx] = 1;
    }
  }
}

void MarkObjectOccupancy(const lidar::LidarFrame& frame, const double inflate,
                         FreespaceGridContext* context) {
  if (nullptr == context) {
    return;
  }
  for (const auto& obj : frame.segmented_objects) {
    if (nullptr == obj || obj->type == base::ObjectType::UNKNOWN) {
      continue;
    }
    const double length = std::max(0.0, static_cast<double>(obj->size(0)));
    const double width = std::max(0.0, static_cast<double>(obj->size(1)));
    if (length < 1e-3 || width < 1e-3) {
      continue;
    }
    const double cx = static_cast<double>(obj->center(0));
    const double cy = static_cast<double>(obj->center(1));
    const double yaw = static_cast<double>(obj->theta);
    const double half_l = 0.5 * length + inflate;
    const double half_w = 0.5 * width + inflate;
    const double radius = std::hypot(half_l, half_w);
    if (cx + radius < context->spec.x_min ||
        cx - radius >= context->spec.x_max ||
        cy + radius < context->spec.y_min ||
        cy - radius >= context->spec.y_max) {
      continue;
    }
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    int gx_min = 0;
    int gy_min = 0;
    int gx_max = 0;
    int gy_max = 0;
    context->ToGrid(cx - radius, cy - radius, &gx_min, &gy_min);
    context->ToGrid(cx + radius, cy + radius, &gx_max, &gy_max);
    gx_min = std::max(0, gx_min);
    gy_min = std::max(0, gy_min);
    gx_max = std::min(context->width - 1, gx_max);
    gy_max = std::min(context->height - 1, gy_max);

    for (int gy = gy_min; gy <= gy_max; ++gy) {
      for (int gx = gx_min; gx <= gx_max; ++gx) {
        double x = 0.0;
        double y = 0.0;
        context->CellCenter(gx, gy, &x, &y);
        const double dx = x - cx;
        const double dy = y - cy;
        const double local_x = cos_yaw * dx + sin_yaw * dy;
        const double local_y = -sin_yaw * dx + cos_yaw * dy;
        if (std::fabs(local_x) <= half_l && std::fabs(local_y) <= half_w) {
          context->occupied[static_cast<size_t>(
              ToGridIndex(gx, gy, context->width))] = 1;
        }
      }
    }
  }
}

void ApplyRailwayMask(const lidar::LidarFrame& frame,
                      FreespaceGridContext* context) {
  if (nullptr == context || nullptr == frame.hdmap_struct) {
    return;
  }

  const auto world2lidar = frame.lidar2world_pose.inverse();
  std::vector<base::PointCloud<base::PointF>> local_railway_polygons;
  local_railway_polygons.reserve(
      frame.hdmap_struct->railway_boundary_polygons.size());
  for (const auto& railway_polygon :
       frame.hdmap_struct->railway_boundary_polygons) {
    if (railway_polygon.size() < 3) {
      continue;
    }
    base::PointCloud<base::PointF> local_polygon;
    local_polygon.reserve(railway_polygon.size());
    for (const auto& point : railway_polygon) {
      const Eigen::Vector3d local =
          world2lidar * Eigen::Vector3d(point.x, point.y, point.z);
      base::PointF local_pt;
      local_pt.x = static_cast<float>(local(0));
      local_pt.y = static_cast<float>(local(1));
      local_pt.z = static_cast<float>(local(2));
      local_polygon.push_back(local_pt);
    }
    if (local_polygon.size() >= 3) {
      local_railway_polygons.emplace_back(std::move(local_polygon));
    }
  }
  if (local_railway_polygons.empty()) {
    return;
  }

  for (int gy = 0; gy < context->height; ++gy) {
    for (int gx = 0; gx < context->width; ++gx) {
      double cx = 0.0;
      double cy = 0.0;
      context->CellCenter(gx, gy, &cx, &cy);
      base::PointF query_pt;
      query_pt.x = static_cast<float>(cx);
      query_pt.y = static_cast<float>(cy);
      query_pt.z = 0.0f;

      for (const auto& polygon : local_railway_polygons) {
        if (common::IsPointXYInPolygon2DXY(query_pt, polygon)) {
          context->map_allowed[static_cast<size_t>(
              ToGridIndex(gx, gy, context->width))] = 0;
          break;
        }
      }
    }
  }
}

double ComputeFreespaceSourceX(const lidar::LidarFrame& frame) {
  constexpr double kSourceOffset = 8.0;
  constexpr double kMinLocalXForOffset = 0.2;
  if (!frame.has_motion_direction_world || frame.motion_speed_world_mps <= 1e-3) {
    return 0.0;
  }
  const Eigen::Vector3d motion_world(frame.motion_direction_world.x(),
                                     frame.motion_direction_world.y(), 0.0);
  const Eigen::Vector3d motion_local =
      frame.lidar2world_pose.linear().inverse() * motion_world *
      frame.motion_speed_world_mps;
  if (motion_local.x() > kMinLocalXForOffset) {
    return kSourceOffset;
  }
  if (motion_local.x() < -kMinLocalXForOffset) {
    return -kSourceOffset;
  }
  return 0.0;
}

FreespaceRayState CreateFreespaceRayState(const FreespaceGridContext& context,
                                          const double source_x) {
  FreespaceRayState ray_state;
  ray_state.source_x = source_x;
  ray_state.max_range =
      std::hypot(std::max(std::fabs(context.spec.x_min),
                          std::fabs(context.spec.x_max)),
                 std::max(std::fabs(context.spec.y_min),
                          std::fabs(context.spec.y_max)));
  ray_state.angular_step =
      2.0 * common::Constant<double>::PI() /
      static_cast<double>(ray_state.ray_count);
  ray_state.ray_ranges.assign(static_cast<size_t>(ray_state.ray_count), 0.0);
  ray_state.ray_valid.assign(static_cast<size_t>(ray_state.ray_count), 0);
  ray_state.ray_hit_occupied.assign(
      static_cast<size_t>(ray_state.ray_count), 0);
  ray_state.ray_hit_occ_x.assign(static_cast<size_t>(ray_state.ray_count), 0.0);
  ray_state.ray_hit_occ_y.assign(static_cast<size_t>(ray_state.ray_count), 0.0);
  return ray_state;
}

bool ComputeRayGridRange(const FreespaceGridContext& context,
                         const double source_x, const double source_y,
                         const double dir_x, const double dir_y,
                         double* t_enter, double* t_exit) {
  if (nullptr == t_enter || nullptr == t_exit) {
    return false;
  }

  double near_t = 0.0;
  double far_t = std::numeric_limits<double>::infinity();
  const auto update_axis = [&](const double origin, const double direction,
                               const double min_value,
                               const double max_value) {
    constexpr double kDirectionEpsilon = 1e-12;
    if (std::fabs(direction) < kDirectionEpsilon) {
      return origin >= min_value && origin <= max_value;
    }

    double axis_enter = (min_value - origin) / direction;
    double axis_exit = (max_value - origin) / direction;
    if (axis_enter > axis_exit) {
      std::swap(axis_enter, axis_exit);
    }
    near_t = std::max(near_t, axis_enter);
    far_t = std::min(far_t, axis_exit);
    return near_t <= far_t;
  };

  if (!update_axis(source_x, dir_x, context.spec.x_min, context.spec.x_max) ||
      !update_axis(source_y, dir_y, context.spec.y_min, context.spec.y_max) ||
      far_t < 0.0) {
    return false;
  }

  *t_enter = near_t;
  *t_exit = far_t;
  return true;
}

void WriteRaySourceDebug(const bool show_ray_source,
                         const FreespaceGridContext& context,
                         const FreespaceRayState& ray_state,
                         PerceptionObstacleDebugMsg* debug_msg) {
  if (!show_ray_source || nullptr == debug_msg) {
    return;
  }
  auto* src_pt = debug_msg->mutable_freespace_ray_source();
  src_pt->set_x(ray_state.source_x);
  src_pt->set_y(ray_state.source_y);
  src_pt->set_z(context.ground_z);
}

void CastFreespaceRays(const FreespaceGridContext& context,
                       FreespaceRayState* ray_state) {
  if (nullptr == ray_state) {
    return;
  }
  constexpr double kRayEntryEpsilonScale = 1e-3;
  for (int ray_id = 0; ray_id < ray_state->ray_count; ++ray_id) {
    const double theta =
        -common::Constant<double>::PI() +
        static_cast<double>(ray_id) * ray_state->angular_step;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    double t_enter = 0.0;
    double t_exit = 0.0;
    if (!ComputeRayGridRange(context, ray_state->source_x, ray_state->source_y,
                             cos_theta, sin_theta, &t_enter, &t_exit)) {
      continue;
    }

    double travel_t = std::max(0.0, t_enter);
    if (t_enter > 0.0) {
      travel_t += context.spec.resolution * kRayEntryEpsilonScale;
    }
    double x = ray_state->source_x + travel_t * cos_theta;
    double y = ray_state->source_y + travel_t * sin_theta;
    x = std::min(context.spec.x_max - 1e-9,
                 std::max(context.spec.x_min, x));
    y = std::min(context.spec.y_max - 1e-9,
                 std::max(context.spec.y_min, y));

    int gx = 0;
    int gy = 0;
    context.ToGrid(x, y, &gx, &gy);
    if (gx < 0 || gx >= context.width || gy < 0 || gy >= context.height) {
      continue;
    }

    const int step_x = (0.0 < cos_theta) - (cos_theta < 0.0);
    const int step_y = (0.0 < sin_theta) - (sin_theta < 0.0);
    const double next_boundary_x =
        0 < step_x
            ? context.spec.x_min +
                  static_cast<double>(gx + 1) * context.spec.resolution
            : context.spec.x_min +
                  static_cast<double>(gx) * context.spec.resolution;
    const double next_boundary_y =
        0 < step_y
            ? context.spec.y_min +
                  static_cast<double>(gy + 1) * context.spec.resolution
            : context.spec.y_min +
                  static_cast<double>(gy) * context.spec.resolution;
    const double t_delta_x =
        0 == step_x ? std::numeric_limits<double>::infinity()
                    : context.spec.resolution / std::fabs(cos_theta);
    const double t_delta_y =
        0 == step_y ? std::numeric_limits<double>::infinity()
                    : context.spec.resolution / std::fabs(sin_theta);
    double t_max_x =
        0 == step_x ? std::numeric_limits<double>::infinity()
                    : (next_boundary_x - ray_state->source_x) / cos_theta;
    double t_max_y =
        0 == step_y ? std::numeric_limits<double>::infinity()
                    : (next_boundary_y - ray_state->source_y) / sin_theta;

    bool exited_vehicle = false;
    double last_free_r = -1.0;
    while (travel_t <= t_exit && gx >= 0 && gx < context.width && gy >= 0 &&
           gy < context.height) {
      double cell_center_x = 0.0;
      double cell_center_y = 0.0;
      context.CellCenter(gx, gy, &cell_center_x, &cell_center_y);
      const int idx = ToGridIndex(gx, gy, context.width);
      if (!context.IsInsideVehicle(cell_center_x, cell_center_y)) {
        exited_vehicle = true;
        if (0 == context.map_allowed[static_cast<size_t>(idx)] ||
            0 != context.occupied[static_cast<size_t>(idx)]) {
          if (0 != context.occupied[static_cast<size_t>(idx)]) {
            ray_state->ray_hit_occupied[static_cast<size_t>(ray_id)] = 1;
            ray_state->ray_hit_occ_x[static_cast<size_t>(ray_id)] =
                cell_center_x;
            ray_state->ray_hit_occ_y[static_cast<size_t>(ray_id)] =
                cell_center_y;
          }
          break;
        }
      }

      const double cell_exit_t = std::min(std::min(t_max_x, t_max_y), t_exit);
      if (exited_vehicle) {
        last_free_r = cell_exit_t;
      }

      if (t_max_x < t_max_y) {
        gx += step_x;
        travel_t = t_max_x;
        t_max_x += t_delta_x;
      } else if (t_max_y < t_max_x) {
        gy += step_y;
        travel_t = t_max_y;
        t_max_y += t_delta_y;
      } else {
        gx += step_x;
        gy += step_y;
        travel_t = t_max_x;
        t_max_x += t_delta_x;
        t_max_y += t_delta_y;
      }
    }

    if (last_free_r > 0.0) {
      ray_state->ray_ranges[static_cast<size_t>(ray_id)] =
          std::min(last_free_r, ray_state->max_range);
      ray_state->ray_valid[static_cast<size_t>(ray_id)] = 1;
    }
  }
}

void FilterRayOutliers(FreespaceRayState* ray_state) {
  if (nullptr == ray_state) {
    return;
  }
  constexpr int kHalfWin = 14;
  constexpr int kMaxOutlierRun = 3;
  constexpr double kMinAbsOutlier = 1.0;
  constexpr double kIqrScale = 1.5;

  std::vector<double> local_med(static_cast<size_t>(ray_state->ray_count), 0.0);
  std::vector<double> local_low(static_cast<size_t>(ray_state->ray_count), 0.0);
  std::vector<double> local_high(static_cast<size_t>(ray_state->ray_count), 0.0);
  std::vector<double> win;
  win.reserve(static_cast<size_t>(kHalfWin * 2));
  for (int i = 0; i < ray_state->ray_count; ++i) {
    win.clear();
    for (int d = -kHalfWin; d <= kHalfWin; ++d) {
      if (0 == d) {
        continue;
      }
      const double r = ray_state->ray_ranges[static_cast<size_t>(
          (i + d + ray_state->ray_count) % ray_state->ray_count)];
      if (r > 1e-3) {
        win.emplace_back(r);
      }
    }
    if (win.size() < 6) {
      continue;
    }
    std::sort(win.begin(), win.end());
    const size_t n = win.size();
    const double q1 = win[n / 4];
    const double med = win[n / 2];
    const double q3 = win[(3 * n) / 4];
    const double iqr = std::max(0.2, q3 - q1);
    local_med[static_cast<size_t>(i)] = med;
    local_low[static_cast<size_t>(i)] =
        std::max(0.0, q1 - kIqrScale * iqr - 0.2);
    local_high[static_cast<size_t>(i)] = q3 + kIqrScale * iqr + 0.2;
  }

  std::vector<bool> is_outlier(static_cast<size_t>(ray_state->ray_count), false);
  for (int i = 0; i < ray_state->ray_count; ++i) {
    const double r = ray_state->ray_ranges[static_cast<size_t>(i)];
    const double med = local_med[static_cast<size_t>(i)];
    if (r <= 1e-3 || med <= 1e-3) {
      continue;
    }
    const double low = local_low[static_cast<size_t>(i)];
    const double high = local_high[static_cast<size_t>(i)];
    if ((r < low || r > high) && std::fabs(r - med) > kMinAbsOutlier) {
      is_outlier[static_cast<size_t>(i)] = true;
    }
  }

  int start = 0;
  for (int i = 0; i < ray_state->ray_count; ++i) {
    if (!is_outlier[static_cast<size_t>(i)]) {
      start = i;
      break;
    }
  }

  for (int ci = 0; ci < ray_state->ray_count;) {
    const int idx = (start + ci) % ray_state->ray_count;
    if (!is_outlier[static_cast<size_t>(idx)]) {
      ++ci;
      continue;
    }
    int run_len = 0;
    while (run_len < ray_state->ray_count &&
           is_outlier[static_cast<size_t>(
               (start + ci + run_len) % ray_state->ray_count)]) {
      ++run_len;
    }
    if (run_len <= kMaxOutlierRun) {
      const int before_ray = (start + ci - 1 + ray_state->ray_count) %
                             ray_state->ray_count;
      const int after_ray =
          (start + ci + run_len) % ray_state->ray_count;
      const double rb = ray_state->ray_ranges[static_cast<size_t>(before_ray)];
      const double ra = ray_state->ray_ranges[static_cast<size_t>(after_ray)];
      if (rb > 1e-3 && ra > 1e-3) {
        const double tb =
            -common::Constant<double>::PI() +
            static_cast<double>(before_ray) * ray_state->angular_step;
        const double ta =
            -common::Constant<double>::PI() +
            static_cast<double>(after_ray) * ray_state->angular_step;
        const double xb = ray_state->source_x + rb * std::cos(tb);
        const double yb = ray_state->source_y + rb * std::sin(tb);
        const double xa = ray_state->source_x + ra * std::cos(ta);
        const double ya = ray_state->source_y + ra * std::sin(ta);
        for (int k = 0; k < run_len; ++k) {
          const size_t s = static_cast<size_t>(
              (start + ci + k) % ray_state->ray_count);
          if (std::hypot(xa - xb, ya - yb) < 12.0) {
            const double alpha =
                static_cast<double>(k + 1) / static_cast<double>(run_len + 1);
            ray_state->ray_ranges[s] = (1.0 - alpha) * rb + alpha * ra;
            ray_state->ray_valid[s] = 1;
          } else {
            ray_state->ray_ranges[s] = 0.0;
            ray_state->ray_valid[s] = 0;
          }
        }
      }
    }
    ci += run_len;
  }
}

void CloseNarrowRayGaps(FreespaceRayState* ray_state) {
  if (nullptr == ray_state) {
    return;
  }
  constexpr double kMinPassableGap = 3.0;
  const auto is_open_run_ray = [&](const int ray_id) {
    return ray_state->ray_valid[static_cast<size_t>(ray_id)] != 0 &&
           ray_state->ray_hit_occupied[static_cast<size_t>(ray_id)] == 0;
  };

  int start = 0;
  for (int i = 0; i < ray_state->ray_count; ++i) {
    if (!is_open_run_ray(i)) {
      start = i;
      break;
    }
  }

  for (int ci = 0; ci < ray_state->ray_count;) {
    const int idx = (start + ci) % ray_state->ray_count;
    if (!is_open_run_ray(idx)) {
      ++ci;
      continue;
    }
    int run_len = 0;
    while (run_len < ray_state->ray_count &&
           is_open_run_ray((start + ci + run_len) % ray_state->ray_count)) {
      ++run_len;
    }
    const int before_ray = (start + ci - 1 + ray_state->ray_count) %
                           ray_state->ray_count;
    const int after_ray = (start + ci + run_len) % ray_state->ray_count;
    if (ray_state->ray_hit_occupied[static_cast<size_t>(before_ray)] &&
        ray_state->ray_hit_occupied[static_cast<size_t>(after_ray)]) {
      const double dx =
          ray_state->ray_hit_occ_x[static_cast<size_t>(after_ray)] -
          ray_state->ray_hit_occ_x[static_cast<size_t>(before_ray)];
      const double dy =
          ray_state->ray_hit_occ_y[static_cast<size_t>(after_ray)] -
          ray_state->ray_hit_occ_y[static_cast<size_t>(before_ray)];
      if (std::hypot(dx, dy) < kMinPassableGap) {
        for (int k = 0; k < run_len; ++k) {
          const size_t s = static_cast<size_t>(
              (start + ci + k) % ray_state->ray_count);
          ray_state->ray_ranges[s] = 0.0;
          ray_state->ray_valid[s] = 0;
        }
      }
    }
    ci += run_len;
  }
}

void AppendBoundaryPoints(const FreespaceRayState& ray_state,
                          century::common::Polyline* output_polyline,
                          const int from, const int to, const int step,
                          const double ground_z) {
  if (nullptr == output_polyline) {
    return;
  }
  for (int i = from; i != to + step; i += step) {
    const double range = ray_state.ray_ranges[static_cast<size_t>(i)];
    if (range <= 1e-3) {
      continue;
    }
    const double theta = -common::Constant<double>::PI() +
                         static_cast<double>(i) * ray_state.angular_step;
    auto* pt = output_polyline->add_point();
    pt->set_x(ray_state.source_x + range * std::cos(theta));
    pt->set_y(ray_state.source_y + range * std::sin(theta));
    pt->set_z(ground_z);
  }
}

bool WriteFreespaceDebugOutput(const lidar::LidarFrame& frame,
                               const FreespaceGridContext& context,
                               const FreespaceRayState& ray_state,
                               PerceptionObstacleDebugMsg* debug_msg) {
  if (nullptr == debug_msg) {
    return false;
  }
  auto* polygon = debug_msg->add_freespace_mask_polygon();
  for (int i = 0; i < ray_state.ray_count; ++i) {
    const double range = ray_state.ray_ranges[static_cast<size_t>(i)];
    if (range <= 1e-3) {
      continue;
    }
    const double theta = -common::Constant<double>::PI() +
                         static_cast<double>(i) * ray_state.angular_step;
    auto* point = polygon->add_point();
    point->set_x(ray_state.source_x + range * std::cos(theta));
    point->set_y(ray_state.source_y + range * std::sin(theta));
    point->set_z(context.ground_z);
  }
  if (polygon->point_size() < 3) {
    debug_msg->mutable_freespace_mask_polygon()->RemoveLast();
    return false;
  }

  const int half = ray_state.ray_count / 2;
  const bool travel_forward = frame.corridor_forward_sign >= 0;
  auto* left_boundary = debug_msg->mutable_freespace_left_boundary();
  auto* right_boundary = debug_msg->mutable_freespace_right_boundary();
  if (travel_forward) {
    AppendBoundaryPoints(ray_state, right_boundary, 1, half - 1, 1,
                         context.ground_z);
    AppendBoundaryPoints(ray_state, left_boundary, ray_state.ray_count - 1,
                         half + 1, -1, context.ground_z);
  } else {
    AppendBoundaryPoints(ray_state, left_boundary, half - 1, 1, -1,
                         context.ground_z);
    AppendBoundaryPoints(ray_state, right_boundary, half + 1,
                         ray_state.ray_count - 1, 1, context.ground_z);
  }
  if (debug_msg->freespace_right_boundary().point_size() < 2) {
    debug_msg->clear_freespace_right_boundary();
  }
  if (debug_msg->freespace_left_boundary().point_size() < 2) {
    debug_msg->clear_freespace_left_boundary();
  }
  return true;
}

bool HasCompressedImage(const CameraInputData& input) {
  return nullptr != input.compressed_image;
}

bool HasRawImage(const CameraInputData& input) {
  return nullptr != input.raw_image;
}
}  // namespace

LidarDetectorComponent::~LidarDetectorComponent() {
  pointcloud_msg_queue_.reset();
  merged_msg_queue_.reset();
  around_vehicle_queue_.reset();
  for (auto& stream : camera_streams_) {
    if (nullptr != stream) {
      cudaStreamDestroy(stream);
      stream = nullptr;
    }
  }
}

bool LidarDetectorComponent::LoadComponentConfig(
    const LidarDetectionComponentConfig& comp_config) noexcept {
  AINFO << "Perception Version: " << comp_config.version();
  AINFO << "Lidar Component Configs: " << comp_config.DebugString();
  use_viz_debug_ = comp_config.use_viz_debug();
  use_filter_bank_ = comp_config.use_filter_bank();
  use_pose_query_ = comp_config.use_pose_query();
  pose_query_offset_ = comp_config.pose_query_offset();
  use_point_interpolation_ = comp_config.use_point_interpolation();
  enable_freespace_mask_ = comp_config.enable_freespace_mask();
  freespace_mask_resolution_ = comp_config.freespace_mask_resolution();
  freespace_mask_x_min_ = comp_config.freespace_mask_x_min();
  freespace_mask_x_max_ = comp_config.freespace_mask_x_max();
  freespace_mask_y_min_ = comp_config.freespace_mask_y_min();
  freespace_mask_y_max_ = comp_config.freespace_mask_y_max();
  freespace_obstacle_inflate_ = comp_config.freespace_obstacle_inflate();
  freespace_use_hdmap_road_ = comp_config.freespace_use_hdmap_road();
  freespace_min_points_per_cell_ =
      std::max(1, comp_config.freespace_min_points_per_cell());
  freespace_show_ray_source_ = comp_config.freespace_show_ray_source();
  use_camera_ = comp_config.use_camera();
  image_format_ = comp_config.image_format();
  freespace_use_cuda_ = comp_config.freespace_use_cuda();
  freespace_enable_temporal_filter_ =
      comp_config.freespace_enable_temporal_filter();
  freespace_temporal_expand_alpha_ =
      std::max(0.0, std::min(1.0,
                             comp_config.freespace_temporal_expand_alpha()));
  freespace_temporal_max_expand_ =
      std::max(0.0, comp_config.freespace_temporal_max_expand());
  freespace_temporal_source_shift_reset_ =
      std::max(0.0, comp_config.freespace_temporal_source_shift_reset());
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
  post_detection_frame_output_channel_name_ =
      comp_config.post_detection_frame_output_channel_name();
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
  use_map_manager_ = comp_config.use_map_manager() && enable_hdmap_;
  return true;
}

bool LidarDetectorComponent::InitIoResources(
    const LidarDetectionComponentConfig& comp_config) noexcept {
  perception_writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_channel_name_);
  around_ego_perception_writer_ =
      node_->CreateWriter<PerceptionObstacles>(around_ego_output_channel_name_);
  perception_debug_writer_ = node_->CreateWriter<PerceptionObstacleDebugMsg>(
      debug_output_channel_name_);
  perception_debug_around_writer_ =
      node_->CreateWriter<PerceptionObstacleDebugMsg>(
          debug_around_output_channel_name_);
  if (comp_config.publish_post_detection_frame() &&
      !post_detection_frame_output_channel_name_.empty()) {
    post_detection_frame_writer_ =
        node_->CreateWriter<LidarFrameMessage>(
            post_detection_frame_output_channel_name_);
  }

  auto& input_channel = comp_config.input_channel_name();
  input_channels_.clear();
  lidar_readers_.clear();
  input_channels_.reserve(input_channel.size());
  lidar_readers_.reserve(input_channel.size());
  for (const auto& channel : input_channel) {
    input_channels_.emplace_back(channel);
    lidar_readers_.emplace_back(
        node_->CreateReader<LidarFrameMessage>(channel));
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
  constexpr int kLocalizationHistoryDepth = 250;
  localization_reader_->SetHistoryDepth(kLocalizationHistoryDepth);
  return true;
}

bool LidarDetectorComponent::InitCameraResources() noexcept {
  if (!use_camera_) {
    return true;
  }

  common::CameraSensorConfig& camera_sensor_config =
      common::CameraSensorConfig::GetInstance();
  camera_sensor_config.Initialize(
      "/century/modules/perception/data/params/camera_sensor.yaml");
  if (!camera_sensor_config.IsInitialized()) {
    AERROR << "Camera sensor config is not initialized";
    return false;
  }

  camera_channels_.clear();
  jpeg_decoders_.clear();
  compressed_image_readers_.clear();
  raw_image_readers_.clear();
  camera_streams_.clear();

  std::vector<std::vector<double>> camera_intrinsics_vector;
  for (const auto& camera_name : camera_sensor_config.GetCameraNames()) {
    std::string channel;
    if (!camera_sensor_config.GetCameraChannel(camera_name, channel)) {
      continue;
    }
    camera_channels_.emplace_back(channel);
    camera_intrinsics_vector.emplace_back(
        camera_sensor_config.GetCameraIntrinsicsVector(camera_name));
  }
  if (camera_channels_.size() != kCameraCount) {
    AERROR << "Expected " << kCameraCount << " camera channels, got "
           << camera_channels_.size();
    return false;
  }

  time_window_synchronizer_ptr_ = std::make_unique<TimeWindowSynchronizer>();
  dmapx_.resize(kCameraCount);
  dmapy_.resize(kCameraCount);

  auto camera_names = camera_sensor_config.GetCameraNames();
  if (!camera_names.empty()) {
    common::CameraParms params;
    if (camera_sensor_config.GetCameraConfig(camera_names[0], params)) {
      image_width_ = params.width;
      image_height_ = params.height;
    }
  }

  for (int i = 0; i < kCameraCount; ++i) {
    cv::Mat k =
        (cv::Mat_<double>(3, 3) << camera_intrinsics_vector[i][0], 0,
         camera_intrinsics_vector[i][2], 0, camera_intrinsics_vector[i][1],
         camera_intrinsics_vector[i][3], 0, 0, 1);
    cv::Mat d = (cv::Mat_<double>(4, 1) << camera_intrinsics_vector[i][4],
                 camera_intrinsics_vector[i][5], camera_intrinsics_vector[i][6],
                 camera_intrinsics_vector[i][7]);
    cv::Mat new_k;
    cv::Mat mapx;
    cv::Mat mapy;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        k, d, {image_width_, image_height_}, cv::Matx33d::eye(), new_k);
    cv::fisheye::initUndistortRectifyMap(k, d, cv::Matx33d::eye(), new_k,
                                         {image_width_, image_height_},
                                         CV_32FC1, mapx, mapy);

    const int total = image_width_ * image_height_;
    dmapx_[i].resize(total);
    dmapy_[i].resize(total);
    std::memcpy(dmapx_[i].data(), mapx.ptr<float>(), total * sizeof(float));
    std::memcpy(dmapy_[i].data(), mapy.ptr<float>(), total * sizeof(float));
  }

  int device_id = 0;
  cudaSetDevice(device_id);
  int least_priority = 0;
  int greatest_priority = 0;
  cudaDeviceGetStreamPriorityRange(&least_priority, &greatest_priority);
  camera_streams_.resize(camera_channels_.size());
  for (size_t i = 0; i < camera_channels_.size(); ++i) {
    cudaStreamCreateWithPriority(&camera_streams_[i], cudaStreamNonBlocking,
                                 least_priority);
  }

  for (size_t i = 0; i < camera_channels_.size(); ++i) {
    auto decoder = std::make_unique<century::perception::camera::NvJpegDecoder>(
        camera_channels_[i], 0, image_width_, image_height_, 3, true);
    decoder->SetDMap(dmapx_[i], dmapy_[i]);
    decoder->SetExternalStream(camera_streams_[i]);
    jpeg_decoders_.emplace_back(std::move(decoder));

    if ("raw" == image_format_) {
      raw_image_readers_.emplace_back(node_->CreateReader<drivers::Image>(
          camera_channels_[i],
          [this, i](const std::shared_ptr<drivers::Image>& msg) {
            this->OnReceiveRawImage(msg, static_cast<int>(i));
          }));
    } else {
      compressed_image_readers_.emplace_back(
          node_->CreateReader<drivers::CompressedImage>(
              camera_channels_[i],
              [this, i](const std::shared_ptr<drivers::CompressedImage>& msg) {
                this->OnReceiveCompressedImage(msg, static_cast<int>(i));
              }));
    }
  }
  return true;
}

void LidarDetectorComponent::StartBackgroundTasks() noexcept {
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
  if (!LoadComponentConfig(comp_config)) {
    return false;
  }
  if (!InitIoResources(comp_config)) {
    return false;
  }
  if (!InitCameraResources()) {
    return false;
  }
  lidar::SceneManagerInitOptions scene_manager_init_options;
  ACHECK(lidar::SceneManager::Instance().Init(scene_manager_init_options));

  auto ret = InitAlgorithmPlugin();
  if (!ret) {
    AERROR << "Init algorithm plugin failed.";
    return false;
  }

  lidar::ObjectBuilderInitOptions builder_init_options;
  ACHECK(builder_.Init(builder_init_options));
  StartBackgroundTasks();
  return true;
}

void LidarDetectorComponent::OnReceiveCompressedImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image,
    int index) noexcept {
  if (!use_camera_ || nullptr == time_window_synchronizer_ptr_ ||
      nullptr == compressed_image) {
    return;
  }
  time_window_synchronizer_ptr_->PushCompressed(index, compressed_image);
}

void LidarDetectorComponent::OnReceiveRawImage(
    const std::shared_ptr<drivers::Image>& raw_image, int index) noexcept {
  if (!use_camera_ || nullptr == time_window_synchronizer_ptr_ ||
      nullptr == raw_image) {
    return;
  }
  time_window_synchronizer_ptr_->PushRaw(index, raw_image);
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
  auto get_sensor_name = [](const LidarFrameMessagePtr& msg) -> std::string {
    if (nullptr == msg || nullptr == msg->lidar_frame_) {
      return "unknown_lidar";
    }
    return msg->lidar_frame_->sensor_info.name.empty()
               ? "unknown_lidar"
               : msg->lidar_frame_->sensor_info.name;
  };

  pointcloud_msg_queue_->Pop(&msg_ptr);
  if (nullptr == msg_ptr || msg_ptr->empty()) {
    AERROR << "No messages to synchronize.";
    return nullptr;
  }

  for (const auto& msg : *msg_ptr) {
    if (nullptr == msg || nullptr == msg->lidar_frame_) {
      AERROR << "Invalid message or cloud in synchronized messages.";
      continue;
    }
    synchronized_msgs.emplace_back(msg);
  }

  double earliest_timestamp = std::numeric_limits<double>::max();
  LidarFrameMessagePtr earliest_msg = nullptr;
  for (const auto& msg_item : synchronized_msgs) {
    if (msg_item->timestamp_ < earliest_timestamp) {
      earliest_timestamp = msg_item->timestamp_;
      earliest_msg = msg_item;
    }
  }

  if (std::numeric_limits<double>::max() == earliest_timestamp) {
    AERROR << "No valid messages in the queues: " << synchronized_msgs.size();
    return nullptr;
  }

  for (auto& msg : synchronized_msgs) {
    double time_diff = std::abs(msg->timestamp_ - earliest_timestamp);
    if (time_diff > tolerance) {
      AWARN << std::fixed << std::setprecision(6)
            << "Lidar sync failed between " << get_sensor_name(earliest_msg)
            << " (ts=" << earliest_timestamp << ") and " << get_sensor_name(msg)
            << " (ts=" << msg->timestamp_ << "), diff=" << time_diff
            << ", tolerance=" << tolerance;
      return nullptr;
    }
  }

  if (synchronized_msgs.size() != lidar_readers_.size()) {
    AERROR << "Not all channels are synchronized. Expected: "
           << ", Got: " << synchronized_msgs.size();
    return nullptr;
  }

  auto merged_message = std::make_shared<LidarFrameMessage>();
  if (nullptr == merged_message->lidar_frame_) {
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
    if (nullptr == msg || nullptr == msg->lidar_frame_) {
      AERROR << "Invalid message or cloud in synchronized messages.";
      continue;
    }
    auto non_ground_indices = msg->lidar_frame_->non_ground_indices.indices;
    auto points_num = merged_message->lidar_frame_->raw_cloud->points().size();

    for (const auto& index : non_ground_indices) {
      merged_message->lidar_frame_->non_ground_indices.indices.emplace_back(
          points_num + index);
    }

    *merged_message->lidar_frame_->cloud += *msg->lidar_frame_->cloud;

    *merged_message->lidar_frame_->raw_cloud += *msg->lidar_frame_->raw_cloud;
    *merged_message->lidar_frame_->ego_cloud += *msg->lidar_frame_->ego_cloud;

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
  merged_message->lidar_frame_->hdmap_struct =
      synchronized_msgs[0]->lidar_frame_->hdmap_struct;

  // Propagate ground-z from the first sensor frame that has a valid value so
  // that GetGroundZ() returns the correct reference for the merged frame.
  // (The default is 10.0f which GetGroundZ filters out → falls back to 0.0.)
  for (const auto& msg : synchronized_msgs) {
    if (nullptr == msg || nullptr == msg->lidar_frame_) {
      continue;
    }
    const float gz = msg->lidar_frame_->original_ground_z;
    if (std::isfinite(gz) && std::fabs(gz) < 5.0f) {
      merged_message->lidar_frame_->original_ground_z = gz;
      break;
    }
  }

  merged_message->timestamp_ = earliest_timestamp;
  merged_message->lidar_timestamp_ = synchronized_msgs[0]->lidar_timestamp_;
  merged_message->seq_num_ = synchronized_msgs[0]->seq_num_;
  merged_message->type_name_ = synchronized_msgs[0]->type_name_;
  merged_message->process_stage_ = synchronized_msgs[0]->process_stage_;
  merged_message->error_code_ = synchronized_msgs[0]->error_code_;
  merged_message->lidar_frame_->sensor_info =
      synchronized_msgs[0]->lidar_frame_->sensor_info;
  merged_message->lidar_frame_->lidar2world_pose =
      synchronized_msgs[0]->lidar_frame_->lidar2world_pose;
  merged_message->lidar_frame_->novatel2world_pose =
      synchronized_msgs[0]->lidar_frame_->novatel2world_pose;
  merged_message->lidar_frame_->vehicle2imu_pose =
      synchronized_msgs[0]->lidar_frame_->vehicle2imu_pose;

  AINFO << "Merged point cloud size: "
        << merged_message->lidar_frame_->raw_cloud->size() << ", from "
        << synchronized_msgs.size() << " sensors.";

  return merged_message;
}

void LidarDetectorComponent::SyncPointCloudMsgTask() noexcept {
  const double kTolerance = 0.1;
  while (century::cyber::OK()) {
    auto merged_msg = SyncAndMergePointCloudMessages(kTolerance);
    if (nullptr == merged_msg) {
      continue;
    }

    merged_msg_queue_->PushBack(merged_msg);
  }
}

void LidarDetectorComponent::ProcessLidarFrameAroundVehicle() noexcept {
  while (century::cyber::OK()) {
    LidarFrameMessagePtr frame_msg;
    around_vehicle_queue_->Pop(&frame_msg);

    if (nullptr == frame_msg || nullptr == frame_msg->lidar_frame_) {
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

    frame_msg->lidar_frame_->segmented_objects.insert(
        frame_msg->lidar_frame_->segmented_objects.end(), det_objects.begin(),
        det_objects.end());
    if (perception_debug_around_writer_->HasReader() && use_viz_debug_) {
      const auto& localization_msg = localization_reader_->GetLatestObserved();
      if (nullptr == localization_msg) {
        AERROR << "No localization message received.";
        continue;
      }
      PublishPerceptionObstacles(perception_debug_around_writer_, frame_msg,
                                 localization_msg->header().timestamp_sec());
    }
    PERCEPTION_PERF_BLOCK_END("Around Vehicle PointCloud Cluster");

    constexpr float kDuplicatedDistanceThres = 1.0f;
    lidar::DuplicatedObjectFilter duplicate_filter(kDuplicatedDistanceThres);

    Eigen::Vector3d add_pt;
    for (const auto& iter : frame_msg->lidar_frame_->segmented_objects) {
      if (iter->type != base::ObjectType::UNKNOWN) {
        add_pt = Eigen::Vector3d(iter->center(0), iter->center(1), iter->theta);
        duplicate_filter.AddPoint(
            add_pt, iter->size(0), iter->size(1),
            base::ObjectType::PEDESTRIAN == iter->type);
      }
    }

    Eigen::Vector2d check_pt;
    auto& vec = frame_msg->lidar_frame_->segmented_objects;
    vec.erase(
        std::remove_if(vec.begin(), vec.end(),
                       [&](century::perception::base::ObjectPtr iter) {
                         check_pt = Eigen::Vector2d(iter->center(0),
                                                    iter->center(1));
                         return ((base::ObjectType::UNKNOWN == iter->type) &&
                                 (duplicate_filter.IsDuplicated(check_pt)));
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
        perception_obstacles->mutable_perception_obstacle(i)->set_id(seq_num);
        ++seq_num;
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
      if (enable_freespace_mask_) {
        PerceptionObstacleDebugMsg freespace_debug_msg;
        BuildFreespaceMaskPolygon(frame_msg, &freespace_debug_msg);
        FillFreespaceIntoPerceptionObstacles(freespace_debug_msg,
                                             lidar2world_pose,
                                             perception_obstacles.get());
      }
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

  constexpr int kMaxRetryCount = 20;
  constexpr int kRetryIntervalMs = 5;
  auto block_read_lidar_msg = [&](auto reader, auto msg_vec) -> bool {
    int retry_count = kMaxRetryCount;
    while (0 < retry_count) {
      --retry_count;
      reader->Observe();
      auto lidar_frame = reader->GetLatestObserved();
      if (nullptr != lidar_frame) {
        msg_vec->emplace_back(lidar_frame);
        reader->ClearData();
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kRetryIntervalMs));
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
      ++count;
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
    if (nullptr == localization_msg) {
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
        const size_t points_num = merged_msg->lidar_frame_->cloud->size();
        if (use_point_interpolation_) {
          Eigen::Vector3d vec3d_lidar;
          Eigen::Affine3d pose;
          Eigen::Vector3d vec3d_world;
          base::PointD world_point;
          Eigen::Vector3d vec3d_local;
          for (size_t i = 0; i < points_num; ++i) {
            auto& point =
                (*merged_msg->lidar_frame_->raw_cloud->mutable_points())[i];
            vec3d_lidar = Eigen::Vector3d(point.x, point.y, point.z);
            auto point_timestamp =
                merged_msg->lidar_frame_->raw_cloud->points_timestamp()[i];

            pose = Eigen::Affine3d::Identity();
            double point_pose_time = 0.0;
            if (!lidar::GetNearest(merged_msg->lidar_frame_->localizetion_poses,
                                   point_timestamp, pose, point_pose_time,
                                   false)) {
              AERROR << std::fixed << std::setprecision(6)
                     << "Failed to get point pose at time: " << point_timestamp
                     << " : " << point_pose_time;
            }
            vec3d_world =
                pose * Eigen::Affine3d(imu2vehicle_matrix) * vec3d_lidar;
            world_point.x = vec3d_world(0);
            world_point.y = vec3d_world(1);
            world_point.z = vec3d_world(2);
            vec3d_local =
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

    UpdateFreespaceMotionState(localization_msg, merged_msg);

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
    if (nullptr != post_detection_frame_writer_) {
      post_detection_frame_writer_->Write(merged_msg);
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
      if (enable_freespace_mask_) {
        PerceptionObstacleDebugMsg freespace_debug_msg;
        BuildFreespaceMaskPolygon(merged_msg, &freespace_debug_msg);
        FillFreespaceIntoPerceptionObstacles(freespace_debug_msg,
                                             lidar2world_pose,
                                             perception_obstacles.get());
      }
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

  if (use_camera_) {
    if (nullptr == time_window_synchronizer_ptr_) {
      AERROR << "Camera synchronizer is not initialized.";
      return;
    }
    std::array<CameraInputData, kCameraCount> camera_inputs;
    std::array<bool, kCameraCount> camera_available_mask;
    camera_available_mask.fill(true);
    bool use_black_image_fallback = false;
    if (!time_window_synchronizer_ptr_->Query(frame->timestamp_, camera_inputs)) {
      AERROR << std::fixed << std::setprecision(6)
             << "Failed to get synchronized image data for lidar detection. "
             << "lidar_frame_ts=" << frame->timestamp_
             << ", fallback to black images for missing cameras.";
      time_window_synchronizer_ptr_->BuildFallbackInput(
          frame->timestamp_, camera_inputs, camera_available_mask);
      use_black_image_fallback = true;

      std::ostringstream black_camera_stream;
      std::ostringstream ready_camera_stream;
      bool first_black = true;
      bool first_ready = true;
      for (int i = 0; i < kCameraCount; ++i) {
        std::ostringstream& target_stream =
            camera_available_mask[i] ? ready_camera_stream : black_camera_stream;
        bool& first_flag = camera_available_mask[i] ? first_ready : first_black;
        if (!first_flag) {
          target_stream << ",";
        }
        first_flag = false;
        target_stream << i;
      }
      AWARN << std::fixed << std::setprecision(6)
            << "[CameraSyncFallback] lidar_frame_ts=" << frame->timestamp_
            << " black_image_cameras=["
            << (first_black ? "none" : black_camera_stream.str())
            << "] ready_cameras=["
            << (first_ready ? "none" : ready_camera_stream.str()) << "]";
    }

    frame->lidar_frame_->camera_ready_events.clear();
    frame->lidar_frame_->camera_stream_ptrs.clear();
    for (int i = 0; i < kCameraCount; ++i) {
      const bool camera_available = camera_available_mask[i];
      if (use_black_image_fallback && !camera_available) {
        if (!jpeg_decoders_[i]->FillBlackImage()) {
          AERROR << "Failed to fill black image for missing camera " << i;
          return;
        }
        AERROR << "Success to fill black image for missing camera " << i;
        frame->lidar_frame_->camera_data[i] = jpeg_decoders_[i]->GetDeviceBuffer();
        frame->lidar_frame_->camera_stream_ptrs.emplace_back(
            reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetStream()));
        frame->lidar_frame_->camera_ready_events.emplace_back(
            reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetReadyEvent()));
        continue;
      }

      if ("raw" == image_format_) {
        if (!HasRawImage(camera_inputs[i])) {
          if (!jpeg_decoders_[i]->FillBlackImage()) {
            AERROR << "Missing raw image for camera " << i
                   << " and failed to fill black image.";
            return;
          }
          frame->lidar_frame_->camera_data[i] = jpeg_decoders_[i]->GetDeviceBuffer();
          frame->lidar_frame_->camera_stream_ptrs.emplace_back(
              reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetStream()));
          frame->lidar_frame_->camera_ready_events.emplace_back(
              reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetReadyEvent()));
          continue;
        }
        const auto& raw_image = camera_inputs[i].raw_image;
        int width = static_cast<int>(raw_image->width());
        int height = static_cast<int>(raw_image->height());
        std::string encoding = raw_image->encoding();
        if (!jpeg_decoders_[i]->DecodeRaw(
                reinterpret_cast<const uint8_t*>(raw_image->data().data()),
                raw_image->data().size(), width, height, encoding)) {
          AERROR << "Failed to process raw image for camera " << i
                 << " with encoding: " << encoding;
          return;
        }
      } else {
        if (!HasCompressedImage(camera_inputs[i])) {
          if (!jpeg_decoders_[i]->FillBlackImage()) {
            AERROR << "Missing compressed image for camera " << i
                   << " and failed to fill black image.";
            return;
          }
          frame->lidar_frame_->camera_data[i] = jpeg_decoders_[i]->GetDeviceBuffer();
          frame->lidar_frame_->camera_stream_ptrs.emplace_back(
              reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetStream()));
          frame->lidar_frame_->camera_ready_events.emplace_back(
              reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetReadyEvent()));
          continue;
        }
        int width = 0;
        int height = 0;
        if (!jpeg_decoders_[i]->Decode(
                reinterpret_cast<const uint8_t*>(
                    camera_inputs[i].compressed_image->data().data()),
                camera_inputs[i].compressed_image->data().size(), &width,
                &height)) {
          AERROR << "Failed to decode JPEG image for camera " << i;
          return;
        }
      }

      frame->lidar_frame_->camera_data[i] = jpeg_decoders_[i]->GetDeviceBuffer();
      frame->lidar_frame_->camera_stream_ptrs.emplace_back(
          reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetStream()));
      frame->lidar_frame_->camera_ready_events.emplace_back(
          reinterpret_cast<uintptr_t>(jpeg_decoders_[i]->GetReadyEvent()));
    }

    for (auto& stream : camera_streams_) {
      cudaStreamSynchronize(stream);
    }
  }

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

  if (!FLAGS_dump_world_pcl.empty()) {
    DumpWorldPcl(merged_msg->timestamp_, merged_msg->lidar_frame_->cloud,
                 merged_msg->lidar_frame_->lidar2world_pose);
  }

  return true;
}

void LidarDetectorComponent::UpdateFreespaceMotionState(
    const std::shared_ptr<century::localization::LocalizationEstimate>&
        localization_msg,
    LidarFrameMessagePtr merged_msg) noexcept {
  if (nullptr == localization_msg || nullptr == merged_msg ||
      nullptr == merged_msg->lidar_frame_) {
    return;
  }

  auto* frame = merged_msg->lidar_frame_.get();
  frame->motion_direction_world = Eigen::Vector2d::Zero();
  frame->motion_speed_world_mps = 0.0;
  frame->has_motion_direction_world = false;
  frame->vehicle_forward_direction_world = Eigen::Vector2d::Zero();
  frame->has_vehicle_forward_direction_world = false;
  frame->corridor_forward_sign = 1;

  const auto& pose = localization_msg->pose();
  const Eigen::Vector2d velocity_world(pose.linear_velocity().x(),
                                       pose.linear_velocity().y());
  const double speed_world = velocity_world.norm();
  frame->motion_speed_world_mps = speed_world;
  if (speed_world > 1e-3) {
    frame->motion_direction_world = velocity_world / speed_world;
    frame->has_motion_direction_world = true;
  }

  const Eigen::Vector3d local_velocity =
      frame->lidar2world_pose.linear().inverse() *
      Eigen::Vector3d(velocity_world.x(), velocity_world.y(), 0.0);
  if (std::fabs(local_velocity.x()) > 0.2) {
    frame->corridor_forward_sign = local_velocity.x() >= 0.0 ? 1 : -1;
  }

  const Eigen::Vector3d forward_world =
      frame->lidar2world_pose.linear() * Eigen::Vector3d::UnitX();
  const Eigen::Vector2d forward_world_xy(forward_world.x(), forward_world.y());
  if (forward_world_xy.norm() > 1e-3) {
    frame->vehicle_forward_direction_world =
        forward_world_xy / forward_world_xy.norm();
    frame->has_vehicle_forward_direction_world = true;
  }
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
  init_options.use_camera = use_camera_;

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
    AINFO << "filter_bank sensor_name: " << sensor_name_;
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

bool LidarDetectorComponent::BuildFreespaceMaskPolygon(
    const LidarFrameMessagePtr& merged_msg,
    PerceptionObstacleDebugMsg* debug_msg) noexcept {
  if (nullptr != debug_msg) {
    debug_msg->clear_freespace_mask_polygon();
    debug_msg->clear_freespace_left_boundary();
    debug_msg->clear_freespace_right_boundary();
    debug_msg->clear_freespace_ray_source();
  }
  if (nullptr == debug_msg || nullptr == merged_msg ||
      nullptr == merged_msg->lidar_frame_ ||
      (nullptr == merged_msg->lidar_frame_->raw_cloud &&
       nullptr == merged_msg->lidar_frame_->cloud)) {
    return false;
  }

  const auto& frame = *merged_msg->lidar_frame_;
  if (nullptr == frame.raw_cloud || frame.raw_cloud->empty()) {
    return false;
  }
  lidar::FreespaceBuildOptions options;
  options.spec = {freespace_mask_x_min_, freespace_mask_x_max_,
                  freespace_mask_y_min_, freespace_mask_y_max_,
                  freespace_mask_resolution_};
  options.min_points_per_cell = std::max(1, freespace_min_points_per_cell_);
  options.obstacle_inflate = std::max(0.0, freespace_obstacle_inflate_);
  options.use_hdmap_road = freespace_use_hdmap_road_;
  options.show_ray_source = freespace_show_ray_source_;
  options.use_cuda = freespace_use_cuda_;
  options.enable_temporal_filter = freespace_enable_temporal_filter_;
  options.temporal_expand_alpha = freespace_temporal_expand_alpha_;
  options.temporal_max_expand = freespace_temporal_max_expand_;
  options.temporal_source_shift_reset =
      freespace_temporal_source_shift_reset_;

  return lidar::BuildFreespaceMask(frame, options, debug_msg,
                                   &freespace_temporal_state_);
}

void LidarDetectorComponent::FillFreespaceIntoPerceptionObstacles(
    const PerceptionObstacleDebugMsg& debug_msg,
    const Eigen::Affine3d& lidar2world_pose,
    PerceptionObstacles* perception_obstacles) const noexcept {
  lidar::CopyFreespaceToObstaclesWorld(debug_msg, lidar2world_pose,
                                       perception_obstacles);
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
    if (enable_freespace_mask_) {
      BuildFreespaceMaskPolygon(merged_msg,
                                perception_obstacle_debug_msg.get());
      FillFreespaceIntoPerceptionObstacles(*perception_obstacle_debug_msg,
                                           lidar2world_pose,
                                           perception_obstacles.get());
    }

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

    if (nullptr != merged_msg->lidar_frame_->hdmap_struct) {
      const auto& road_polygons =
          merged_msg->lidar_frame_->hdmap_struct->road_polygons;
      for (const auto& road_polygon : road_polygons) {
        auto* output_polygon =
            perception_obstacle_debug_msg->add_road_polygons();
        for (const auto& point : road_polygon) {
          auto* output_point = output_polygon->add_point();
          output_point->set_x(point.x - translation.x());
          output_point->set_y(point.y - translation.y());
          output_point->set_z(0.0);
        }
      }
      const auto& electric_fence_polygons =
          merged_msg->lidar_frame_->hdmap_struct->electric_fence_polygons;
      for (const auto& fence_polygon : electric_fence_polygons) {
        auto* output_polygon =
            perception_obstacle_debug_msg->add_electric_fence_polygons();
        for (const auto& point : fence_polygon) {
          auto* output_point = output_polygon->add_point();
          output_point->set_x(point.x - translation.x());
          output_point->set_y(point.y - translation.y());
          output_point->set_z(0.0);
        }
      }
    }

    perception_obstacles->mutable_lidar2world()->CopyFrom(lidar2world_pose_proto);
    perception_obstacle_debug_msg->mutable_obstacles()->CopyFrom(
        *perception_obstacles);
    if (enable_freespace_mask_) {
      lidar::CopyFreespaceToObstaclesLocal(
          *perception_obstacle_debug_msg,
          perception_obstacle_debug_msg->mutable_obstacles());
    }
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

  Eigen::Vector3f center;
  Eigen::Vector3f size;
  Eigen::Affine3f tf;
  Eigen::Affine3f tf_inv;
  Eigen::Vector3f raw_pt;
  Eigen::Vector3f pt_local;
  for (size_t i = 0; i < non_ground_pc->size(); ++i) {
    const auto& pt = (*non_ground_pc)[i];
    bool is_point_inside_object = false;

    for (const auto& obj : objects) {
      if (obj->type != base::ObjectType::VEHICLE &&
          obj->type != base::ObjectType::STACKER) {
        continue;
      }

      center = Eigen::Vector3f(obj->center(0), obj->center(1), obj->center(2));
      size = obj->size;
      float yaw = obj->theta;

      tf = Eigen::Affine3f::Identity();
      tf.translate(center);
      tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
      tf_inv = tf.inverse();

      raw_pt = Eigen::Vector3f(pt.x, pt.y, pt.z);

      pt_local = tf_inv * raw_pt;

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
      remove_pt_indices.emplace_back(i);
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

  Eigen::Vector3f center;
  Eigen::Vector3f size;
  Eigen::Affine3f tf;
  Eigen::Affine3f tf_inv;
  Eigen::Vector3f raw_pt;
  Eigen::Vector3f pt_local;
  for (size_t obj_idx = 0; obj_idx < objects.size(); ++obj_idx) {
    const auto& obj = objects[obj_idx];

    if (obj->sub_type != base::ObjectSubType::TRAILER_FULL) {
      continue;
    }

    center = Eigen::Vector3f(obj->center(0), obj->center(1), obj->center(2));
    size = obj->size;
    float yaw = obj->theta;

    tf = Eigen::Affine3f::Identity();
    tf.translate(center);
    tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    tf_inv = tf.inverse();

    float half_l = size(0) * 0.5f + kRangeThreshold;
    float half_w = size(1) * 0.5f + kRangeThreshold;
    float half_h = size(2) * 0.5f + kRangeThreshold;

    float min_z = std::numeric_limits<float>::max();
    bool has_points_inside = false;

    for (size_t i = 0; i < non_ground_pc->size(); ++i) {
      const auto& pt = (*non_ground_pc)[i];
      raw_pt = Eigen::Vector3f(pt.x, pt.y, pt.z);
      pt_local = tf_inv * raw_pt;

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
