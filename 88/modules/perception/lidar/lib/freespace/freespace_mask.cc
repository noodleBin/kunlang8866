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

#include "modules/perception/lidar/lib/freespace/freespace_mask.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/common/i_lib/core/i_constant.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/freespace/freespace_mask_cuda.h"

namespace century {
namespace perception {
namespace lidar {

namespace {
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
  std::vector<double> cell_center_x;
  std::vector<double> cell_center_y;

  bool IsInside(const double x, const double y) const {
    return x >= spec.x_min && x < spec.x_max && y >= spec.y_min &&
           y < spec.y_max;
  }

  void ToGrid(const double x, const double y, int* gx, int* gy) const {
    *gx = static_cast<int>(std::floor((x - spec.x_min) / spec.resolution));
    *gy = static_cast<int>(std::floor((y - spec.y_min) / spec.resolution));
  }

  void CellCenter(const int gx, const int gy, double* x, double* y) const {
    *x = cell_center_x[static_cast<size_t>(gx)];
    *y = cell_center_y[static_cast<size_t>(gy)];
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

struct FreespaceRayLookup {
  std::vector<double> cos_theta;
  std::vector<double> sin_theta;
};

inline bool IsFiniteValue(const double v) { return std::isfinite(v); }

inline double GetGroundZ(const LidarFrame& frame) {
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

const FreespaceRayLookup& GetFreespaceRayLookup(const int ray_count) {
  static std::vector<FreespaceRayLookup> cache;
  if (ray_count <= 0) {
    static const FreespaceRayLookup empty_lookup;
    return empty_lookup;
  }
  if (cache.size() <= static_cast<size_t>(ray_count)) {
    cache.resize(static_cast<size_t>(ray_count) + 1);
  }
  FreespaceRayLookup& lookup = cache[static_cast<size_t>(ray_count)];
  if (lookup.cos_theta.size() == static_cast<size_t>(ray_count)) {
    return lookup;
  }

  lookup.cos_theta.resize(static_cast<size_t>(ray_count));
  lookup.sin_theta.resize(static_cast<size_t>(ray_count));
  const double angular_step =
      2.0 * common::Constant<double>::PI() / static_cast<double>(ray_count);
  for (int ray_id = 0; ray_id < ray_count; ++ray_id) {
    const double theta =
        -common::Constant<double>::PI() +
        static_cast<double>(ray_id) * angular_step;
    lookup.cos_theta[static_cast<size_t>(ray_id)] = std::cos(theta);
    lookup.sin_theta[static_cast<size_t>(ray_id)] = std::sin(theta);
  }
  return lookup;
}

double CellRangeFromSource(const FreespaceGridContext& context,
                           const double source_x, const int gx,
                           const int gy) {
  const double dx = context.cell_center_x[static_cast<size_t>(gx)] - source_x;
  const double dy = context.cell_center_y[static_cast<size_t>(gy)];
  return std::hypot(dx, dy);
}

int AdaptiveMinPointsPerCell(const FreespaceGridContext& context,
                             const double source_x,
                             const int gx, const int gy,
                             const int base_min_points_per_cell) {
  const double range = CellRangeFromSource(context, source_x, gx, gy);
  if (range >= 45.0) {
    return 1;
  }
  if (range >= 30.0) {
    return std::max(1, base_min_points_per_cell - 1);
  }
  return base_min_points_per_cell;
}

bool PrepareFreespaceGridContext(const LidarFrame& frame,
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
  context->cell_center_x.resize(static_cast<size_t>(context->width));
  context->cell_center_y.resize(static_cast<size_t>(context->height));
  for (int gx = 0; gx < context->width; ++gx) {
    context->cell_center_x[static_cast<size_t>(gx)] =
        spec.x_min + (static_cast<double>(gx) + 0.5) * spec.resolution;
  }
  for (int gy = 0; gy < context->height; ++gy) {
    context->cell_center_y[static_cast<size_t>(gy)] =
        spec.y_min + (static_cast<double>(gy) + 0.5) * spec.resolution;
  }
  return true;
}

void MarkPointOccupancy(const LidarFrame& frame,
                        const int min_points_per_cell,
                        const double source_x,
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
    gx = std::max(0, std::min(context->width - 1, gx));
    gy = std::max(0, std::min(context->height - 1, gy));
    const size_t idx =
        static_cast<size_t>(ToGridIndex(gx, gy, context->width));
    if (context->occupied_point_count[idx] <
        std::numeric_limits<uint16_t>::max()) {
      ++context->occupied_point_count[idx];
    }
  }

  for (size_t idx = 0; idx < context->occupied_point_count.size(); ++idx) {
    if (0 == context->occupied_point_count[idx]) {
      continue;
    }
    const int gx = static_cast<int>(idx % static_cast<size_t>(context->width));
    const int gy = static_cast<int>(idx / static_cast<size_t>(context->width));
    const int adaptive_min_points = AdaptiveMinPointsPerCell(
        *context, source_x, gx, gy, min_points_per_cell);
    if (context->occupied_point_count[idx] >= adaptive_min_points) {
      context->occupied[idx] = 1;
    }
  }
}

void MarkObjectOccupancy(const LidarFrame& frame, const double inflate,
                         const int min_support_points,
                         FreespaceGridContext* context) {
  if (nullptr == context) {
    return;
  }
  const int support_threshold = std::max(1, min_support_points);
  std::vector<int> supported_cells;
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

    // First pass: check support by counting non-ground points that already
    // fell into cells within the object footprint during MarkPointOccupancy.
    int support_count = 0;
    supported_cells.clear();
    for (int gy = gy_min; gy <= gy_max && support_count < support_threshold;
         ++gy) {
      for (int gx = gx_min; gx <= gx_max; ++gx) {
        const size_t idx =
            static_cast<size_t>(ToGridIndex(gx, gy, context->width));
        const uint16_t cell_points = context->occupied_point_count[idx];
        if (0 == cell_points) {
          continue;
        }
        double x = 0.0;
        double y = 0.0;
        context->CellCenter(gx, gy, &x, &y);
        const double dx = x - cx;
        const double dy = y - cy;
        const double local_x = cos_yaw * dx + sin_yaw * dy;
        const double local_y = -sin_yaw * dx + cos_yaw * dy;
        if (std::fabs(local_x) <= half_l && std::fabs(local_y) <= half_w) {
          supported_cells.emplace_back(static_cast<int>(idx));
          support_count += cell_points;
          if (support_count >= support_threshold) {
            break;
          }
        }
      }
    }
    if (support_count < support_threshold) {
      continue;
    }

    for (const int supported_idx : supported_cells) {
      context->occupied[static_cast<size_t>(supported_idx)] = 1;
    }
    for (int gy = gy_min; gy <= gy_max; ++gy) {
      for (int gx = gx_min; gx <= gx_max; ++gx) {
        const int idx = ToGridIndex(gx, gy, context->width);
        if (0 != context->occupied[static_cast<size_t>(idx)]) {
          continue;
        }
        double x = 0.0;
        double y = 0.0;
        context->CellCenter(gx, gy, &x, &y);
        const double dx = x - cx;
        const double dy = y - cy;
        const double local_x = cos_yaw * dx + sin_yaw * dy;
        const double local_y = -sin_yaw * dx + cos_yaw * dy;
        if (std::fabs(local_x) <= half_l && std::fabs(local_y) <= half_w) {
          context->occupied[static_cast<size_t>(idx)] = 1;
        }
      }
    }
  }
}

void InflateOccupiedGrid(const double inflate_radius,
                         FreespaceGridContext* context) {
  if (nullptr == context || inflate_radius <= 1e-6) {
    return;
  }
  const int radius_cells = std::max(
      1, static_cast<int>(std::ceil(inflate_radius / context->spec.resolution)));
  std::vector<uint8_t> inflated = context->occupied;
  for (int gy = 0; gy < context->height; ++gy) {
    for (int gx = 0; gx < context->width; ++gx) {
      const size_t idx =
          static_cast<size_t>(ToGridIndex(gx, gy, context->width));
      if (0 == context->occupied[idx]) {
        continue;
      }
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          if (dx * dx + dy * dy > radius_cells * radius_cells) {
            continue;
          }
          const int nx = gx + dx;
          const int ny = gy + dy;
          if (nx < 0 || nx >= context->width || ny < 0 ||
              ny >= context->height) {
            continue;
          }
          double x = 0.0;
          double y = 0.0;
          context->CellCenter(nx, ny, &x, &y);
          if (context->IsInsideVehicle(x, y)) {
            continue;
          }
          inflated[static_cast<size_t>(
              ToGridIndex(nx, ny, context->width))] = 1;
        }
      }
    }
  }
  context->occupied.swap(inflated);
}

struct RailwayPolygonLocal {
  base::PointCloud<base::PointF> points;
  int gx_min = 0;
  int gx_max = 0;
  int gy_min = 0;
  int gy_max = 0;
};

void ApplyRailwayMask(const LidarFrame& frame,
                      FreespaceGridContext* context) {
  if (nullptr == context || nullptr == frame.hdmap_struct) {
    return;
  }

  const auto world2lidar = frame.lidar2world_pose.inverse();
  std::vector<RailwayPolygonLocal> local_railway_polygons;
  local_railway_polygons.reserve(
      frame.hdmap_struct->railway_boundary_polygons.size());
  for (const auto& railway_polygon :
       frame.hdmap_struct->railway_boundary_polygons) {
    if (railway_polygon.size() < 3) {
      continue;
    }
    RailwayPolygonLocal local;
    local.points.reserve(railway_polygon.size());
    double bbox_x_min = std::numeric_limits<double>::infinity();
    double bbox_x_max = -std::numeric_limits<double>::infinity();
    double bbox_y_min = std::numeric_limits<double>::infinity();
    double bbox_y_max = -std::numeric_limits<double>::infinity();
    for (const auto& point : railway_polygon) {
      const Eigen::Vector3d local_pos =
          world2lidar * Eigen::Vector3d(point.x, point.y, point.z);
      base::PointF local_pt;
      local_pt.x = static_cast<float>(local_pos(0));
      local_pt.y = static_cast<float>(local_pos(1));
      local_pt.z = static_cast<float>(local_pos(2));
      local.points.push_back(local_pt);
      bbox_x_min = std::min(bbox_x_min, local_pos(0));
      bbox_x_max = std::max(bbox_x_max, local_pos(0));
      bbox_y_min = std::min(bbox_y_min, local_pos(1));
      bbox_y_max = std::max(bbox_y_max, local_pos(1));
    }
    if (local.points.size() < 3) {
      continue;
    }
    if (bbox_x_max < context->spec.x_min ||
        bbox_x_min >= context->spec.x_max ||
        bbox_y_max < context->spec.y_min ||
        bbox_y_min >= context->spec.y_max) {
      continue;
    }
    context->ToGrid(bbox_x_min, bbox_y_min, &local.gx_min, &local.gy_min);
    context->ToGrid(bbox_x_max, bbox_y_max, &local.gx_max, &local.gy_max);
    local.gx_min = std::max(0, local.gx_min);
    local.gy_min = std::max(0, local.gy_min);
    local.gx_max = std::min(context->width - 1, local.gx_max);
    local.gy_max = std::min(context->height - 1, local.gy_max);
    local_railway_polygons.emplace_back(std::move(local));
  }
  if (local_railway_polygons.empty()) {
    return;
  }

  for (const auto& polygon : local_railway_polygons) {
    for (int gy = polygon.gy_min; gy <= polygon.gy_max; ++gy) {
      for (int gx = polygon.gx_min; gx <= polygon.gx_max; ++gx) {
        const size_t idx =
            static_cast<size_t>(ToGridIndex(gx, gy, context->width));
        if (0 == context->map_allowed[idx]) {
          continue;
        }
        double cx = 0.0;
        double cy = 0.0;
        context->CellCenter(gx, gy, &cx, &cy);
        base::PointF query_pt;
        query_pt.x = static_cast<float>(cx);
        query_pt.y = static_cast<float>(cy);
        query_pt.z = 0.0f;
        if (common::IsPointXYInPolygon2DXY(query_pt, polygon.points)) {
          context->map_allowed[idx] = 0;
        }
      }
    }
  }
}

double ComputeFreespaceSourceX(const LidarFrame& frame) {
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
  const auto& ray_lookup = GetFreespaceRayLookup(ray_state->ray_count);
  constexpr double kRayEntryEpsilonScale = 1e-3;
  for (int ray_id = 0; ray_id < ray_state->ray_count; ++ray_id) {
    const double cos_theta = ray_lookup.cos_theta[static_cast<size_t>(ray_id)];
    const double sin_theta = ray_lookup.sin_theta[static_cast<size_t>(ray_id)];
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

bool TryCastFreespaceRaysCuda(const FreespaceGridContext& context,
                              FreespaceRayState* ray_state) {
  if (nullptr == ray_state || context.occupied.empty() ||
      context.map_allowed.empty()) {
    return false;
  }

  FreespaceCudaRayCastInput input;
  input.occupied = context.occupied.data();
  input.map_allowed = context.map_allowed.data();
  input.width = context.width;
  input.height = context.height;
  input.x_min = context.spec.x_min;
  input.x_max = context.spec.x_max;
  input.y_min = context.spec.y_min;
  input.y_max = context.spec.y_max;
  input.resolution = context.spec.resolution;
  input.self_x_min = context.self_x_min;
  input.self_x_max = context.self_x_max;
  input.self_y_min = context.self_y_min;
  input.self_y_max = context.self_y_max;
  input.source_x = ray_state->source_x;
  input.source_y = ray_state->source_y;
  input.max_range = ray_state->max_range;
  input.ray_count = ray_state->ray_count;

  return CastFreespaceRaysCuda(input, ray_state->ray_ranges.data(),
                               ray_state->ray_valid.data(),
                               ray_state->ray_hit_occupied.data(),
                               ray_state->ray_hit_occ_x.data(),
                               ray_state->ray_hit_occ_y.data());
}

void FilterRayOutliers(FreespaceRayState* ray_state) {
  if (nullptr == ray_state) {
    return;
  }
  // Disabled to inspect raw freespace ray shape without local outlier repair.
  return;
}

void CloseNarrowRayGaps(FreespaceRayState* ray_state) {
  if (nullptr == ray_state) {
    return;
  }
  // Disabled for now to avoid over-closing narrow openings.
  return;
}

void CloseNarrowRangeProtrusions(FreespaceRayState* ray_state) {
  if (nullptr == ray_state) {
    return;
  }
  // Disabled to inspect whether inward dents come from protrusion closing.
  return;
}

void SmoothFreespaceBoundary(FreespaceRayState* ray_state) {
  if (nullptr == ray_state) {
    return;
  }
  // Disabled to preserve raw ray boundary.
  return;
}

void ApplyTemporalFreespaceFilter(const FreespaceBuildOptions& options,
                                  FreespaceRayState* ray_state,
                                  FreespaceTemporalFilterState* state) {
  if (nullptr == ray_state || nullptr == state ||
      !options.enable_temporal_filter) {
    return;
  }
  constexpr uint8_t kNoPendingChange = 0;
  constexpr uint8_t kExpandPendingChange = 1;
  constexpr uint8_t kShrinkPendingChange = 2;
  constexpr uint8_t kTemporalConfirmFrames = 3;
  constexpr double kRangeEpsilon = 1e-3;
  const bool state_compatible =
      state->ray_ranges.size() == ray_state->ray_ranges.size() &&
      state->ray_valid.size() == ray_state->ray_valid.size() &&
      state->pending_ray_ranges.size() == ray_state->ray_ranges.size() &&
      state->pending_ray_valid.size() == ray_state->ray_valid.size() &&
      state->pending_change_type.size() == ray_state->ray_ranges.size() &&
      state->pending_change_count.size() == ray_state->ray_ranges.size() &&
      state->spec.resolution == options.spec.resolution &&
      state->spec.x_min == options.spec.x_min &&
      state->spec.x_max == options.spec.x_max &&
      state->spec.y_min == options.spec.y_min &&
      state->spec.y_max == options.spec.y_max &&
      std::fabs(state->source_x - ray_state->source_x) <=
          options.temporal_source_shift_reset &&
      std::fabs(state->source_y - ray_state->source_y) <=
          options.temporal_source_shift_reset;
  if (!state_compatible) {
    state->spec = options.spec;
    state->source_x = ray_state->source_x;
    state->source_y = ray_state->source_y;
    state->ray_ranges = ray_state->ray_ranges;
    state->ray_valid = ray_state->ray_valid;
    state->pending_ray_ranges = ray_state->ray_ranges;
    state->pending_ray_valid = ray_state->ray_valid;
    state->pending_change_type.assign(ray_state->ray_ranges.size(),
                                      kNoPendingChange);
    state->pending_change_count.assign(ray_state->ray_ranges.size(), 0);
    return;
  }

  for (size_t i = 0; i < ray_state->ray_ranges.size(); ++i) {
    const bool cur_valid = 0 != ray_state->ray_valid[i] &&
                           ray_state->ray_ranges[i] > 1e-3;
    const bool prev_valid =
        0 != state->ray_valid[i] && state->ray_ranges[i] > 1e-3;
    const uint8_t observed_change_type =
        (!prev_valid && cur_valid)
            ? kExpandPendingChange
            : (prev_valid && !cur_valid)
                  ? kShrinkPendingChange
                  : (!prev_valid && !cur_valid)
                        ? kNoPendingChange
                        : (ray_state->ray_ranges[i] > state->ray_ranges[i] +
                                   kRangeEpsilon)
                              ? kExpandPendingChange
                              : (ray_state->ray_ranges[i] + kRangeEpsilon <
                                         state->ray_ranges[i])
                                    ? kShrinkPendingChange
                                    : kNoPendingChange;

    if (kNoPendingChange == observed_change_type) {
      state->pending_change_type[i] = kNoPendingChange;
      state->pending_change_count[i] = 0;
      state->pending_ray_ranges[i] = state->ray_ranges[i];
      state->pending_ray_valid[i] = state->ray_valid[i];
      ray_state->ray_ranges[i] = state->ray_ranges[i];
      ray_state->ray_valid[i] = state->ray_valid[i];
      continue;
    }

    const bool same_pending =
        state->pending_change_type[i] == observed_change_type &&
        ((cur_valid && state->pending_ray_valid[i] &&
          std::fabs(state->pending_ray_ranges[i] - ray_state->ray_ranges[i]) <=
              kRangeEpsilon) ||
         (!cur_valid && 0 == state->pending_ray_valid[i]));
    if (!same_pending) {
      state->pending_change_type[i] = observed_change_type;
      state->pending_change_count[i] = 1;
      state->pending_ray_ranges[i] = ray_state->ray_ranges[i];
      state->pending_ray_valid[i] = ray_state->ray_valid[i];
    } else {
      state->pending_change_count[i] = static_cast<uint8_t>(
          std::min<int>(kTemporalConfirmFrames,
                        static_cast<int>(state->pending_change_count[i]) + 1));
    }

    if (state->pending_change_count[i] >= kTemporalConfirmFrames) {
      state->ray_ranges[i] = state->pending_ray_ranges[i];
      state->ray_valid[i] = state->pending_ray_valid[i];
      state->pending_change_type[i] = kNoPendingChange;
      state->pending_change_count[i] = 0;
    }
    ray_state->ray_ranges[i] = state->ray_ranges[i];
    ray_state->ray_valid[i] = state->ray_valid[i];
  }

  state->spec = options.spec;
  state->source_x = ray_state->source_x;
  state->source_y = ray_state->source_y;
  state->ray_ranges = ray_state->ray_ranges;
  state->ray_valid = ray_state->ray_valid;
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

void SmoothPolyline(century::common::Polyline* polyline) {
  if (nullptr == polyline || polyline->point_size() < 5) {
    return;
  }
  // Disabled to preserve raw boundary line shape for debugging.
  return;
}

bool WriteFreespaceDebugOutput(const LidarFrame& frame,
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
  } else {
    SmoothPolyline(debug_msg->mutable_freespace_right_boundary());
  }
  if (debug_msg->freespace_left_boundary().point_size() < 2) {
    debug_msg->clear_freespace_left_boundary();
  } else {
    SmoothPolyline(debug_msg->mutable_freespace_left_boundary());
  }
  return true;
}

}  // namespace

bool BuildFreespaceMask(const LidarFrame& frame,
                        const FreespaceBuildOptions& options,
                        PerceptionObstacleDebugMsg* debug_msg,
                        FreespaceTemporalFilterState* temporal_state) {
  if (nullptr != debug_msg) {
    debug_msg->clear_freespace_mask_polygon();
    debug_msg->clear_freespace_left_boundary();
    debug_msg->clear_freespace_right_boundary();
    debug_msg->clear_freespace_ray_source();
  }
  if (nullptr == debug_msg || nullptr == frame.raw_cloud ||
      frame.raw_cloud->empty()) {
    return false;
  }

  FreespaceGridContext context;
  if (!PrepareFreespaceGridContext(frame, options, &context)) {
    return false;
  }
  const double source_x = ComputeFreespaceSourceX(frame);
  MarkPointOccupancy(frame, std::max(1, options.min_points_per_cell), source_x,
                     &context);
  MarkObjectOccupancy(frame, std::max(0.0, options.obstacle_inflate),
                      std::max(1, options.min_points_per_cell), &context);
  InflateOccupiedGrid(std::max(0.0, options.obstacle_inflate), &context);
  if (options.use_hdmap_road) {
    ApplyRailwayMask(frame, &context);
  }

  FreespaceRayState ray_state =
      CreateFreespaceRayState(context, source_x);
  WriteRaySourceDebug(options.show_ray_source, context, ray_state, debug_msg);
  if (!options.use_cuda || !TryCastFreespaceRaysCuda(context, &ray_state)) {
    CastFreespaceRays(context, &ray_state);
  }

  FilterRayOutliers(&ray_state);
  CloseNarrowRayGaps(&ray_state);
  CloseNarrowRangeProtrusions(&ray_state);
  SmoothFreespaceBoundary(&ray_state);
  ApplyTemporalFreespaceFilter(options, &ray_state, temporal_state);

  return WriteFreespaceDebugOutput(frame, context, ray_state, debug_msg);
}

namespace {

void TransformPointToWorld(const Eigen::Affine3d& lidar2world_pose,
                           const century::common::Point3D& point_lidar,
                           century::common::Point3D* point_world) {
  if (nullptr == point_world) {
    return;
  }
  const Eigen::Vector3d pt(point_lidar.x(), point_lidar.y(), point_lidar.z());
  const Eigen::Vector3d transformed = lidar2world_pose * pt;
  point_world->set_x(transformed.x());
  point_world->set_y(transformed.y());
  point_world->set_z(transformed.z());
}

}  // namespace

void CopyFreespaceToObstaclesLocal(
    const PerceptionObstacleDebugMsg& debug_msg,
    PerceptionObstacles* perception_obstacles) {
  if (nullptr == perception_obstacles) {
    return;
  }
  perception_obstacles->clear_freespace_mask_polygon();
  perception_obstacles->clear_freespace_left_boundary();
  perception_obstacles->clear_freespace_right_boundary();
  for (const auto& polygon : debug_msg.freespace_mask_polygon()) {
    perception_obstacles->add_freespace_mask_polygon()->CopyFrom(polygon);
  }
  if (debug_msg.has_freespace_left_boundary()) {
    perception_obstacles->mutable_freespace_left_boundary()->CopyFrom(
        debug_msg.freespace_left_boundary());
  }
  if (debug_msg.has_freespace_right_boundary()) {
    perception_obstacles->mutable_freespace_right_boundary()->CopyFrom(
        debug_msg.freespace_right_boundary());
  }
}

void CopyFreespaceToObstaclesWorld(
    const PerceptionObstacleDebugMsg& debug_msg,
    const Eigen::Affine3d& lidar2world_pose,
    PerceptionObstacles* perception_obstacles) {
  if (nullptr == perception_obstacles) {
    return;
  }
  perception_obstacles->clear_freespace_mask_polygon();
  perception_obstacles->clear_freespace_left_boundary();
  perception_obstacles->clear_freespace_right_boundary();
  for (const auto& polygon_lidar : debug_msg.freespace_mask_polygon()) {
    auto* polygon_world = perception_obstacles->add_freespace_mask_polygon();
    for (const auto& point_lidar : polygon_lidar.point()) {
      TransformPointToWorld(lidar2world_pose, point_lidar,
                            polygon_world->add_point());
    }
  }
  if (debug_msg.has_freespace_left_boundary()) {
    auto* left = perception_obstacles->mutable_freespace_left_boundary();
    for (const auto& point_lidar : debug_msg.freespace_left_boundary().point()) {
      TransformPointToWorld(lidar2world_pose, point_lidar, left->add_point());
    }
  }
  if (debug_msg.has_freespace_right_boundary()) {
    auto* right = perception_obstacles->mutable_freespace_right_boundary();
    for (const auto& point_lidar :
         debug_msg.freespace_right_boundary().point()) {
      TransformPointToWorld(lidar2world_pose, point_lidar, right->add_point());
    }
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace century
