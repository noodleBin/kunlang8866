/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

/**
 * @file gridded_path_time_graph.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

namespace century {
namespace planning {

using century::common::ErrorCode;
using century::common::SpeedPoint;
using century::common::Status;
using century::common::util::PointFactory;

namespace {

static constexpr double kDoubleEpsilon = 1.0e-6;
static constexpr double kcoeff = 1.1;  // v coeff
static constexpr double kStopBoundaryBuffer = 1.0e-2;
static constexpr double kBounadryEpsilon = 1.0e-2;
static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
constexpr double kMinDecel = -6.0;

}  // namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(
    const StGraphData& st_graph_data, const DpStSpeedOptimizerConfig& dp_config,
    const PathDecision& decision, const common::TrajectoryPoint& init_point,
    const std::shared_ptr<DependencyInjector>& injector)
    : st_graph_data_(st_graph_data),
      gridded_path_time_graph_config_(dp_config),
      decision_(decision),
      init_point_(init_point),
      injector_(injector),
      dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),
                  st_graph_data_.path_length(), decision.obstacles().Items(),
                  st_graph_data_.st_drivable_boundary(), init_point_) {
  total_length_t_ = st_graph_data_.total_time_by_conf();
  unit_t_ = gridded_path_time_graph_config_.unit_t();
  CHECK_GT(unit_t_, 0.0);
  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ =
      std::min(std::abs(vehicle_param_.max_acceleration()),
               std::abs(gridded_path_time_graph_config_.max_acceleration()));
  max_deceleration_ =
      -1.0 *
      std::min(std::abs(vehicle_param_.max_deceleration()),
               std::abs(gridded_path_time_graph_config_.max_deceleration()));
  //  // obtain the longest achievable distance
  double delt_t =
      std::fabs((init_point_.v() - st_graph_data_.cruise_speed() * kcoeff) /
                max_acceleration_);
  double max_s =
      init_point_.v() * delt_t + 0.5 * max_acceleration_ * delt_t * delt_t +
      (total_length_t_ - delt_t) * st_graph_data_.cruise_speed() * kcoeff;
  total_length_s_ = std::min(st_graph_data_.path_length(), max_s);

  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();
  CHECK_NE(dense_unit_s_, 0.0);
  CHECK_NE(sparse_unit_s_, 0.0);
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();
}

void GriddedPathTimeGraph::GenerateFallbackProfile(
    SpeedData* const speed_data) {
  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;
  std::vector<SpeedPoint> speed_profile;
  double t = 0.0;
  for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {
    speed_profile.emplace_back(PointFactory::ToSpeedPoint(0, t));
  }
  *speed_data = SpeedData(speed_profile);
}

// Continuous-time collision check using linear interpolation as closed-loop
// dynamics
bool GriddedPathTimeGraph::CheckOverlapOnDpStGraph(
    const std::vector<const STBoundary*>& boundaries, const StGraphPoint& p1,
    const StGraphPoint& p2) {
  if (FLAGS_use_st_drivable_boundary) {
    return false;
  }
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // Check collision between a polygon and a line segment
    if (boundary->HasOverlap({p1.point(), p2.point()})) {
      const auto* obs = decision_.Find(boundary->id());
      if (obs != nullptr) {
        if (obs->IsSlowBreakingObstacle() && is_need_slow_breaking_) {
          // AINFO << "collision";
          continue;
        }
      }
      return true;
    }
  }
  return false;
}

Status GriddedPathTimeGraph::Search(const bool is_need_slow_breaking,
                                    SpeedData* const speed_data) {
  is_need_slow_breaking_ = is_need_slow_breaking;
  double min_stop_boundary_s = std::numeric_limits<double>::max();
  double min_boundary_s = std::numeric_limits<double>::max();
  std::string min_stop_boundary_id("");
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    // KeepClear obstacles not considered in Dp St decision
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // If init point in collision with obstacle, return speed fallback
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      GenerateFallbackProfile(speed_data);
      return Status::OK();
    }
    if (STBoundary::BoundaryType::STOP == boundary->boundary_type() &&
        boundary->min_s() < min_stop_boundary_s) {
      min_stop_boundary_s = boundary->min_s();
      min_stop_boundary_id = boundary->id();
    }
    if (boundary->min_s() < min_boundary_s) {
      min_boundary_s = boundary->min_s();
    }
  }

  const Obstacle* block_obstacle = nullptr;
  if (!min_stop_boundary_id.empty()) {
    block_obstacle = decision_.Find(min_stop_boundary_id);
    if (nullptr == block_obstacle) {
      AERROR << "Null obstacle pointer of minimal stop decision.";
    }
    if (!CheckBlockObstacle(block_obstacle)) {
      AERROR << "STOP decision obstacle hasn't stop boundary feature.";
    } else {
      // set close block obstacle.
      close_block_obstacle_ = block_obstacle;
    }
  }

  Status ret = InitCostTable();
  if (!ret.ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }

  ret = InitSpeedLimitLookUp();
  if (!ret.ok()) {
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }

  ret = CalculateTotalCost();
  if (!ret.ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }

  ret = RetrieveSpeedProfile(speed_data);
  if (!ret.ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  return Status::OK();
}

bool GriddedPathTimeGraph::CheckBlockObstacle(const Obstacle* block_obstacle) {
  if (!block_obstacle->is_path_st_boundary_initialized()) {
    return false;
  }
  ADEBUG << "bottom_left_point:("
         << block_obstacle->path_st_boundary().bottom_left_point().s() << ","
         << block_obstacle->path_st_boundary().bottom_left_point().t() << ")";
  ADEBUG << "bottom_right_point:("
         << block_obstacle->path_st_boundary().bottom_right_point().s() << ","
         << block_obstacle->path_st_boundary().bottom_right_point().t() << ")";
  if (std::abs(block_obstacle->path_st_boundary().bottom_left_point().s() -
               block_obstacle->path_st_boundary().bottom_right_point().s()) >
          kStopBoundaryBuffer ||
      block_obstacle->path_st_boundary().bottom_left_point().t() > 0.0 ||
      block_obstacle->path_st_boundary().bottom_right_point().t() <
          total_length_t_) {
    return false;
  }
  return true;
}

Status GriddedPathTimeGraph::InitCostTable() {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;

  double sparse_length_s =
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  sparse_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_;
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }

  const auto& cost_table_0 = cost_table_[0];
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {
  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);
  const auto& speed_limit = st_graph_data_.speed_limit();

  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] =
        speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  for (size_t c = 0; c < cost_table_.size(); ++c) {
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;

    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {
      std::vector<std::future<void>> results;
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {
          results.emplace_back(
              cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {
          CalculateCostAt(msg);
        }
      }
      if (FLAGS_enable_multi_thread_in_dp_st_graph) {
        for (auto& result : results) {
          result.get();
        }
      }
    }

    if (c < cost_table_.size() - 1UL) {
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        const auto& cost_cr = cost_table_[c][r];
        if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
          size_t h_r = 0UL;
          size_t l_r = 0UL;
          GetRowRange(cost_cr, &h_r, &l_r);
          size_t next_r = l_r < 1UL ? l_r : l_r - 1UL;
          size_t high_r =
              h_r >= static_cast<size_t>(dimension_s_ - 1U) ? h_r : h_r + 1UL;
          for (; next_r <= high_r; ++next_r) {
            cost_table_[c + 1UL][next_r].AddPrePointR(static_cast<uint32_t>(r));
          }
          highest_row = std::max(highest_row, h_r);
          lowest_row = std::min(lowest_row, l_r);
        }
      }
      // ST points cannot exceed the lower boundary of the nearest block
      // obstacle
      if (nullptr != close_block_obstacle_) {
        double min_block_s = close_block_obstacle_->path_st_boundary().min_s();
        double exceed_dense_s =
            min_block_s -
            static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
        size_t min_block_index =
            exceed_dense_s > std::numeric_limits<double>::epsilon()
                ? dense_dimension_s_ + static_cast<size_t>(std::ceil(
                                           exceed_dense_s / sparse_unit_s_))
                : static_cast<size_t>(std::ceil(min_block_s / dense_unit_s_)) +
                      1;
        ADEBUG << "close_block_obstacle: " << close_block_obstacle_->Id();
        ADEBUG << "orignal highest_row: " << highest_row;
        ADEBUG << "reduced highest_row: " << min_block_index;
        ADEBUG << "min_block_s: " << min_block_s;
        if (min_block_index < highest_row) {
          ADEBUG << "reduce the highest_row by the block obstacle.";
          highest_row = min_block_index;
        }
      }
      next_highest_row = highest_row;
      next_lowest_row = lowest_row;
    }
  }

  return Status::OK();
}

void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row) {
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s();
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.point().s();
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

void GriddedPathTimeGraph::CalculateCostAtColOne(
    const uint32_t r, const double min_s_consider_speed,
    const double speed_limit, const double cruise_speed,
    const StGraphPoint& cost_init, StGraphPoint* const ptr_cost_cr) {
  const double acc =
      2 * (ptr_cost_cr->point().s() / unit_t_ - init_point_.v()) / unit_t_;
  if (acc < max_deceleration_ || acc > max_acceleration_) {
    return;
  }

  if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&
      ptr_cost_cr->point().s() > min_s_consider_speed) {
    return;
  }

  if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), *ptr_cost_cr,
                              cost_init)) {
    return;
  }
  ptr_cost_cr->SetTotalCost(
      ptr_cost_cr->obstacle_cost() + ptr_cost_cr->spatial_potential_cost() +
      cost_init.total_cost() +
      CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));
  ptr_cost_cr->SetPrePoint(cost_init);
  ptr_cost_cr->SetOptimalSpeed(init_point_.v() + acc * unit_t_);
}

void GriddedPathTimeGraph::CalculateCostAtColTwo(
    const uint32_t r, const std::vector<uint32_t>& rs_pre,
    const double min_s_consider_speed, const double speed_limit,
    const double cruise_speed, const std::vector<StGraphPoint>& pre_col,
    StGraphPoint* const ptr_cost_cr) {
  double curr_speed_limit = speed_limit;
  for (const auto r_pre : rs_pre) {
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
    // data in ST point.
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const double curr_a =
        2 *
        ((ptr_cost_cr->point().s() - pre_col[r_pre].point().s()) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;
    if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
      continue;
    }

    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        ptr_cost_cr->point().s() > min_s_consider_speed) {
      continue;
    }

    // Filter out continuous-time node connection which is in collision with
    // obstacle
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), *ptr_cost_cr,
                                pre_col[r_pre])) {
      continue;
    }
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    const double cost =
        ptr_cost_cr->obstacle_cost() + ptr_cost_cr->spatial_potential_cost() +
        pre_col[r_pre].total_cost() +
        CalculateEdgeCostForThirdCol(r, r_pre, curr_speed_limit, cruise_speed);

    if (cost < ptr_cost_cr->total_cost()) {
      ptr_cost_cr->SetTotalCost(cost);
      ptr_cost_cr->SetPrePoint(pre_col[r_pre]);
      ptr_cost_cr->SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                   curr_a * unit_t_);
    }
  }
}

void GriddedPathTimeGraph::CalculateCostAtColRest(
    const uint32_t c, const std::vector<uint32_t>& rs_pre,
    const double min_s_consider_speed, const double speed_limit,
    const double cruise_speed, const std::vector<StGraphPoint>& pre_col,
    StGraphPoint* const ptr_cost_cr) {
  double curr_speed_limit = speed_limit;
  for (uint32_t r_pre : rs_pre) {
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const double curr_a =
        2 *
        ((ptr_cost_cr->point().s() - pre_col[r_pre].point().s()) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        ptr_cost_cr->point().s() > min_s_consider_speed) {
      continue;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), *ptr_cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = ptr_cost_cr->point();
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    double cost = ptr_cost_cr->obstacle_cost() +
                  ptr_cost_cr->spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    if (cost < ptr_cost_cr->total_cost()) {
      ptr_cost_cr->SetTotalCost(cost);
      ptr_cost_cr->SetPrePoint(pre_col[r_pre]);
      ptr_cost_cr->SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                   curr_a * unit_t_);
    }
  }
}

void GriddedPathTimeGraph::CalculateCostAt(
    const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto& cost_cr = cost_table_[c][r];

  cost_cr.SetObstacleCost(
      dp_st_cost_.GetObstacleCost(is_need_slow_breaking_, cost_cr));
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }

  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  const auto& cost_init = cost_table_[0][0];
  if (c == 0) {
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();
  // The mininal s to model as constant acceleration formula
  // default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  if (c == 1) {
    CalculateCostAtColOne(r, min_s_consider_speed, speed_limit, cruise_speed,
                          cost_init, &cost_cr);
    return;
  }

  const auto& pre_col = cost_table_[c - 1];
  const auto& rs_pre = cost_cr.GetPrePointsR();

  if (c == 2) {
    CalculateCostAtColTwo(r, rs_pre, min_s_consider_speed, speed_limit,
                          cruise_speed, pre_col, &cost_cr);
    return;
  }

  CalculateCostAtColRest(c, rs_pre, min_s_consider_speed, speed_limit,
                         cruise_speed, pre_col, &cost_cr);
}

Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    for (const auto& row : cost_table_) {
      for (const auto& cur_point : row) {
        if (!std::isinf(cur_point.total_cost())) {
          ADEBUG << "index(" << cur_point.index_t() << ", "
                 << cur_point.index_s() << "): STpoint["
                 << cur_point.point().s() << ", " << cur_point.point().t()
                 << "], total cost = " << cur_point.total_cost()
                 << ", OptimalSpeed = " << cur_point.GetOptimalSpeed();
          ADEBUG << "\tobstacle_cost = " << cur_point.obstacle_cost()
                 << ", spatial_potential_cost = "
                 << cur_point.spatial_potential_cost()
                 << ", reference_cost = " << cur_point.reference_cost();
        }
      }
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    ADEBUG << "DP rough v = " << v;
    speed_profile[i].set_v(v);
  }

  *speed_data = SpeedData(speed_profile);
  return Status::OK();
}

double GriddedPathTimeGraph::CalculateEdgeCost(
    const STPoint& first, const STPoint& second, const STPoint& third,
    const STPoint& forth, const double speed_limit, const double cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit,
                                  cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(
    const uint32_t curr_row, const uint32_t pre_row, const double speed_limit,
    const double cruise_speed) {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace century
