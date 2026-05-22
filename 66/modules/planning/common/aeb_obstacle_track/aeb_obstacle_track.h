/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
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
 * @file aeb_obstacle_track.h
 **/

#pragma once

#include <unordered_map>
#include <vector>
#include <limits>
#include <memory>
#include <string>
#include <climits>

#include "modules/common/math/vec2d.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/common/util/normal_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

namespace century {
namespace planning {

struct TrackedObstacle {
  int track_id;
  common::math::Vec2d center;
  double last_seen_time;
  perception::PerceptionObstacle::Type type;
};

struct TrackResult {
  int track_id;
  const perception::PerceptionObstacle* obstacle;
};

class ObstacleTracker {
 public:
  struct Config {
    double match_distance_threshold = 1.5;  // meters
    double track_timeout = 0.5;              // seconds
  };

  explicit ObstacleTracker(const Config& config)
      : config_(config) {}

  std::vector<TrackResult> Track(
      const std::vector<perception::PerceptionObstacle>& obstacles,
      double timestamp);

  int MatchTrack(const perception::PerceptionObstacle& obstacle);

  bool HasTrack(int track_id) const;

 private:
  int GenerateTrackId();

  void RemoveExpiredTracks(double timestamp);

 private:
  Config config_;
  int next_track_id_ = 0;

  std::unordered_map<int, TrackedObstacle> tracks_;
};

}  // namespace planning
}  // namespace century