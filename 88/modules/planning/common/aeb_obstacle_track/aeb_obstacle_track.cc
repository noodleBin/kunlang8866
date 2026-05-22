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
 * @file aeb_obstacle_track.cpp
 **/

#include "aeb_obstacle_track.h"

namespace century {
namespace planning {

int ObstacleTracker::GenerateTrackId() {
  return (next_track_id_ >= INT_MAX) ? 0 : ++next_track_id_;
}

bool ObstacleTracker::HasTrack(int track_id) const {
  return tracks_.find(track_id) != tracks_.end();
}

std::vector<TrackResult> ObstacleTracker::Track(
    const std::vector<perception::PerceptionObstacle>& obstacles,
    double timestamp) {
  std::vector<TrackResult> results;
  TrackResult track_result;

  for (const auto& obs : obstacles) {
    int track_id = MatchTrack(obs);

    if (track_id < 0) {
      track_id = GenerateTrackId();
    }

    TrackedObstacle& track_obstacle = tracks_[track_id];
    track_obstacle.track_id = track_id;
    track_obstacle.center =
        common::math::Vec2d(obs.position().x(), obs.position().y());
    track_obstacle.last_seen_time = timestamp;
    track_obstacle.type = obs.type();

    track_result.track_id = track_id;
    track_result.obstacle = &obs;
    results.emplace_back(track_result);
  }
  RemoveExpiredTracks(timestamp);
  return results;
}

int ObstacleTracker::MatchTrack(
    const perception::PerceptionObstacle& obstacle) {
  const common::math::Vec2d obs_center(obstacle.position().x(),
                                       obstacle.position().y());
  int best_track_id = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& val : tracks_) {
    const auto& track_obstacle = val.second;

    if (track_obstacle.type != obstacle.type()) {
      continue;
    }

    double dist = obs_center.DistanceTo(track_obstacle.center);
    if (dist < min_dist && dist < config_.match_distance_threshold) {
      min_dist = dist;
      best_track_id = track_obstacle.track_id;
    }
  }
  return best_track_id;
}

void ObstacleTracker::RemoveExpiredTracks(double timestamp) {
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (timestamp - it->second.last_seen_time > config_.track_timeout) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace planning
}  // namespace century