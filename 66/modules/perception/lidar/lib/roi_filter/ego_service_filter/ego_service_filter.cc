/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/roi_filter/ego_service_filter/ego_service_filter.h"

#include "modules/perception/lidar/common/lidar_point_label.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

namespace century {
namespace perception {
namespace lidar {

bool EgoServiceFilter::Init(const ROIFilterInitOptions& options) {
  options_ = options;
  return true;
}

bool EgoServiceFilter::Filter(const ROIFilterOptions& options,
                              LidarFrame* frame) {
  if (frame == nullptr || frame->cloud == nullptr) {
    AERROR << "Frame is nullptr.";
    return false;
  }

  FilterEgoTire(frame);

  return true;
}

void EgoServiceFilter::FilterEgoTire(LidarFrame* frame) noexcept {
  auto& filter_config = options_.ego_service_filter_configs;
  constexpr float kEgoTireWidth = 0.5f;

  if (0 == filter_config.size()) {
    return;
  }
  
  for (const auto& config : filter_config) {
    if (!config.enable_filter) {
      continue;
    }
    float x = config.x;
    float y = config.y;
    float length = config.size / 2.0f;

    auto will_del =
        std::remove_if(frame->cloud->mutable_points()->begin(),
                       frame->cloud->mutable_points()->end(),
                       [&](const base::PointF& point) {
                         if (point.x < x + length && point.x > x - length &&
                             point.y < y + kEgoTireWidth && point.y > y - kEgoTireWidth) {
                           return true;
                         } else {
                           return false;
                         }
                       });

    auto points = frame->cloud->mutable_points();
    points->erase(will_del, points->end());
  }
}

PERCEPTION_REGISTER_ROIFILTER(EgoServiceFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace century
