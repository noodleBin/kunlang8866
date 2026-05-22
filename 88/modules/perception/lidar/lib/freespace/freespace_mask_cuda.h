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

#include <cstdint>

namespace century {
namespace perception {
namespace lidar {

struct FreespaceCudaRayCastInput {
  const uint8_t* occupied = nullptr;
  const uint8_t* map_allowed = nullptr;
  int width = 0;
  int height = 0;
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double resolution = 0.0;
  double self_x_min = 0.0;
  double self_x_max = 0.0;
  double self_y_min = 0.0;
  double self_y_max = 0.0;
  double source_x = 0.0;
  double source_y = 0.0;
  double max_range = 0.0;
  int ray_count = 0;
};

bool CastFreespaceRaysCuda(const FreespaceCudaRayCastInput& input,
                           double* ray_ranges,
                           uint8_t* ray_valid,
                           uint8_t* ray_hit_occupied,
                           double* ray_hit_occ_x,
                           double* ray_hit_occ_y);

}  // namespace lidar
}  // namespace perception
}  // namespace century
