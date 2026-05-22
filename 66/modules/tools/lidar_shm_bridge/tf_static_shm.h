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

#pragma once

#include <cstddef>
#include <cstdint>

namespace century {
namespace tools {

constexpr size_t kMaxFrameIdLength = 64;
constexpr size_t kMaxTfStaticTransforms = 64;

#pragma pack(push, 1)
struct TransformStampedShm {
  double timestamp_sec;
  uint32_t sequence_num;
  char frame_id[kMaxFrameIdLength];
  char child_frame_id[kMaxFrameIdLength];
  double translation_x;
  double translation_y;
  double translation_z;
  double rotation_qx;
  double rotation_qy;
  double rotation_qz;
  double rotation_qw;
};

struct TransformStampedsShm {
  double timestamp_sec;
  uint32_t sequence_num;
  uint32_t transform_count;
  TransformStampedShm transforms[kMaxTfStaticTransforms];
};
#pragma pack(pop)

}  // namespace tools
}  // namespace century
