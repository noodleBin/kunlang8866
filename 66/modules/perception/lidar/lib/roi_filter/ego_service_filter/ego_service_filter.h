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

#pragma once

#include <string>

#include "modules/perception/lidar/lib/interface/base_roi_filter.h"
#include "modules/perception/lidar/lib/scene_manager/roi_service/roi_service.h"

namespace century {
namespace perception {
namespace lidar {

class EgoServiceFilter : public BaseROIFilter {
 public:
  EgoServiceFilter() : BaseROIFilter() {}
  ~EgoServiceFilter() = default;

  bool Init(const ROIFilterInitOptions& options) override;

  std::string Name() const override { return "EgoServiceFilter"; }

  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;

 private:
  void FilterEgoTire(LidarFrame* frame) noexcept;

 private:
  ROIServicePtr roi_service_ = nullptr;
  ROIServiceContent roi_service_content_;
  ROIFilterInitOptions options_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace century
