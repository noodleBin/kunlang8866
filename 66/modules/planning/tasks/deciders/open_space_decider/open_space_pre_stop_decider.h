/******************************************************************************
 * Copyright 2021 The Century Authors. All Rights Reserved.
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
 * @file
 **/

#pragma once

#include <memory>
#include <vector>
#include "cyber/common/macros.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace century {
namespace planning {

class OpenSpacePreStopDecider : public Decider {
 public:
  // constructor
  OpenSpacePreStopDecider(const TaskConfig& config,
                          const std::shared_ptr<DependencyInjector>& injector);

 private:
  /*
   * @brief process function
   * @param frame   data frame
   * @param reference_line_info   reference line
   */
  century::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  /*
   * @brief get target parking space stop s
   * @param frame   data frame
   * @param reference_line_info   reference line
   * @param target_s   target parking space center s
   */
  bool CheckParkingSpotPreStop(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info,
                               double* target_s);

  bool CheckPullOverPreStop(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info,
                            double* target_s);
  bool CheckDeadEndPreStop(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info,
                           double* target_x);

  void SetDeadEndStopFence(const double target_x,
                           Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

  /*
   * @brief
   * @param target_s       target parking space center s
   * @param frame          data frame
   * @param reference_line_info   reference line
   */
  void SetParkingSpotStopFence(const double target_s, Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);

  void SetPullOverStopFence(const double target_s, Frame* const frame,
                            ReferenceLineInfo* const reference_line_info);

  static bool SelectTargetDeadEndJunction(
        std::vector<hdmap::JunctionInfoConstPtr>* junctions,
        const century::common::PointENU& dead_end_point,
        hdmap::JunctionInfoConstPtr* target_junction);

 private:
  // stop reason
  static constexpr const char* OPEN_SPACE_STOP_ID = "OPEN_SPACE_PRE_STOP";
  // open space stop configure
  OpenSpacePreStopDeciderConfig open_space_pre_stop_decider_config_;
  bool routing_in_flag_ = true;
  common::PointENU dead_end_point_;
};

}  // namespace planning
}  // namespace century
