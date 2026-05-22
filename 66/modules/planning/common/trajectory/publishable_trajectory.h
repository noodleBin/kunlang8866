/******************************************************************************
 * Copyright 2017 The Century Authors. All Rights Reserved.
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
 * @file publishable_trajectory.h
 **/

#pragma once

#include "modules/planning/proto/planning.pb.h"

#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/pass_stacker_response.pb.h"

namespace century {
namespace planning {
using century::planning::PassStackerRequest;
class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  /**
   * Create a publishable trajectory based on a trajectory protobuf
   */
  explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

  double header_time() const;

  void PopulateTrajectoryProtobuf(ADCTrajectory* trajectory_pb) const;

  void SetBorrowRequest(bool has_borrow_request) {
    has_borrow_request_ = has_borrow_request;
  };
  bool HasBorrowRequest() { return has_borrow_request_; };
  PassStackerRequest HasPassStackerRequest() { return pass_stacker_request_; };
  void SetPassStackerRequest(PassStackerRequest pass_stacker_request) {
    pass_stacker_request_ = pass_stacker_request;
  };

 private:
  double header_time_ = 0.0;
  bool has_borrow_request_ = false;
  PassStackerRequest pass_stacker_request_;
};

}  // namespace planning
}  // namespace century
