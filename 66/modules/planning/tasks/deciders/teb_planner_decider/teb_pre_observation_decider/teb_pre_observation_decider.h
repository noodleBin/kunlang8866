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
 * @file teb_pre_observation_decider.h
 **/

#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/proto/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/open_space/teb/optimal_planner.h"
#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/tasks/deciders/teb_planner_decider/teb_pre_observation_decider/teb_tar_decider_fsm.h"

namespace century {
namespace planning {

class TEBPreObservationDecider : public Decider {
 public:
  explicit TEBPreObservationDecider(
      const TaskConfig &config,
      const std::shared_ptr<DependencyInjector> &injector);
  ~TEBPreObservationDecider() = default;

  static void CalcSLBasedPosition(const Vec2d &start_point,
                                  const double start_heading,
                                  const Vec2d &end_point, Vec2d *const result);

 private:
  /**
   * @brief main function.
   *
   */
  century::common::Status Process(Frame *frame) override;
  /**
   * @brief Determine if there is a oncoming vehicle ahead.
   * @details Consider two aspects:
   * 1, the target from the failure of the oncoming vehicle in prp.
   * 2, the target from the fallback of teb trajectory.
   * 3, the target from the reasssessment of rescue teb scnario.
   *
   */
  void OncomingTarDecider(Frame *const frame);
  bool CheckPrpTarInfo();
  bool CheckTarWithTrajInter();
  bool CheckTarWithAdcStatus();
  /**
   * @brief Provide a non fsm method to deal with tar case.
   */
  void SimpleHandleTar();
  /**
   * @brief fill in the Tar info by id.
   */
  bool UpdateTarInfoWithId();

  century::planning::RescueStatus *rescue_status_;
  common::math::Vec2d rescue_end_point_;
  const hdmap::HDMap *hdmap_ = nullptr;
  century::common::VehicleParam vehicle_params_;
  common::VehicleState vehicle_state_;
  ThreadSafeIndexedObstacles *obstacles_by_frame_;
  // Tar is the case of oncoming vhicle existence.
  std::shared_ptr<TEBTarDeciderFsm> tar_decider_fsm_;
  std::shared_ptr<TarVehicleInfo> prp_reverse_;
  std::shared_ptr<TarVehicleInfo> tar_reverse_;
  uint32_t stop_time_;
};

}  // namespace planning
}  // namespace century
