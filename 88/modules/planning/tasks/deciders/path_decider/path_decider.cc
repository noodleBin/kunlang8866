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
 * @file
 **/

#include "modules/planning/tasks/deciders/path_decider/path_decider.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "modules/map/proto/map_electric_fence.pb.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/common/util/util.h"

using century::common::math::Polygon2d;
using century::hdmap::HDMapUtil;
using century::hdmap::JunctionInfo;
using century::hdmap::JunctionInfoConstPtr;
using century::hdmap::Junction;
using century::common::VehicleState;

namespace century {
namespace planning {
namespace {
constexpr double kDistanceToObstacle = 3.0;
constexpr double kComfortableAcc = 3.0;
constexpr double kBreakBuff = 1.0;
constexpr double kStackerLength = 8.1;
constexpr double kStackerWidth = 4.4;
constexpr double kStackerFrontToLocPoint = 0.8;
constexpr double kStackerConsiderDistance = 12.0;
constexpr double kElectricFenceSearchRadius = 100.0;

constexpr double kPathPointOffLineLatThreshold = 3.0;
constexpr double kPathPointOffEndPointLongThreshold = 5.0;
constexpr double kPathPointLatBuffer = 0.0;
constexpr double kPathPointLongBuffer = 0.0;
constexpr double kVehLatBuffer = 0.2;
constexpr double kVehLongBuffer = 0.5;
constexpr double kMaxElectricFenceCollisionLength = 11;
constexpr double kMinElectricFenceCollisionLength = 3;
constexpr double kBrakeDeceleration = 1.8;
constexpr double kMinKappa = 0.01;
constexpr double kStraightKappa = 0.001;
constexpr double kMinLatOffest = 0.2;
constexpr double kMaxLatOffest = 1.2;
constexpr int kCntInRefLine = 5;

constexpr double kDegrees = 90.0;
constexpr double kMinAngleDiff = 2.0;
constexpr double kMinLateralDiff = 1.0;
constexpr double kMaxLateralDiff = 5.0;
constexpr double kStartNoticeStackerDistance = 50.0;
constexpr double kStartConsiderStackerDistance = 50.0;
constexpr double kStackerDisappear = 10.0;
constexpr double kDistanceToTargetPointInLateral = 12.0;
constexpr double kDistanceToTargetPointInLon = 3.0;
constexpr int kStackerDiacoverCount = 5;
constexpr double kSameDirectionAngle = 0.785;//45
constexpr double kReverseDirectionAngle = 2.355;//135
constexpr double kStaticSameDirectionAngle = 0.35;//20
constexpr double kStaticReverseDirectionAngle = 2.8;//160
constexpr double kDynamicStackerOverlapBuffer = 1.0;
constexpr double kStaticStackerRemainSpaceBuffer = 1.0;
constexpr int kDynamicStackerStillCount = 100;
constexpr int kStaticStackerStillCount = 100;
constexpr int kStaticToDynamicStackerCount = 100;
constexpr double kDefaultRoadWidth = 2.5;
constexpr double kJunctionSearchRadius = 1.0;
constexpr double kStackerRealLength = 8.1;
constexpr double kStackerRealWidth = 4.4;
constexpr double kEnterPointX = 508.166339;
constexpr double kEnterPointY = 416.410822;
constexpr double kOutPointX = 510.143127;
constexpr double kOutPointY = 421.620010;
constexpr double kNearReferenceLineBuffer = 0.3;
constexpr double kNearDestinatonBuffer = 1.0;
constexpr double kConsiderStopDistance = 50.0;
constexpr double kStopSpeed = 0.1;
constexpr int kStopTimes = 100;
constexpr double kWheelCraneStopDistance = 60.0;
// constexpr double kNarrowPassLatBuffer = -0.175;
constexpr double kGateSearchRadius = 25.0;
constexpr double kStopAcc = 1.0;
constexpr double kWheelCraneConsiderLateral = 20.0;
constexpr double kWestInPointX = -3268.33545157;
constexpr double kWestInPointY = 2289.43241804;
constexpr double kWestOutPointX = -3319.67728402;
constexpr double kWestOutPointY = 2366.24827517;
constexpr double kEastOutStopPointX = -2454.96063803;
constexpr double kEastOutStopPointY = 2222.18270931;
constexpr double kEastOutStopForwardDistance = 40.0;
constexpr double kEastInPointX = -2501.77906154;
constexpr double kEastInPointY = 2179.29088668;
constexpr double kEastOutPointX = -2473.97899231;
constexpr double kEastOutPointY = 2238.25796282;
constexpr double kLonOverlapDistance = 5.0;
constexpr double kLatOverlapDistance = 4.0;
constexpr int kWorkingStackerCountTimes = 6;
constexpr int kNoWheelCraneCountTimes = 10;
constexpr int kStaticStillCountTimes = 10;
}  // namespace
using century::common::ErrorCode;
using century::common::Status;
using century::common::VehicleConfigHelper;
using century::common::math::Box2d;

PathDecider::PathDecider(const TaskConfig &config,
                         const std::shared_ptr<DependencyInjector> &injector)
    : Task(config, injector) {}

Status PathDecider::Execute(Frame *frame,
                            ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);

  if (config_.path_decider_config().enable_check_headbang_path()) {
    CheckHeadbangPath(frame, reference_line_info);
  }

  *(injector_->mutable_last_path_data()) = reference_line_info->path_data();

  return Process(reference_line_info, reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

Status PathDecider::Process(ReferenceLineInfo *reference_line_info,
                            const PathData &path_data,
                            PathDecision *const path_decision) {
  // skip path_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }
  std::string blocking_obstacle_id;
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->Id();
  }
  reference_line_info_ = reference_line_info;
  MakePathDecision(path_data, reference_line_info, path_decision);
  return MakeObjectDecision(path_data, blocking_obstacle_id, path_decision);
}

Status PathDecider::MakeObjectDecision(const PathData &path_data,
                                       const std::string &blocking_obstacle_id,
                                       PathDecision *const path_decision) {
  return MakeStaticObstacleAndElectricFenceDecision(
      path_data, blocking_obstacle_id, path_decision);
}

// TODO(jiacheng): eventually this entire "path_decider" should be retired.
// Before it gets retired, its logics are slightly modified so that everything
// still works well for now.
Status PathDecider::MakeStaticObstacleAndElectricFenceDecision(
    const PathData &path_data, const std::string &blocking_obstacle_id,
    PathDecision *const path_decision) {
  // Sanity checks and get important values.
  ACHECK(path_decision);

  double collision_distance = reference_line_info_->AdcSlBoundary().end_s();
  std::string msg = "path is not within the drivable area.";
  if (FLAGS_enable_electric_fence_drivable_area &&
      !IsInDrivableArea(path_data, collision_distance, msg)) {
    AERROR << msg;
    reference_line_info_->SetEStopStatus(
        true, "path is not within the drivable area.");

    double soft_stop_wall_s = collision_distance;

    const std::string stop_wall_id = "out of drivable area";
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                            StopReasonCode::STOP_REASON_ELECTRIC_FENCE,
                            wait_for_obstacles, "OutOfDrivableAreaStop", frame_,
                            reference_line_info_);

    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  const auto &frenet_path = path_data.frenet_frame_path();
  if (frenet_path.empty()) {
    const std::string msg = "Path is empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // bool is_D_zone_to_yuan2 = IsDZoneToYuan2(path_data);

  // TODO(zongxing): adc_l should reset with real data.
  double stop_distance = GetStopDistance();

  // Go through every obstacle and make decisions.
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    ADEBUG << "obstacle_id[<< " << obstacle->Id() << "] type["
           << PerceptionObstacle_Type_Name(obstacle->Perception().type())
           << "]";
    if (obstacle->IsIgv()) {
      stop_distance = FLAGS_distance_to_igv;
    }
    if (obstacle->Perception().type() ==
            perception::PerceptionObstacle::STACKER ||
        obstacle->Perception().type() ==
            perception::PerceptionObstacle::FORKLIFT_STACKER) {
      stop_distance = FLAGS_stop_distance_to_stacker;
    }

    // if (is_D_zone_to_yuan2) {
    //   stop_distance = FLAGS_license_plate_recognition_distance;
    // }

    if (IsNeedToSkipDecisionMaking(path_data, blocking_obstacle_id, *obstacle,
                                   stop_distance, path_decision)) {
      continue;
    }

    // 0. IGNORE by default and if obstacle is not in path s at all.
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();
    const auto &sl_boundary = obstacle->PerceptionSLBoundary();
    const auto &vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    const double half_length = vehicle_param.length() * 0.5;
    const double longitudinal_safe_buffer =
        half_length + FLAGS_longitudinal_check_buffer;
    if (sl_boundary.end_s() <
            frenet_path.front().s() - longitudinal_safe_buffer ||
        sl_boundary.start_s() >
            frenet_path.back().s() + longitudinal_safe_buffer) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle->Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                        object_decision);
      continue;
    }
    GetPathPointInTurnLaneRange(path_data);
    MakeObstacleLateralDecision(path_data, path_decision, *obstacle,
                                stop_distance);
  }
  return Status::OK();
}

bool PathDecider::IsDZoneToYuan2(const PathData &path_data) {
  if (IsPointInJunction(injector_->vehicle_state()->x(),
                        injector_->vehicle_state()->y(), kGateSearchRadius,
                        Junction::DongJiaZhen_Middle_Gate)) {
    for (const auto &path_point : path_data.discretized_path()) {
      if (IsPointInJunction(path_point.x(), path_point.y(), 0.0,
                            Junction::DongJiaZhen_Middle_Gate)) {
        return true;
      }
    }
  }
  return false;
}
void PathDecider::GetPathPointInTurnLaneRange(const PathData& path_data) {
  const auto& adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  for (const auto& path_point : path_data.discretized_path()) {
    double path_step = 1.0;
    bool ref_line_has_turn = false;
    double s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5 +
               path_point.s();
    std::pair<double, bool> is_in_turn = {s, false};
    for (size_t i = 0; i < FLAGS_distance_to_turnlane * 0.5; i += path_step) {
      hdmap::LaneInfoConstPtr locate_lane =
          reference_line_info_->LocateLaneInfo(s + i);
      if (nullptr != locate_lane) {
        // AINFO<<"1";
        common::SLPoint sl;
        sl.set_l(0);
        sl.set_s(s + i);
        common::math::Vec2d xy;
        if (!reference_line_info_->reference_line().SLToXY(sl, &xy)) {
          //  AINFO<<"CONTINUE";

          reference_line_info_->FillInPathPointInTurn(is_in_turn);
          continue;
        }
        double s_projection = s + i;
        const auto& reference_point =
            reference_line_info_->reference_line().GetNearestReferencePoint(
                s_projection);
        //  AINFO<<"s_projection = "<<s_projection<<"  reference_point kappa =
        //  "<<reference_point.kappa();
        double minddle_kappa = reference_point.kappa();
        // AINFO << "minddle_kappa = " << minddle_kappa;
        if (std::fabs(minddle_kappa) > kMinKappa ||
            locate_lane->lane().turn() != hdmap::Lane::NO_TURN) {
          // if (std::fabs(minddle_kappa) > kMinKappa) {
          ref_line_has_turn = true;
        }
      }
      if (ref_line_has_turn) {
        is_in_turn.second = true;
        reference_line_info_->FillInPathPointInTurn(is_in_turn);
        // AINFO << "       path_point in " << s << "  reach turn";
      } else {
        // AINFO << "       path_point in " << s << "  no turn";
        reference_line_info_->FillInPathPointInTurn(is_in_turn);
      }
    }
  }
}

void PathDecider::MakeObstacleLateralDecision(const PathData &path_data,
                                              PathDecision *const path_decision,
                                              const Obstacle &obstacle,
                                              const double stop_distance) {
  const auto &vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double half_width = vehicle_param.width() * 0.5;
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;
  const auto &sl_boundary = obstacle.PerceptionSLBoundary();
  const double curr_l =
      path_data.frenet_frame_path().GetNearestPoint(sl_boundary).l();
  ObjectDecisionType object_decision;
  object_decision.mutable_ignore();
  // 1. IGNORE if laterally too far away.
  if (curr_l - lateral_radius > sl_boundary.end_l() ||
      curr_l + lateral_radius < sl_boundary.start_l()) {
    path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle.Id(),
                                      object_decision);
    return;
  }
  // if is higher obs skip 
  if (obstacle.IsHigherObs()) {
    AINFO << "no lateral decision for higher obs  "<<obstacle.Id();
    return;
  }
  // 2. STOP if polygon collision.
  double min_nudge_l = half_width + FLAGS_lateral_check_buffer;
  double stop_limit_s = sl_boundary.start_s();
  double adc_lateral_collision_safe_buffer =
      config_.path_decider_config().adc_lateral_collision_safe_buffer();
  const auto obs_type = obstacle.Perception().type();
  if ((perception::PerceptionObstacle::VEHICLE == obs_type &&
       obstacle.IsStatic()) ||
      perception::PerceptionObstacle::STACKER == obs_type ||
      perception::PerceptionObstacle::FORKLIFT_STACKER == obs_type) {
    adc_lateral_collision_safe_buffer =
        adc_lateral_collision_safe_buffer + FLAGS_car_type_lateral_buffer;
  }
  bool is_tire_lifter = obstacle.Perception().type() ==
                        perception::PerceptionObstacle::WHEELCRANE;
  if (is_tire_lifter) {
    // todo tire_lifer collision
    adc_lateral_collision_safe_buffer =
        adc_lateral_collision_safe_buffer + FLAGS_car_type_lateral_buffer;
  }

  if (FLAGS_allow_narrow_pass && injector_->is_adc_in_gate_junction_) {
    adc_lateral_collision_safe_buffer = 0.0;
  }
  // use st boundary collision check to make decision
  if(injector_->enable_shrink_collision_buffer_){
    adc_lateral_collision_safe_buffer = 0.0;
  }
  if (IsCollisionWithObstacle(path_data, obstacle,
                              adc_lateral_collision_safe_buffer,
                              &stop_limit_s)) {
    AINFO << "path will collision with obstacle: " << obstacle.Id()
          << ", stop_limit_s: " << stop_limit_s;
    const std::string stop_tag = "PathDecider/nearest-stop";
    //change 
    // util::GenerateObjectStopDecision(
    //     stop_limit_s, -GetObstacleStopDistance(obstacle, stop_distance),
    //     stop_tag, *reference_line_info_, object_decision.mutable_stop());

    util::GenerateObjectStopDecision(
        obstacle, -GetObstacleStopDistance(obstacle, stop_distance), stop_tag,
        *reference_line_info_, object_decision.mutable_stop());
    if (path_decision->MergeWithMainStop(
            object_decision.stop(), obstacle.Id(),
            reference_line_info_->reference_line(),
            reference_line_info_->AdcSlBoundary())) {
      path_decision->AddLongitudinalDecision(stop_tag, obstacle.Id(),
                                             object_decision);
    } else {
      ObjectDecisionType object_decision;
      object_decision.mutable_ignore();
      path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                             obstacle.Id(), object_decision);
    }
  } else {
    // 3. NUDGE if laterally very close.
    if (sl_boundary.end_l() < curr_l - min_nudge_l) {  // &&
      // sl_boundary.end_l() > curr_l - min_nudge_l - 0.3) {
      // LEFT_NUDGE
      const std::string nudge_tag = "PathDecider/left-nudge";
      ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
      object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
      object_nudge_ptr->set_tag(nudge_tag);
      object_nudge_ptr->set_distance_l(FLAGS_static_obstacle_buffer);
      path_decision->AddLateralDecision(nudge_tag, obstacle.Id(),
                                        object_decision);
    } else if (sl_boundary.start_l() > curr_l + min_nudge_l) {  // &&
      // sl_boundary.start_l() < curr_l + min_nudge_l + 0.3) {
      // RIGHT_NUDGE
      const std::string nudge_tag = "PathDecider/right-nudge";
      ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
      object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
      object_nudge_ptr->set_tag(nudge_tag);
      object_nudge_ptr->set_distance_l(-FLAGS_static_obstacle_buffer);
      path_decision->AddLateralDecision(nudge_tag, obstacle.Id(),
                                        object_decision);
    }
  }
}

void PathDecider::MakePathDecision(const PathData &path_data,
                                   ReferenceLineInfo *reference_line_info,
                                   PathDecision *const path_decision) {
  MakeFallbackPathDecision(path_data, reference_line_info);
  if (FLAGS_enable_use_pass_stacker) {
    MakeStackerObjectDecision(path_data, reference_line_info, path_decision);
    MakeWheelCraneObjectDecision(path_data, reference_line_info, path_decision);
  }
  if (FLAGS_enable_use_huamn_in_junction) {
    MakeJunctionDecision(path_data, reference_line_info, path_decision);
  }
  if (FLAGS_enable_use_pass_stacker_with_perception) {
    MakeStackerObjectDecisionUsePerception(path_data, reference_line_info,
                                           path_decision);
  }
  // handle top-bull waiting stop wall
  HandleTopBullWaiting(path_data, reference_line_info, path_decision);
  MakeWeighingDecision(path_data, reference_line_info, path_decision);
  MakeTemporaryParkingDecision(path_data, reference_line_info, path_decision);
  MakeBarrierDecision(path_data, reference_line_info, path_decision);
}

void PathDecider::MakeJunctionDecision(const PathData &path_data,
                          ReferenceLineInfo *reference_line_info,
                          PathDecision *const path_decision) {
  if (IsPointInJunction(injector_->vehicle_state()->x(),
                        injector_->vehicle_state()->y(), 0.0, Junction::DongJiaZhen_D_WEST)) {
    for (const auto &path_point : path_data.discretized_path()) {
      if (IsPointInJunction(path_point.x(), path_point.y(), 0.0, Junction::DongJiaZhen_Middle_Gate)) {
        injector_->SetReroutinForHuman(true);
        return;
      }
    }
  }
  return;
}

void PathDecider::HandleTopBullWaiting(const PathData &path_data,
                                      ReferenceLineInfo *reference_line_info,
                                      PathDecision *const path_decision) {
  auto planning_status = injector_->planning_context()->mutable_planning_status();
  if (nullptr == planning_status) {
    return;
  }
  if (!planning_status->has_top_bull()) {
    return;
  }
  const auto &top_bull = planning_status->top_bull();
  if (!top_bull.is_in_top_bull()) {
    return;
  }
  if (top_bull.action_type() != planning::TopBullStatus::WAITING &&
      top_bull.action_type() != planning::TopBullStatus::REVERSE) {
    return;
  }

  if (planning::TopBullStatus::REVERSE == top_bull.action_type() &&
      !planning_status->mutable_top_bull()->ego_complete_action()) {
    return;
  }
  const auto &reference_line = reference_line_info->reference_line();
  common::SLPoint adc_sl;
  common::math::Vec2d adc_point;
  adc_point.set_x(reference_line_info->vehicle_state().pose().position().x());
  adc_point.set_y(reference_line_info->vehicle_state().pose().position().y());
  if (!reference_line.XYToSL(adc_point, &adc_sl)) {
    return;
  }

  const auto adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  double soft_stop_wall_s = adc_sl.s() + vehicle_param.length() * 0.5;
  if (soft_stop_wall_s > adc_sl_boundary.end_s() + reference_line.Length()) {
    return;
  }
  soft_stop_wall_s = std::max(adc_sl_boundary.end_s(), soft_stop_wall_s);

  const std::string stop_wall_id = "STOP_REASON_TOP_BULL";
  std::vector<std::string> wait_for_obstacles;
  util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                          StopReasonCode::STOP_REASON_OBSTACLE,
                          wait_for_obstacles, "STOP_REASON_TOP_BULL", frame_,
                          reference_line_info);

  bool is_adc_stop = std::fabs(reference_line_info->vehicle_state().linear_velocity()) < kStopSpeed;
  if (is_adc_stop) {
    planning_status->mutable_top_bull()->set_ego_complete_action(true);
  }
}

bool PathDecider::CheckIsWorkingStacker(const std::string &stacker_id) {
  auto *find_obs = reference_line_info_->path_decision()->Find(stacker_id);

  // const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  if (find_obs == nullptr) {
    return false;
  }
  auto nearst_stacker_sl = find_obs->PerceptionSLBoundary();
  bool is_static_stacker = find_obs->IsStatic();
  bool is_lateral_direction_stacker = false;
  double nearst_stacker_heading =
      find_obs->Perception().theta();  // static no use speedheadng 
  const auto refer_point =
      reference_line_info_->reference_line().GetNearestReferencePoint(
          nearst_stacker_sl.end_s());
  double adc_ref_heading = refer_point.heading();

  AINFO << "nearst_stacker_heading = " << nearst_stacker_heading;
  double diff_heading =
      common::math::NormalizeAngle(nearst_stacker_heading - adc_ref_heading);
  AINFO << "diff_heading = " << diff_heading;
  double diff_angle = std::fabs(diff_heading);
  is_lateral_direction_stacker =
      diff_angle > kSameDirectionAngle && diff_angle < kReverseDirectionAngle;
  AINFO << "is_lateral_direction_stacker = " << is_lateral_direction_stacker;
  bool face_right = diff_heading < 0.0;
  bool face_left = diff_heading > 0.0;
  AINFO << "face_right = " << face_right;
  AINFO << "face_left = " << face_left;
  if (!is_lateral_direction_stacker || !is_static_stacker) {
    AINFO << "no lateral direction stacker or no static stacker";
    return false;
  }
  // lon and lateral has overlap.
  for (const auto *obstacle : reference_line_info_->path_decision()->obstacles().Items()) {
    // only vehicle
    if (obstacle->Perception().type() !=
        perception::PerceptionObstacle::VEHICLE) {
      continue;
    }
    // AINFO<<"obstacle = "<<obstacle->Id();
    const auto &obs_sl = obstacle->PerceptionSLBoundary();
    bool is_lon_overlap =
        !(nearst_stacker_sl.start_s() > obs_sl.end_s() + kLonOverlapDistance ||
          nearst_stacker_sl.end_s() < obs_sl.start_s() - kLonOverlapDistance);
    AINFO << "is_lon_overlap = " << is_lon_overlap;
    bool is_lateral_consider_obs = false;
    if (face_right) {
      is_lateral_consider_obs =
          (nearst_stacker_sl.end_l() > obs_sl.start_l() &&
           nearst_stacker_sl.start_l() - kLatOverlapDistance < obs_sl.end_l());
    }
    if (face_left) {
      is_lateral_consider_obs =
          (nearst_stacker_sl.end_l() + kLatOverlapDistance > obs_sl.start_l() &&
           nearst_stacker_sl.start_l() < obs_sl.end_l());
    }
    AINFO << "is_lateral_consider_obs = " << is_lateral_consider_obs;
    if (is_lateral_consider_obs && is_lon_overlap) {
      AINFO << "stacker front has vehicle";
      return true;
    }
  }

  return false;
}

void PathDecider::CheckForDynamicStacker(const std::string& stacker_id,
                                         bool* is_dynamic_stacker,double* nearst_stacker_heading) {
  auto* find_obs = reference_line_info_->path_decision()->Find(stacker_id);
  double adc_end_s = reference_line_info_->AdcSlBoundary().end_s();
  double adc_start_s = reference_line_info_->AdcSlBoundary().start_s();
  const auto& adc_sl = reference_line_info_->AdcSlBoundary();
  bool enable_pass_stacker = false;
  if (nullptr == find_obs) {
    return;
  }
  auto nearst_stacker_sl = find_obs->PerceptionSLBoundary();
  *is_dynamic_stacker = !find_obs->IsStatic();
  auto stackers_info = injector_->stackers_info();
  if(is_dynamic_stacker){
  *nearst_stacker_heading = find_obs->SpeedHeading();
  }
  for (const auto &stacker_info : stackers_info.stacker_info()) {
    std::string stacker_id_perception = "stacker_" + stacker_info.stacker_id();
    if (stacker_id_perception == stacker_id) {
      *is_dynamic_stacker = std::fabs(stacker_info.speed()) > 0.1;
    }
  }
  if (!(*is_dynamic_stacker)) {
    return;
  }
  bool is_right_obs = nearst_stacker_sl.end_l() + kDynamicStackerOverlapBuffer <
                      adc_sl.start_l();
  bool is_left_obs =
      nearst_stacker_sl.start_l() - kDynamicStackerOverlapBuffer >
      adc_sl.end_l();
  bool is_no_overlap_with_adc = (is_right_obs || is_left_obs);
  if (is_no_overlap_with_adc) {
    const auto refer_point =
        reference_line_info_->reference_line().GetNearestReferencePoint(
            nearst_stacker_sl.end_s());
    double adc_ref_heading = refer_point.heading();
    double diff_heading =
        common::math::NormalizeAngle(*nearst_stacker_heading - adc_ref_heading);
    double diff_angle = std::fabs(diff_heading);
    if (is_right_obs && (diff_heading < 0.0)) {
      enable_pass_stacker = true;
    }
    if (is_left_obs && diff_heading > 0.0) {
      enable_pass_stacker = true;
    }
    bool is_no_lateral_overlap =
        nearst_stacker_sl.end_l() + kDynamicStackerOverlapBuffer <
            adc_sl.start_l() ||
        nearst_stacker_sl.start_l() - kDynamicStackerOverlapBuffer >
            adc_sl.end_l();
    bool is_reverse_or_same_direction_stacker =
        diff_angle < kSameDirectionAngle || diff_angle > kReverseDirectionAngle;
    if (is_no_lateral_overlap && is_reverse_or_same_direction_stacker) {
      ++dynamic_still_count_;
      dynamic_still_count_ = std::min(1000, dynamic_still_count_);
      enable_pass_stacker = true;
      static_still_count_ = 0;
    } else {
      dynamic_still_count_ = 0;
    }
    if (dynamic_still_count_ > kDynamicStackerStillCount) {
      enable_pass_stacker = true;
    }
  }
  bool is_back_stacker = adc_start_s > nearst_stacker_sl.end_s();
  if (is_back_stacker) {
    enable_pass_stacker = true;
  }
  if (!enable_pass_stacker) {
    double soft_stop_wall_s =
        nearst_stacker_sl.start_s() - FLAGS_pass_stacker_stop_distance;
    if (soft_stop_wall_s >
        adc_end_s +
            reference_line_info_->path_data().discretized_path().Length()) {
      return;
    }
    soft_stop_wall_s = std::max(adc_end_s, soft_stop_wall_s);
    const std::string stop_wall_id = "STOP_REASON_STACKER";
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                            StopReasonCode::STOP_REASON_STACKER,
                            wait_for_obstacles, "STOP_REASON_STACKER", frame_,
                            reference_line_info_);
    injector_->stacker_static_to_dynamic_times_++;
    injector_->stacker_static_to_dynamic_times_ =
        std::min(1000, injector_->stacker_static_to_dynamic_times_);
    if (injector_->stacker_static_to_dynamic_times_ >
        kStaticToDynamicStackerCount) {
      planning::PassStackerResponse pass_stacker_response;
      pass_stacker_response.set_pass_stacker_response_type(
          planning::PassStackerResponseType::ORIGINAL);
      pass_stacker_response.set_has_response(false);
      injector_->set_pass_stacker_response(pass_stacker_response);
      injector_->stacker_static_to_dynamic_times_ = 0;
      planning::BorrowResponse borrow_response;
      borrow_response.set_response_type(planning::ResponseType::UNTREATED);
      borrow_response.set_block_obs_id("");
      borrow_response.set_has_response(false);
      injector_->set_borrow_response(borrow_response);
    }
    PassStackerRequest pass_stacker_request;
    pass_stacker_request.mutable_header()->set_timestamp_sec(
        Clock::NowInSeconds());
    pass_stacker_request.set_stacker_id(stacker_id);
    pass_stacker_request.set_request_for_pass_stacker(true);
    reference_line_info_->SetPassStackerRequest(pass_stacker_request);
    PassStackerRequest notice_stacker;
    notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    notice_stacker.set_stacker_id(stacker_id);
    notice_stacker.set_request_for_pass_stacker(true);
    notice_stacker.set_request_type(
        planning::PassStackerRequestType::PASS_READY);
    reference_line_info_->SetNeedNoticeStacker(notice_stacker);
    injector_->pass_stacker_id_ =
        injector_->pass_stacker_response().stacker_id();
    return;
  } else {
    injector_->stacker_static_to_dynamic_times_ = 0;
  }
}
void PathDecider::SetPassingNotice(const std::string& stacker_id) {
  PassStackerRequest notice_stacker;
  notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
  notice_stacker.set_stacker_id(stacker_id);
  notice_stacker.set_request_for_pass_stacker(true);
  notice_stacker.set_request_type(planning::PassStackerRequestType::PASSING);
  reference_line_info_->SetNeedNoticeStacker(notice_stacker);
  injector_->pass_stacker_id_ = injector_->pass_stacker_response().stacker_id();
}
void PathDecider::MakeNaearstStackerObjectDecision(
    const std::string& stacker_id) {
  if (CheckStartPonitorEndPointNearStackerForStacker(stacker_id, frame_,
                                                     reference_line_info_)) {
    return;
  }
  bool has_pass_stacker_response =
      (injector_->pass_stacker_response().pass_stacker_response_type() ==
       planning::PassStackerResponseType::PASS);
  if (!has_pass_stacker_response) {
    injector_->is_manual_pass_stacker_ = false;
    injector_->is_pass_dynamic_stacker_ = false;
  }
  auto *find_obs = reference_line_info_->path_decision()->Find(stacker_id);
  double adc_end_s = reference_line_info_->AdcSlBoundary().end_s();
  double adc_start_s = reference_line_info_->AdcSlBoundary().start_s();
  const auto &adc_sl = reference_line_info_->AdcSlBoundary();
  bool enable_pass_stacker = false;
  if (find_obs == nullptr) {
    return;
  }
  auto nearst_stacker_sl = find_obs->PerceptionSLBoundary();
  bool  is_dynamic_stacker= !find_obs->IsStatic();
  auto stackers_info = injector_->stackers_info();
  double nearst_stacker_heading = find_obs->Perception().theta();
  CheckForDynamicStacker(stacker_id, &is_dynamic_stacker,&nearst_stacker_heading);
  if (has_pass_stacker_response && injector_->is_manual_pass_stacker_) {
    if (stacker_id == "stacker_"+injector_->pass_stacker_response().stacker_id()) {
      SetPassingNotice(stacker_id);
      return;
    }
  }
  injector_->stacker_static_to_dynamic_times_ = 0;
  bool is_need_left_borrow = false, is_need_right_borrow = false;
  const auto refer_point =
      reference_line_info_->reference_line().GetNearestReferencePoint(
          nearst_stacker_sl.end_s());
  double adc_ref_heading = refer_point.heading();
  double diff_heading =
      common::math::NormalizeAngle(nearst_stacker_heading - adc_ref_heading);
  double diff_angle = std::fabs(diff_heading);
  bool is_right_obs = adc_sl.start_l() > nearst_stacker_sl.end_l() + 0.5;
  bool is_left_obs = nearst_stacker_sl.start_l() > adc_sl.end_l() +0.5;
  if (diff_heading < 0.0) {
    is_need_left_borrow = true;
  }
  if (diff_heading > 0.0) {
    is_need_right_borrow = true;
  }
  double curr_road_left_width = 0.0, curr_road_right_width = 0.0;
  double past_road_left_width = kDefaultRoadWidth;
  double past_road_right_width = kDefaultRoadWidth;
  const auto &vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  if (!reference_line_info_->reference_line().GetRoadWidth(
          nearst_stacker_sl.end_s(), &curr_road_left_width,
          &curr_road_right_width)) {
    curr_road_left_width = past_road_left_width;
    curr_road_right_width = past_road_right_width;
  } else {
    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;
  }
  if (is_need_left_borrow) {
    if (past_road_left_width - nearst_stacker_sl.end_l() >
        vehicle_param.width() + kStaticStackerRemainSpaceBuffer) {
      enable_pass_stacker = true;
    }
  } else if (is_need_right_borrow) {
    if (nearst_stacker_sl.start_l() + past_road_right_width >
        vehicle_param.width() + kStaticStackerRemainSpaceBuffer) {
      enable_pass_stacker = true;
    }
  }
  bool is_reverse_or_same_direction_stacker =
      diff_angle < kStaticSameDirectionAngle || diff_angle > kStaticReverseDirectionAngle;

  if(is_reverse_or_same_direction_stacker) {
    if ((past_road_left_width - nearst_stacker_sl.end_l() >
         vehicle_param.width() + kStaticStackerRemainSpaceBuffer) ||
        (nearst_stacker_sl.start_l() + past_road_right_width >
         vehicle_param.width() + kStaticStackerRemainSpaceBuffer)) {
      enable_pass_stacker = true;
    }
  }
  bool is_working_stacker = false;
  if (CheckIsWorkingStacker(stacker_id) && stacker_id != "stacker_ITCS") {
    enable_pass_stacker = false;
    is_working_stacker = true;
  }
  if (!is_working_stacker) {
    injector_->working_stacker_count_times_++;
    injector_->working_stacker_count_times_ =
        std::min(injector_->working_stacker_count_times_, 100);
  } else {
    injector_->working_stacker_count_times_ = 0;
  }
  if (injector_->working_stacker_count_times_ < kWorkingStackerCountTimes) {
    enable_pass_stacker = false;
  }
  if (is_dynamic_stacker && (!is_right_obs && !is_left_obs)) {
    enable_pass_stacker = false;
  }
  if(enable_pass_stacker && !is_dynamic_stacker){
    static_still_count_++;
    static_still_count_ = std::min(static_still_count_, 1000);
    dynamic_still_count_ = 0;
  }else{
    static_still_count_ = 0;
  }
  if (!enable_pass_stacker || static_still_count_ > kStaticStillCountTimes) {
    injector_->is_pass_dynamic_stacker_ = false;
  }
  if (enable_pass_stacker&& (static_still_count_ > kStaticStackerStillCount || is_dynamic_stacker)) {
    if(FLAGS_allow_smi_diagonal){
      planning::PassStackerResponse pass_stacker_response;
      pass_stacker_response.set_pass_stacker_response_type(
          planning::PassStackerResponseType::PASS);
      pass_stacker_response.set_has_response(true);
      pass_stacker_response.set_stacker_id(stacker_id);
      injector_->set_pass_stacker_response(pass_stacker_response);
    }
    has_pass_stacker_response = true;
  }
  if (has_pass_stacker_response && enable_pass_stacker) {
    if (adc_start_s - FLAGS_distance_borrow_return <
        nearst_stacker_sl.end_s()) {
      if(is_dynamic_stacker){
       injector_-> is_pass_dynamic_stacker_ = true;
      }
      SetPassingNotice(find_obs->Id());
      if ((injector_->pass_stacker_response().pass_stacker_response_type() ==
           planning::PassStackerResponseType::PASS)) {
        return;
      } else {
        double soft_stop_wall_s =
            nearst_stacker_sl.start_s() - FLAGS_pass_stacker_stop_distance;
        if (soft_stop_wall_s >
            adc_end_s +
                reference_line_info_->path_data().discretized_path().Length()) {
          return;
        }
        soft_stop_wall_s = std::max(adc_end_s, soft_stop_wall_s);
        const std::string stop_wall_id = "STOP_REASON_STACKER";
        std::vector<std::string> wait_for_obstacles;
        util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                                StopReasonCode::STOP_REASON_STACKER,
                                wait_for_obstacles, "STOP_REASON_STACKER",
                                frame_, reference_line_info_);
        PassStackerRequest pass_stacker_request;
        pass_stacker_request.mutable_header()->set_timestamp_sec(
            Clock::NowInSeconds());
        pass_stacker_request.set_stacker_id(find_obs->Id());
        pass_stacker_request.set_request_for_pass_stacker(true);
        reference_line_info_->SetPassStackerRequest(pass_stacker_request);
        return;
      }
    } else {
      planning::PassStackerResponse pass_stacker_response;
      pass_stacker_response.set_pass_stacker_response_type(
          planning::PassStackerResponseType::ORIGINAL);
      pass_stacker_response.set_has_response(false);
      injector_->set_pass_stacker_response(pass_stacker_response);
      PassStackerRequest notice_stacker;
      notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
      notice_stacker.set_stacker_id(find_obs->Id());
      notice_stacker.set_request_for_pass_stacker(true);
      notice_stacker.set_request_type(planning::PassStackerRequestType::PASSED);
      injector_->is_manual_pass_stacker_ = true;
      reference_line_info_->SetNeedNoticeStacker(notice_stacker);
    }
  }
  double soft_stop_wall_s =
      nearst_stacker_sl.start_s() - FLAGS_pass_stacker_stop_distance;
  if (soft_stop_wall_s >
      adc_end_s +
          reference_line_info_->path_data().discretized_path().Length()) {
    return;
  }
  soft_stop_wall_s = std::max(adc_end_s, soft_stop_wall_s);
  const std::string stop_wall_id = "STOP_REASON_STACKER";
  std::vector<std::string> wait_for_obstacles;
  util::BuildStopDecision(
      stop_wall_id, soft_stop_wall_s, 0.0, StopReasonCode::STOP_REASON_STACKER,
      wait_for_obstacles, "STOP_REASON_STACKER", frame_, reference_line_info_);
  PassStackerRequest pass_stacker_request;
  pass_stacker_request.mutable_header()->set_timestamp_sec(
      Clock::NowInSeconds());
  pass_stacker_request.set_stacker_id(stacker_id);
  pass_stacker_request.set_request_for_pass_stacker(true);
  reference_line_info_->SetPassStackerRequest(pass_stacker_request);
  PassStackerRequest notice_stacker;
  notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
  notice_stacker.set_stacker_id(stacker_id);
  notice_stacker.set_request_for_pass_stacker(true);
  notice_stacker.set_request_type(planning::PassStackerRequestType::PASS_READY);
  reference_line_info_->SetNeedNoticeStacker(notice_stacker);
}
bool PathDecider::IsJunctionContainAdc(
    const VehicleState& vehicle_state,
    const hdmap::JunctionInfo& junction_info)  {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Compute the ADC bounding box.
  Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                              vehicle_param.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param.left_edge_to_center() -
                              vehicle_param.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(vehicle_state.heading());
  ego_center_map_frame.set_x(vehicle_state.x() + ego_center_map_frame.x());
  ego_center_map_frame.set_y(vehicle_state.y() + ego_center_map_frame.y());
  Box2d adc_box(ego_center_map_frame, vehicle_state.heading(),
                vehicle_param.length(), vehicle_param.width());
  // Check whether Junction's polygon contain ADC bounding box.
  const auto& polygon = junction_info.polygon();
  return polygon.Contains(Polygon2d(adc_box));
}
bool PathDecider::IsJunctionContainStacker(
    const common::PathPoint &stacker_point,
    const hdmap::JunctionInfo &junction_info) {
  double stacker_heading = stacker_point.theta();
  double stacker_length = kStackerRealLength;
  double stacker_width = kStackerRealWidth;
  // center to loc point distance
  double shift_distance = stacker_length * 0.5 - kStackerFrontToLocPoint;
  // Compute the ADC bounding box.
  Box2d stacker_box({stacker_point.x(), stacker_point.y()}, stacker_heading,
                    stacker_length, stacker_width);
  // reverse so need add -
  Vec2d shift_vec{-shift_distance * std::cos(stacker_heading),
                  -shift_distance * std::sin(stacker_heading)};
  stacker_box.Shift(shift_vec);
  // Check whether Junction's polygon contain ADC bounding box.
  const auto &polygon = junction_info.polygon();
  return polygon.Contains(Polygon2d(stacker_box));
}

void PathDecider::GetStackerPoseArea(ReferenceLineInfo *reference_line_info,
                                     const common::PathPoint& stacker_point, bool *stacker_in_J1,
                                     bool *stacker_in_J23) {
  const hdmap::HDMap *base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  common::PointENU pose;
  pose.set_x(stacker_point.x());
  pose.set_y(stacker_point.y());
  if (0 ==
      base_map_ptr->GetJunctions(pose, kJunctionSearchRadius, &junctions)) {
    for (const auto &ptr_junction : junctions) {
      if (IsJunctionContainStacker(stacker_point,
                               *ptr_junction)) {
        AINFO << "junction conrain stacker ";
      } else {
        AINFO << "junction no conrain satcker ";
        continue;
      }
      if (Junction::BLOCKING_AREA_J1 == ptr_junction->junction().type()) {
        *stacker_in_J1 = true;
        break;
      }
      if (Junction::BLOCKING_AREA_J2J3 == ptr_junction->junction().type()) {
        *stacker_in_J23 = true;
        break;
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }
}

void PathDecider::GetAdcPoseArea(bool *adc_in_J1, bool *adc_in_J23) {
  const hdmap::HDMap *base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  common::PointENU adc_point_enu;
  adc_point_enu.set_x(injector_->vehicle_state()->vehicle_state().x());
  adc_point_enu.set_y(injector_->vehicle_state()->vehicle_state().y());
  std::vector<JunctionInfoConstPtr> junctions;
  if (0 == base_map_ptr->GetJunctions(adc_point_enu, kJunctionSearchRadius,
                                      &junctions)) {
    for (const auto &ptr_junction : junctions) {
      if (IsJunctionContainAdc(injector_->vehicle_state()->vehicle_state(),
                               *ptr_junction)) {
        AINFO << "junction conrain adc ";
      } else {
        AINFO << "junction no conrain adc ";
        continue;
      }
      if (Junction::BLOCKING_AREA_J1 == ptr_junction->junction().type()) {
        *adc_in_J1 = true;
        AINFO << "adc in J1;";
      }
      if (Junction::BLOCKING_AREA_J2J3 == ptr_junction->junction().type()) {
        *adc_in_J23 = true;
        AINFO << "adc in J23;";
      }
      if (Junction::BLOCKING_AREA_J4 == ptr_junction->junction().type()) {
        AINFO << "adc in J4;";
      }
    }
  } else {
    ADEBUG << "Fail to get junctions from base_map.";
  }
}

void PathDecider::MakeBarrierBlockDecision(
    const PathData& path_data, ReferenceLineInfo* reference_line_info,
    PathDecision* const path_decision) {
  double stop_point_x = 0.0;
  double stop_point_y = 0.0;
  double pass_point_x = 0.0;
  double pass_point_y = 0.0;
  bool is_can_pass = false;
  if (reference_line_info->IsWestIn()) {
    if (reference_line_info->Barrier().west_north() &&
        reference_line_info->Barrier().west_south()) {
      stop_point_x = kWestInPointX;
      stop_point_y = kWestInPointY;
      pass_point_x = kWestOutPointX;
      pass_point_y = kWestOutPointY;
      is_can_pass = true;
    }

  } else if (reference_line_info->IsWestOut()) {
    if (reference_line_info->Barrier().west_north() &&
        reference_line_info->Barrier().west_south()) {
      stop_point_x = kWestOutPointX;
      stop_point_y = kWestOutPointY;
      pass_point_x = kWestInPointX;
      pass_point_y = kWestInPointY;
      is_can_pass = true;
    }
  } else if (reference_line_info->IsEastIn()) {
    if (reference_line_info->Barrier().east_north() &&
        reference_line_info->Barrier().east_south()) {
      stop_point_x = kEastInPointX;
      stop_point_y = kEastInPointY;
      pass_point_x = kEastOutPointX;
      pass_point_y = kEastOutPointY;
      is_can_pass = true;
    }
  } else if (reference_line_info->IsEastOut()) {
    if (reference_line_info->Barrier().east_north() &&
        reference_line_info->Barrier().east_south()) {
      stop_point_x = kEastOutPointX;
      stop_point_y = kEastOutPointY;
      pass_point_x = kEastInPointX;
      pass_point_y = kEastInPointY;
      is_can_pass = true;
    }
  }
  if (!is_can_pass) {
    return;
  }
  common::SLPoint stop_point_sl;
  common::SLPoint pass_point_sl;
  const auto& reference_line = reference_line_info->reference_line();
  common::math::Vec2d stop_point;
  common::math::Vec2d pass_point;
  stop_point.set_x(stop_point_x);
  stop_point.set_y(stop_point_y);
  pass_point.set_x(pass_point_x);
  pass_point.set_y(pass_point_y);
  if (!reference_line.XYToSL(stop_point, &stop_point_sl)) {
    return;
  }
  if (!reference_line.XYToSL(stop_point, &pass_point_sl)) {
    return;
  }
  auto adc_sl = reference_line_info->AdcSlBoundary();
  double adc_center_s = (adc_sl.start_s() + adc_sl.end_s()) * 0.5;
  if (adc_center_s > stop_point_sl.s() && adc_center_s < pass_point_sl.s()) {
    reference_line_info_->SetIsPassingGate(true);
  }
}
void PathDecider::MakeBarrierDecision(const PathData& path_data,
                                      ReferenceLineInfo* reference_line_info,
                                      PathDecision* const path_decision) {
  double stop_point_x = 0.0;
  double stop_point_y = 0.0;
  bool is_east_out_stop = false;
  bool is_need_to_stop = false;
  if (reference_line_info->IsWestIn()) {
    AINFO << "need west in";
    if (!reference_line_info->Barrier().west_north() ||
        !reference_line_info->Barrier().west_south()) {
      stop_point_x = kWestInPointX;
      stop_point_y = kWestInPointY;
      is_need_to_stop = true;
    }

  } else if (reference_line_info->IsWestOut()) {
    if (!reference_line_info->Barrier().west_north() ||
        !reference_line_info->Barrier().west_south()) {
      stop_point_x = kWestOutPointX;
      stop_point_y = kWestOutPointY;
      is_need_to_stop = true;
    }
  } else if (reference_line_info->IsEastIn()) {
    if (!reference_line_info->Barrier().east_north() ||
        !reference_line_info->Barrier().east_south()) {
      stop_point_x = kEastInPointX;
      stop_point_y = kEastInPointY;
      is_need_to_stop = true;
    }
  } else if (reference_line_info->IsEastOut()) {
    if (!reference_line_info->Barrier().east_north() ||
        !reference_line_info->Barrier().east_south()) {
      stop_point_x = kEastOutStopPointX;
      stop_point_y = kEastOutStopPointY;
      is_east_out_stop = true;
      is_need_to_stop = true;
    }
  } else {
  }
  if(!is_need_to_stop){
    return;
  }
    common::SLPoint stop_point_sl;
  const auto& reference_line = reference_line_info->reference_line();
  common::math::Vec2d stop_point;
  stop_point.set_x(stop_point_x);
  stop_point.set_y(stop_point_y);
  if (!reference_line.XYToSL(stop_point, &stop_point_sl)) {
    return;
  }
  if (is_east_out_stop) {
    const double projected_s = std::min(
        stop_point_sl.s() - kEastOutStopForwardDistance, reference_line.Length());
    stop_point_sl.set_s(projected_s);
    stop_point_sl.set_l(0.0);
    ADEBUG << "east out stop uses projected point - "
          << kEastOutStopForwardDistance << "m, s = " << projected_s;
  }
  auto adc_sl = reference_line_info->AdcSlBoundary();
  double half_lane_width =
      (reference_line_info_->GetLaneWidthByS(adc_sl.end_s()).first +
       reference_line_info_->GetLaneWidthByS(adc_sl.end_s()).second) *
      0.5;
  double ref_length = reference_line.Length();
  bool is_in_lateral_range = stop_point_sl.l() > -half_lane_width &&
                             stop_point_sl.l() < half_lane_width;
  bool is_in_lon_range =
      stop_point_sl.s() > adc_sl.start_s() &&
      stop_point_sl.s() - ref_length < adc_sl.start_s();
  bool is_near_stop_point = is_in_lateral_range && is_in_lon_range;
  AINFO<<"ref_length = "<<ref_length;
  AINFO<<"is_in_lateral_range = "<<is_in_lateral_range;
  AINFO<<"is_in_lon_range = "<<is_in_lon_range;
  AINFO<<"is_near_stop_point = "<<is_near_stop_point;


  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double soft_stop_wall_s =
      stop_point_sl.s() + vehicle_param.length() * 0.5;
  // if ori no obs ,no build stop wall.
  if (soft_stop_wall_s > adc_sl.end_s() + ref_length) {
    return;
  }
  soft_stop_wall_s = std::max(adc_sl.end_s(), soft_stop_wall_s);
  const std::string stop_wall_id = "STOP_REASON_LINE_UP_STOP_POINT";
  std::vector<std::string> wait_for_obstacles;
  util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                          StopReasonCode::STOP_REASON_LINE_UP_STOP_POINT,
                          wait_for_obstacles, "STOP_REASON_LINE_UP_STOP_POINT",
                          frame_, reference_line_info);
  AINFO << "stop reason train stop point";

  return;

}

void PathDecider::MakeTemporaryParkingDecision(
    const PathData &path_data, ReferenceLineInfo *reference_line_info,
    PathDecision *const path_decision) {
  const auto &temporary_parking =
      frame_->local_view().temporary_parking_request;
  const auto &multi_path_temp_stop =
      frame_->local_view().multi_path_temp_stop_request;
  bool is_need_stop = false;
  if (nullptr != temporary_parking) {
    if (temporary_parking->need_stop()) {
      is_need_stop = true;
    }
  }
  if (nullptr != multi_path_temp_stop) {
    if (multi_path_temp_stop->need_stop()) {
      is_need_stop = true;
    }
  }
  if (is_need_stop) {
    double adc_speed = reference_line_info->vehicle_state().linear_velocity();
    AINFO << "adc_speed = " << adc_speed;
    AINFO << "init a = "
          << reference_line_info->vehicle_state().linear_acceleration();
    double center_s = (reference_line_info->AdcSlBoundary().end_s() +
                       reference_line_info->AdcSlBoundary().start_s()) *
                      0.5;
    double stop_distance = std::fabs(adc_speed * adc_speed * 0.5 / kStopAcc);
    double stop_s = center_s + stop_distance;
    const std::string stop_wall_id = "STOP_REASON_TEMPORARY_PARKING";
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(stop_wall_id, stop_s, 0.0,
                            StopReasonCode::STOP_REASON_TEMPORARY_PARKING,
                            wait_for_obstacles, "STOP_REASON_TEMPORARY_PARKING",
                            frame_, reference_line_info);
    AINFO << "stop reason STOP_REASON_TEMPORARY_PARKING";
    return;
  }
}
void PathDecider::MakeWeighingDecision(const PathData& path_data,
                                       ReferenceLineInfo* reference_line_info,
                                       PathDecision* const path_decision) {
  if (injector_->borrow_response().response_type() ==
          planning::ResponseType::ACCEPT &&
      injector_->borrow_response().block_obs_id() ==
          "STOP_REASON_WEIGHING_POINT") {
    AINFO << "CAN PASS FOR RECEIVE PASS WEIGHING";
    return;
  }
  // enter point
  double weighing_point_x = kEnterPointX;
  double weighing_point_y = kEnterPointY;

  common::SLPoint weighing_point_sl;
  const auto& reference_line = reference_line_info->reference_line();
  common::math::Vec2d weighing_point;
  weighing_point.set_x(weighing_point_x);
  weighing_point.set_y(weighing_point_y);
  if (reference_line.XYToSL(weighing_point, &weighing_point_sl)) {
    // near reference line
    if (std::fabs(weighing_point_sl.l()) < kNearReferenceLineBuffer) {
    } else {
      weighing_point_x = kOutPointX;
      weighing_point_y = kOutPointY;
      weighing_point.set_x(weighing_point_x);
      weighing_point.set_y(weighing_point_y);
      if (!reference_line.XYToSL(weighing_point, &weighing_point_sl)) {
        return;
      }
    }
  } else {
    weighing_point_x = kOutPointX;
    weighing_point_y = kOutPointY;
    weighing_point.set_x(weighing_point_x);
    weighing_point.set_y(weighing_point_y);
    if (!reference_line.XYToSL(weighing_point, &weighing_point_sl)) {
      return;
    }
  }

  double adc_x = reference_line_info->vehicle_state().pose().position().x();
  double adc_y = reference_line_info->vehicle_state().pose().position().y();
  double distance_to_wighting_point =
      std::sqrt((weighing_point_x - adc_x) * (weighing_point_x - adc_x) +
                (weighing_point_y - adc_y) * (weighing_point_y - adc_y));
  AINFO << "distance_to_wighting_point = " << distance_to_wighting_point;
  bool is_in_range = distance_to_wighting_point < kConsiderStopDistance;
  if (!is_in_range) {
    is_can_pass_weighing_ = false;
    return;
  }
  auto adc_sl = reference_line_info->AdcSlBoundary();
  double half_lane_width =
      (reference_line_info_->GetLaneWidthByS(adc_sl.end_s()).first +
       reference_line_info_->GetLaneWidthByS(adc_sl.end_s()).second) *
      0.5;

  bool is_in_lateral_range = weighing_point_sl.l() > -half_lane_width &&
                             weighing_point_sl.l() < half_lane_width;
  bool is_in_lon_range =
      weighing_point_sl.s() > adc_sl.start_s() &&
      weighing_point_sl.s() - kConsiderStopDistance < adc_sl.start_s();
  bool is_near_weighing_point = is_in_lateral_range && is_in_lon_range;
  if (!is_near_weighing_point) {
    is_can_pass_weighing_ = false;
    return;
  }
  // if is pass
  if (is_can_pass_weighing_) {
    return;
  }

  bool is_adc_stop =
      std::fabs(reference_line_info->vehicle_state().linear_velocity()) <
      kStopSpeed;
  bool is_reach_weighing_point =
      distance_to_wighting_point < kNearDestinatonBuffer;
  AINFO << "is_adc_stop = " << is_adc_stop;
  AINFO << "is_reach_weighing_point = " << is_reach_weighing_point;
  if (is_adc_stop && is_reach_weighing_point) {
    adc_stop_times_++;
  } else {
    if (!is_can_pass_weighing_) {
      adc_stop_times_ = 0;
    }
  }

  if (adc_stop_times_ > kStopTimes) {
    is_can_pass_weighing_ = true;
  }
  AINFO << "adc_stop_times_ = " << adc_stop_times_;
  AINFO << "is_can_pass_weighing_ = " << is_can_pass_weighing_;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double soft_stop_wall_s =
      weighing_point_sl.s() + vehicle_param.length() * 0.5;
  // if ori no obs ,no build stop wall.
  if (soft_stop_wall_s >
      adc_sl.end_s() +
          reference_line_info_->path_data().discretized_path().Length()) {
    return;
  }
  soft_stop_wall_s = std::max(adc_sl.end_s(), soft_stop_wall_s);
  const std::string stop_wall_id = "STOP_REASON_WEIGHING_POINT";
  std::vector<std::string> wait_for_obstacles;
  util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                          StopReasonCode::STOP_REASON_WEIGHING_POINT,
                          wait_for_obstacles, "STOP_REASON_WEIGHING_POINT",
                          frame_, reference_line_info);
  AINFO << "stop reason weighing point";

  return;
}
void PathDecider::MakeWheelCraneObjectDecision(
    const PathData &path_data, ReferenceLineInfo *reference_line_info,
    PathDecision *const path_decision) {
      AINFO<<"MakeWheelCraneObjectDecision =";
  double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  // const auto &adc_sl = reference_line_info->AdcSlBoundary();
  std::string nearst_wheel_crane_id = "";
  double min_wheel_crane_start_s = std::numeric_limits<double>::max();
  bool has_wheel_crane = false;
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    if (obstacle->Perception().type() !=
        perception::PerceptionObstacle::WHEELCRANE) {
      continue;
    }
    const auto &obs_sl = obstacle->PerceptionSLBoundary();
    if (obs_sl.start_s() - adc_end_s > FLAGS_wheelcrane_consider_distance) {
      AINFO << "large distance ,no consider";
      continue;
    }
    if (obs_sl.end_s() + FLAGS_distance_borrow_return < adc_start_s) {
      AINFO << "back stacker ,no consider";
      continue;
    }
    if (obs_sl.start_l() - kWheelCraneConsiderLateral >
            reference_line_info->AdcSlBoundary().end_l() ||
        obs_sl.end_l() + kWheelCraneConsiderLateral <
            reference_line_info->AdcSlBoundary().start_l()) {
      AINFO << "no lateral  stacker ,no consider";
      continue;
    }
    AINFO<<"obs_sl.start_l = "<<obs_sl.start_l()<<"   "<<obs_sl.end_l();
    has_wheel_crane = true;
    if (obs_sl.start_s() < min_wheel_crane_start_s) {
      min_wheel_crane_start_s = obs_sl.start_s();
      nearst_wheel_crane_id = obstacle->Id();
    }
  }
  AINFO << "has_wheel_crane = " << has_wheel_crane;
  AINFO<<"(injector_->borrow_response().response_type() = "<<injector_->borrow_response().response_type() ;
  AINFO<<"injector_->borrow_response().block_obs_id() = "<<injector_->borrow_response().block_obs_id();
  AINFO << "nearst_wheel_crane_id = " << nearst_wheel_crane_id;
  const auto& routing = frame_->local_view().routing;
  const auto& operation_stacker_id =
      routing->routing_request().operation_stacker_id();
  AINFO << "operation_stacker_id = " << operation_stacker_id;
  if (!operation_stacker_id.empty() && !nearst_wheel_crane_id.empty() &&
      operation_stacker_id != nearst_wheel_crane_id) {
    if (routing->routing_request().task_type() ==
            routing::YARD_OPERATIONAREA_DYNAMIC ||
        routing->routing_request().task_type() ==
            routing::RAILWAY_OPERATIONAREA_DYNAMIC) {
      AINFO << "no consider wheelcrane";
      return;
    }
  }
  // wheel crane no in range
  if(!has_wheel_crane){
    no_wheelcrane_count_++;
    AINFO<<"no_wheelcrane_count_ = "<<no_wheelcrane_count_;
    if (no_wheelcrane_count_ <kNoWheelCraneCountTimes) {
      PassStackerRequest notice_stacker;
      notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
      notice_stacker.set_stacker_id(injector_->wheelcrane_notice_.stacker_id());
      notice_stacker.set_request_for_pass_stacker(true);
      notice_stacker.set_request_type(
          planning::PassStackerRequestType::PASS_DEFAULT);
      injector_->wheelcrane_notice_ = notice_stacker;
    }else{
      PassStackerRequest notice_stacker;
      notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
      notice_stacker.set_stacker_id("");
      notice_stacker.set_request_for_pass_stacker(false);
      notice_stacker.set_request_type(
          planning::PassStackerRequestType::PASS_DEFAULT);
          injector_->wheelcrane_notice_ = notice_stacker;
    }
    return;
  }
  no_wheelcrane_count_ = 0;
  if (injector_->borrow_response().response_type() ==
          planning::ResponseType::ACCEPT &&
      "STOP_REASON_WHEEL_CRANE" ==
          injector_->borrow_response().block_obs_id()) {
    AINFO << "adc is passing wheelcrane now";
    PassStackerRequest notice_stacker;
    notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    notice_stacker.set_stacker_id(nearst_wheel_crane_id);
    notice_stacker.set_request_for_pass_stacker(true);
    notice_stacker.set_request_type(planning::PassStackerRequestType::PASSING);
    injector_->wheelcrane_notice_ = notice_stacker;
    return;
  } else {
    //has find wheelcrane,but no accept
    PassStackerRequest notice_stacker;
    notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    notice_stacker.set_stacker_id(nearst_wheel_crane_id);
    notice_stacker.set_request_for_pass_stacker(true);
    notice_stacker.set_request_type(
        planning::PassStackerRequestType::PASS_READY);
        injector_->wheelcrane_notice_ = notice_stacker;
  }
  // need reset borrow direction
  if (has_wheel_crane &&
      (injector_->borrow_response().block_obs_id().find("wheel") == std::string::npos &&
       injector_->borrow_response().block_obs_id() != nearst_wheel_crane_id)) {
    double soft_stop_wall_s =
        min_wheel_crane_start_s - kWheelCraneStopDistance;
    // if ori no obs ,no build stop wall.
    if (soft_stop_wall_s >
        adc_end_s +
            reference_line_info_->path_data().discretized_path().Length()) {
      return;
    }
    soft_stop_wall_s = std::max(adc_end_s, soft_stop_wall_s);
    if (CheckStartPonitorEndPointNearStackerForStacker(nearst_wheel_crane_id, frame_,
                                                     reference_line_info_)) {
    return;
  }

    const std::string stop_wall_id = "STOP_REASON_WHEEL_CRANE";
    std::vector<std::string> wait_for_obstacles;
    AINFO<<"BUILD STOP WALL FOR wheelcrane";
    util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                            StopReasonCode::STOP_REASON_WHEEL_CRANE,
                            wait_for_obstacles, "STOP_REASON_WHEEL_CRANE",
                            frame_, reference_line_info);
  }
  return;
}

void PathDecider::MakeStackerObjectDecision(
    const PathData &path_data, ReferenceLineInfo *reference_line_info,
    PathDecision *const path_decision) {
  double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  const auto &adc_sl = reference_line_info->AdcSlBoundary();
  double min_obs_start_s = std::numeric_limits<double>::max();
  std::string stacker_id = "";
  double min_stacker_start_s = std::numeric_limits<double>::max();
  std::string nearst_stacker_id = "";
  SLBoundary nearst_tacker_slboundary;
  bool is_get_consider_stacker = false;
    const ReferenceLine &reference_line = reference_line_info->reference_line();
  bool adc_in_J1 = false;
  bool adc_in_J23 = false;
  GetAdcPoseArea(&adc_in_J1, &adc_in_J23);
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    if (obstacle->Perception().type() !=
            perception::PerceptionObstacle::STACKER &&
        obstacle->Perception().type() !=
            perception::PerceptionObstacle::FORKLIFT_STACKER) {
      continue;
    }
    stacker_id = obstacle->Id();
    const auto &stacker_sl = obstacle->PerceptionSLBoundary();
    if (stacker_sl.start_s() - adc_end_s > kStartConsiderStackerDistance) {
      AINFO << "large distance ,no consider";
      continue;
    }
    if (stacker_sl.end_s() + FLAGS_distance_borrow_return < adc_start_s) {
      AINFO << "back stacker ,no consider";
      continue;
    }
    ADEBUG << "stacker_id = " << stacker_id
           << "     heading = " << obstacle->Perception().theta()
           << "  x = " << obstacle->Perception().position().x()
           << "  y = " << obstacle->Perception().position().y();
    ADEBUG << "stacker_sl.start_s() = " << stacker_sl.start_s()
           << "   stacker_sl.end_s()=" << stacker_sl.end_s()
           << "stacker_sl.start_L() = " << stacker_sl.start_l()
           << "   stacker_sl.end_l()=" << stacker_sl.end_l();
    double consider_start_l = adc_sl.start_l() - kStackerConsiderDistance;
    double consider_end_l = adc_sl.end_l() + kStackerConsiderDistance;
    bool is_need_consider_stacker = !(stacker_sl.start_l() > consider_end_l ||
                                      consider_start_l > stacker_sl.end_l());
    if (is_need_consider_stacker) {
      bool stacker_in_J1 = false;
      bool stacker_in_J23 = false;
      common::PathPoint stacker_point;
      stacker_point.set_x(obstacle->Perception().position().x());
      stacker_point.set_y(obstacle->Perception().position().y());
      stacker_point.set_theta(obstacle->Perception().theta());
      GetStackerPoseArea(reference_line_info, stacker_point,&stacker_in_J1,
                         &stacker_in_J23);
      AINFO << "adc_in_J1 = " << adc_in_J1 << "    adc_in_J23 = " << adc_in_J23;
      if ((!adc_in_J1 && stacker_in_J1) || (adc_in_J1 && !stacker_in_J1)) {
        AINFO << "adc or stacker no in J1";
        continue;
      }
      if ((!adc_in_J23 && stacker_in_J23) || (adc_in_J23 && !stacker_in_J23)) {
        AINFO << "adc or stacker no in J23";
        continue;
      }
    }

    if (is_need_consider_stacker) {
      AINFO << "need consider stacker " << stacker_id << "   "
            << is_need_consider_stacker;
      if (stacker_sl.start_s() < min_stacker_start_s) {
        min_stacker_start_s = stacker_sl.start_s();
        nearst_stacker_id = stacker_id;
        is_get_consider_stacker = true;
        nearst_tacker_slboundary = stacker_sl;
      }
    }
  }

  AINFO << "nearst stacker id = " << nearst_stacker_id;
  bool is_loc_stacker = false;
  if (is_get_consider_stacker) {
    if (nearst_stacker_id.find("stacker") == std::string::npos) {
      for (auto stacker_boundary : reference_line_info_->StackerBoundaryMap()) {
        if (stacker_boundary.first.empty()) {
          AINFO << "stacker boundary id is empty";
        }
        AINFO << "stacker_boundary.first = " << stacker_boundary.first;
        const auto &perception_stacker_sl = stacker_boundary.second;
        bool is_lon_overlap = !(perception_stacker_sl.start_s() >
                                    nearst_tacker_slboundary.end_s() ||
                                perception_stacker_sl.end_s() <
                                    nearst_tacker_slboundary.start_s());
        bool is_lat_overlap = !(perception_stacker_sl.start_l() >
                                    nearst_tacker_slboundary.end_l() ||
                                perception_stacker_sl.end_l() <
                                    nearst_tacker_slboundary.start_l());
        if (is_lon_overlap && is_lat_overlap) {
          AINFO << "perception stacker sl boundary overlap with loc stacker";
          //  min_stacker_start_s = perception_stacker_sl.start_s();
          nearst_stacker_id = stacker_boundary.first;
          is_loc_stacker = true;
        }
      }
    } else {
      is_loc_stacker = true;
    }
    AINFO << "is loc stacker = " << is_loc_stacker;
    if (is_loc_stacker) {
      MakeNaearstStackerObjectDecision(nearst_stacker_id);
      injector_->stacker_discover_times_ = 0;
    return;
  }
  }
AINFO<<"nearst stacker is perception stacker";
    injector_->target_stacker_info_.second.first = nearst_stacker_id;
  SLBoundary target_tacker_slboundary;
  if (injector_->target_stacker_info_.second.first != "") {
    century::hdmap::Polygon polygon =
        injector_->target_stacker_info_.second.second;
    if (!reference_line.GetSLBoundary(polygon, &target_tacker_slboundary)) {
      // AINFO << "no get sl boundary stacker";
      injector_->target_stacker_info_.second.first.clear();
    }
    } else {
    // injector_->target_stacker_info_.second.first.clear();
     AINFO << "no stacker id in injector";
  }

  // AINFO << "has pass stacker response = "
  //       << injector_->pass_stacker_response().has_response();
  bool has_pass_stacker_response =
      (injector_->pass_stacker_response().pass_stacker_response_type() ==
       planning::PassStackerResponseType::PASS);
  AINFO << "has_pass_stacker_response = " << has_pass_stacker_response;
  AINFO << " response stacker id = "
        << injector_->pass_stacker_response().stacker_id();
  // check is loc stacker response
  if (has_pass_stacker_response) {
    if (injector_->pass_stacker_response().stacker_id().find("stacker") !=
        std::string::npos) {
      AINFO << "nearst stacker is perception,but last response is loc stacker";
      planning::PassStackerResponse pass_stacker_response;
      pass_stacker_response.set_pass_stacker_response_type(
          planning::PassStackerResponseType::ORIGINAL);
      pass_stacker_response.set_has_response(false);
      injector_->set_pass_stacker_response(pass_stacker_response);
      injector_->target_stacker_info_.second.first.clear();
      has_pass_stacker_response = false;
    }
  }

  if (!has_pass_stacker_response) {
    AINFO << "no pass stacker response,need request pass";
    if (!is_get_consider_stacker) {
      injector_->stacker_discover_times_ = 0;
      AINFO << " no get need consider stacker,now";
      // return;
      if (injector_->target_stacker_info_.first) {
        injector_->target_stacker_check_times_++;
        injector_->target_stacker_check_times_ =
            std::min(4, injector_->target_stacker_check_times_);
      }
      AINFO << "target_stacker_check_times_ = "
            << injector_->target_stacker_check_times_;
      if (injector_->target_stacker_check_times_ > kStackerDisappear) {
        injector_->stacker_discover_times_ = 0;
        AINFO << "clear target stacker info,no need consider";
        injector_->target_stacker_info_.first = false;
        injector_->target_stacker_info_.second.first = "";
        injector_->target_stacker_check_times_ = 0;
        return;
      }
    } else {
      AINFO << "DICOVER TACKER";
      AINFO << "injector_->target_stacker_info_.first = "
            << injector_->target_stacker_info_.first;
      AINFO << "injector_->stacker_discover_times_  = "
            << injector_->stacker_discover_times_;
      // if (injector_->stacker_discover_times_ < kStackerDiacoverCount &&
      //     !injector_->target_stacker_info_.first) {
      if (injector_->stacker_discover_times_ < kStackerDiacoverCount) {
        injector_->stacker_discover_times_++;
        AINFO << "RETURN";
        return;
      }
      // injector_->stacker_discover_times_ = 0;

      // AINFO << " current frame get need consider stacker";
      // AINFO << "nearst_stacker_id = " << nearst_stacker_id;
      auto *find_obs =
          reference_line_info->path_decision()->Find(nearst_stacker_id);
      if (find_obs) {
        century::hdmap::Polygon polygon;
        for (const auto &point : find_obs->PerceptionPolygon().points()) {
          century::common::PointENU *hdmap_point = polygon.add_point();
          hdmap_point->set_x(point.x());
          hdmap_point->set_y(point.y());
        }
        injector_->target_stacker_info_ =
            std::make_pair(true, std::make_pair(nearst_stacker_id, polygon));
        if (!reference_line.GetSLBoundary(polygon, &target_tacker_slboundary)) {
          // AINFO << "no update sl boundary stacker";
        }
      }
      const auto &nearst_stacker_sl_boundary = target_tacker_slboundary;
      min_obs_start_s = nearst_stacker_sl_boundary.start_s();
    }
      } else {
     AINFO << "has pass stacker response,do not need request";
    injector_->target_stacker_info_.first = false;
    injector_->target_stacker_check_times_ = 0;  // id can't clear

    if (injector_->target_stacker_info_.second.first != "") {
      AINFO<<"is_get_consider_stacker = "<<is_get_consider_stacker;
      if (is_get_consider_stacker) {
        auto *find_obs =
            reference_line_info->path_decision()->Find(nearst_stacker_id);
        if (find_obs) {
          century::hdmap::Polygon polygon;
          for (const auto &point : find_obs->PerceptionPolygon().points()) {
            century::common::PointENU *hdmap_point = polygon.add_point();
            hdmap_point->set_x(point.x());
            hdmap_point->set_y(point.y());
          }
          injector_->target_stacker_info_.second.first = nearst_stacker_id;
          injector_->target_stacker_info_.second.second = polygon;
          // no update
          // if (!reference_line.GetSLBoundary(polygon,
          //                                   &target_tacker_slboundary)) {
          //   AINFO << "no get sl boundary stacker";
          // }
        } else {
          AINFO << "no find obs,use last frame";
        }
        injector_->stacker_disappear_times_ = 0;
      } else {
        injector_->stacker_disappear_times_++;
        // AINFO << "no find obs,use last frame";
        if (injector_->stacker_change_times_ > 10) {
          // AINFO<<"stacker disappear large 5 frame";
          AINFO << "adc passed stacker,clear response.";
        planning::PassStackerResponse pass_stacker_response;
        pass_stacker_response.set_pass_stacker_response_type(
              planning::PassStackerResponseType::ORIGINAL);
          pass_stacker_response.set_has_response(false);
        injector_->set_pass_stacker_response(pass_stacker_response);
          injector_->target_stacker_info_.second.first.clear();
        }
        // AINFO<<"injector_->stacker_disappear_times_ =
        // "<<injector_->stacker_disappear_times_;
      }
    } else {
      // AINFO << "no get target stacker id,do nothing";
    }
    if (injector_->target_stacker_info_.second.first == "") {
      planning::PassStackerResponse pass_stacker_response;
      pass_stacker_response.set_pass_stacker_response_type(
          planning::PassStackerResponseType::ORIGINAL);
      pass_stacker_response.set_has_response(false);
      injector_->set_pass_stacker_response(pass_stacker_response);
      AINFO << "target_stacker_info_.second.first is empty,do nothing";
      return;
    }
    const auto &nearst_stacker_sl_boundary = target_tacker_slboundary;
    // min_obs_start_s = nearst_stacker_sl_boundary.start_s();
    // AINFO << "nearst_stacker_sl_boundary = "
    //       << nearst_stacker_sl_boundary.end_s();
      if (adc_start_s - FLAGS_distance_borrow_return <
          nearst_stacker_sl_boundary.end_s()) {
        AINFO << "adc is passing stacker now";
        PassStackerRequest notice_stacker;
      notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
        notice_stacker.set_stacker_id(nearst_stacker_id);
        notice_stacker.set_request_for_pass_stacker(true);
      notice_stacker.set_request_type(planning::PassStackerRequestType::PASSING);
        reference_line_info_->SetNeedNoticeStacker(notice_stacker);
        return;
      }
      AINFO << "adc passed stacker,clear response.";
      planning::PassStackerResponse pass_stacker_response;
      pass_stacker_response.set_pass_stacker_response_type(
          planning::PassStackerResponseType::ORIGINAL);
      pass_stacker_response.set_has_response(false);
      injector_->set_pass_stacker_response(pass_stacker_response);
  }
  double soft_stop_wall_s = min_obs_start_s - FLAGS_pass_stacker_stop_distance;
  // if ori no obs ,no build stop wall.
  if (soft_stop_wall_s >
      adc_end_s +
          reference_line_info_->path_data().discretized_path().Length()) {
    return;
  }
  soft_stop_wall_s = std::max(adc_end_s, soft_stop_wall_s);
  if (CheckStartPonitorEndPointNearStacker(frame_, reference_line_info_)) {
    return;
  }

  const std::string stop_wall_id = "STOP_REASON_STACKER";
  std::vector<std::string> wait_for_obstacles;
  // AINFO<<"BUILD STOP WALL FOR STACKER";
  util::BuildStopDecision(
      stop_wall_id, soft_stop_wall_s, 0.0, StopReasonCode::STOP_REASON_STACKER,
      wait_for_obstacles, "STOP_REASON_STACKER", frame_, reference_line_info);
  PassStackerRequest pass_stacker_request;
  pass_stacker_request.mutable_header()->set_timestamp_sec(
      Clock::NowInSeconds());
  pass_stacker_request.set_stacker_id(
      injector_->target_stacker_info_.second.first);
  pass_stacker_request.set_request_for_pass_stacker(true);
  reference_line_info_->SetPassStackerRequest(pass_stacker_request);
  if (std::fabs(reference_line_info_->vehicle_state().linear_velocity()) <
      0.01) {
    injector_->pass_stacker_count_++;
  }

  // need stop ,and count
  if (injector_->pass_stacker_count_ > FLAGS_pass_stacker_wait_times) {
    injector_->pass_stacker_count_ = 0;
    planning::PassStackerResponse pass_stacker_response;
    pass_stacker_response.set_pass_stacker_response_type(
        planning::PassStackerResponseType::PASS);
    pass_stacker_response.set_has_response(true);
    injector_->set_pass_stacker_response(pass_stacker_response);
  }
}
void PathDecider::MakeStackerObjectDecisionUsePerception(
    const PathData &path_data, ReferenceLineInfo *reference_line_info,
    PathDecision *const path_decision) {
  double adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  const auto &adc_sl = reference_line_info->AdcSlBoundary();
  double min_obs_start_s = std::numeric_limits<double>::max();
  std::string stacker_id = "";
  double min_stacker_start_s = std::numeric_limits<double>::max();
  std::string nearst_stacker_id = "";
  bool is_get_consider_stacker = false;
  const ReferenceLine &reference_line = reference_line_info->reference_line();
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    if (obstacle->Perception().type() !=
            perception::PerceptionObstacle::STACKER &&
        obstacle->Perception().type() !=
            perception::PerceptionObstacle::FORKLIFT_STACKER) {
      continue;
    }
    stacker_id = obstacle->Id();
    const auto &stacker_sl = obstacle->PerceptionSLBoundary();
    if (stacker_sl.start_s() - adc_end_s > kStartConsiderStackerDistance) {
      AINFO << "large distance ,no consider";
      continue;
    }
    if (stacker_sl.end_s() + FLAGS_distance_borrow_return < adc_start_s) {
      AINFO << "back stacker ,no consider";
      continue;
    }
    double consider_start_l = adc_sl.start_l() - kStackerConsiderDistance;
    double consider_end_l = adc_sl.end_l() + kStackerConsiderDistance;
    bool is_need_consider_stacker = !(stacker_sl.start_l() > consider_end_l ||
                                      consider_start_l > stacker_sl.end_l());
    if (is_need_consider_stacker) {
      if (stacker_sl.start_s() < min_stacker_start_s) {
        min_stacker_start_s = stacker_sl.start_s();
        nearst_stacker_id = stacker_id;
        is_get_consider_stacker = true;
      }
    }
  }

  SLBoundary target_tacker_slboundary;
  if (injector_->target_stacker_info_.second.first != "") {
    century::hdmap::Polygon polygon =
        injector_->target_stacker_info_.second.second;
    if (!reference_line.GetSLBoundary(polygon, &target_tacker_slboundary)) {
      injector_->target_stacker_info_.second.first.clear();
    }
  }
  bool has_pass_stacker_response =
      (injector_->pass_stacker_response().pass_stacker_response_type() ==
       planning::PassStackerResponseType::PASS);

  if (!has_pass_stacker_response) {
    AINFO << "no pass stacker response,need request pass";
    if (!is_get_consider_stacker) {
      injector_->stacker_discover_times_ = 0;
      AINFO << " no get need consider stacker,now";
      if (injector_->target_stacker_info_.first) {
        injector_->target_stacker_check_times_++;
        injector_->target_stacker_check_times_ =
            std::min(4, injector_->target_stacker_check_times_);
      }
      AINFO << "target_stacker_check_times_ = "
            << injector_->target_stacker_check_times_;
      if (injector_->target_stacker_check_times_ > kStackerDisappear) {
        injector_->stacker_discover_times_ = 0;
        AINFO << "clear target stacker info,no need consider";
        injector_->target_stacker_info_.first = false;
        injector_->target_stacker_info_.second.first = "";
        injector_->target_stacker_check_times_ = 0;
        return;
      }
    } else {
      if (injector_->stacker_discover_times_ < kStackerDiacoverCount &&
          !injector_->target_stacker_info_.first) {
        injector_->stacker_discover_times_++;
        return;
      }
      injector_->stacker_discover_times_ = 0;
      auto *find_obs =
          reference_line_info->path_decision()->Find(nearst_stacker_id);
      if (find_obs) {
        century::hdmap::Polygon polygon;
        for (const auto &point : find_obs->PerceptionPolygon().points()) {
          century::common::PointENU *hdmap_point = polygon.add_point();
          hdmap_point->set_x(point.x());
          hdmap_point->set_y(point.y());
        }
        injector_->target_stacker_info_ =
            std::make_pair(true, std::make_pair(nearst_stacker_id, polygon));
        if (!reference_line.GetSLBoundary(polygon, &target_tacker_slboundary)) {
        }
      }
      const auto &nearst_stacker_sl_boundary = target_tacker_slboundary;
      min_obs_start_s = nearst_stacker_sl_boundary.start_s();
    }
  } else {
    injector_->target_stacker_info_.first = false;
    injector_->target_stacker_check_times_ = 0;
    if (injector_->target_stacker_info_.second.first != "") {
      if (is_get_consider_stacker) {
        auto *find_obs =
            reference_line_info->path_decision()->Find(nearst_stacker_id);
        if (find_obs) {
          century::hdmap::Polygon polygon;
          for (const auto &point : find_obs->PerceptionPolygon().points()) {
            century::common::PointENU *hdmap_point = polygon.add_point();
            hdmap_point->set_x(point.x());
            hdmap_point->set_y(point.y());
          }
          injector_->target_stacker_info_.second.first = nearst_stacker_id;
          injector_->target_stacker_info_.second.second = polygon;
        }
        injector_->stacker_disappear_times_ = 0;
      } else {
        injector_->stacker_disappear_times_++;
        if (injector_->stacker_change_times_ > 10) {
          planning::PassStackerResponse pass_stacker_response;
          pass_stacker_response.set_pass_stacker_response_type(
              planning::PassStackerResponseType::ORIGINAL);
          pass_stacker_response.set_has_response(false);
          injector_->set_pass_stacker_response(pass_stacker_response);
          injector_->target_stacker_info_.second.first.clear();
        }
      }
    }
    if (injector_->target_stacker_info_.second.first == "") {
      return;
    }
    const auto &nearst_stacker_sl_boundary = target_tacker_slboundary;
    if (adc_start_s - FLAGS_distance_borrow_return <
        nearst_stacker_sl_boundary.end_s()) {
      AINFO << "adc is passing stacker now";
      PassStackerRequest notice_stacker;
      notice_stacker.mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
      notice_stacker.set_stacker_id(nearst_stacker_id);
      notice_stacker.set_request_for_pass_stacker(true);
      reference_line_info_->SetNeedNoticeStacker(notice_stacker);
      return;
    }
    AINFO << "adc passed stacker,clear response.";
    planning::PassStackerResponse pass_stacker_response;
    pass_stacker_response.set_pass_stacker_response_type(
        planning::PassStackerResponseType::ORIGINAL);
    pass_stacker_response.set_has_response(false);
    injector_->set_pass_stacker_response(pass_stacker_response);
  }
  double soft_stop_wall_s = min_obs_start_s - FLAGS_pass_stacker_stop_distance;
  // if ori no obs ,no build stop wall.
  if (soft_stop_wall_s >
      adc_end_s +
          reference_line_info_->path_data().discretized_path().Length()) {
    return;
  }
    soft_stop_wall_s = std::max(adc_end_s, soft_stop_wall_s);
  if (CheckStartPonitorEndPointNearStacker(frame_, reference_line_info_)) {
    return;
  }

  const std::string stop_wall_id = "STOP_REASON_STACKER";
  std::vector<std::string> wait_for_obstacles;
  // AINFO<<"BUILD STOP WALL FOR STACKER";
  util::BuildStopDecision(
      stop_wall_id, soft_stop_wall_s, 0.0, StopReasonCode::STOP_REASON_STACKER,
      wait_for_obstacles, "STOP_REASON_STACKER", frame_, reference_line_info);
  PassStackerRequest pass_stacker_request;
  pass_stacker_request.mutable_header()->set_timestamp_sec(
      Clock::NowInSeconds());
  pass_stacker_request.set_stacker_id(
      injector_->target_stacker_info_.second.first);
  pass_stacker_request.set_request_for_pass_stacker(true);
  reference_line_info_->SetPassStackerRequest(pass_stacker_request);
  if (std::fabs(reference_line_info_->vehicle_state().linear_velocity()) <
      0.01) {
    injector_->pass_stacker_count_++;
  }

  // need stop ,and count
  if (injector_->pass_stacker_count_ > FLAGS_pass_stacker_wait_times) {
    injector_->pass_stacker_count_ = 0;
  }
}
void PathDecider::MakeFallbackPathDecision(
    const PathData &path_data, ReferenceLineInfo *reference_line_info) {
  if (std::string::npos == path_data.path_label().find("fallback")) {
    return;
  }

  bool adc_in_ref_lane = reference_line_info->IsAdcOnReferenceLine();
  // TODO: Open after completing the route extension (yjl / gzx)
  // bool adc_in_routing_lane =
  //    reference_line_info->AdcIsOnAllRouteLane(injector_->planning_context());
  bool adc_in_routing_lane = true;
  bool fallback_in_play_street = injector_->is_in_play_street &&
                                 !reference_line_info->IsAdcInCommonJunction();
  if (!adc_in_ref_lane && !adc_in_routing_lane && !fallback_in_play_street) {
    double adc_v =
        std::fabs(reference_line_info->vehicle_state().linear_velocity());
    double brake_distance = std::fabs(
        adc_v * adc_v / FLAGS_soft_deceleration_for_lane_change_stop * 0.5);
    brake_distance = adc_v > config_.path_decider_config()
                                 .out_lane_stop_immediately_velocity_threshold()
                         ? brake_distance
                         : 0.0;
    double adc_end_s = reference_line_info->AdcSlBoundary().end_s();
    double soft_stop_wall_s = adc_end_s + brake_distance;

    const std::string stop_wall_id = "out of route lane wall";
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(stop_wall_id, soft_stop_wall_s, 0.0,
                            StopReasonCode::STOP_REASON_SIDEPASS_SAFETY,
                            wait_for_obstacles, "OutOfRouteLaneStop", frame_,
                            reference_line_info);
    AINFO << "Stop due to adc out of routing lane at s = " << adc_end_s;
  }
}

bool PathDecider::CheckStartPonitorEndPointNearStackerForStacker(
    const std::string &stacker_id, Frame *frame,
    ReferenceLineInfo *const reference_line_info) {
  const auto& routing = frame->local_view().routing;
  auto stacker_id_com = stacker_id;
  if (!stacker_id_com.empty()) {
    std::string prefix = "stacker_";
    if (stacker_id_com.find("stacker_") != std::string::npos) {
      stacker_id_com = stacker_id_com.erase(0, prefix.length());
    }
    bool is_stacker_operating =
        (routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_LEFT ||
         routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_RIGHT ||
         routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_FRONT ||
         routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_BACK ||
         routing->routing_request().task_type() ==
             routing::YARD_OPERATIONAREA_DYNAMIC ||
         routing->routing_request().task_type() ==
             routing::RAILWAY_OPERATIONAREA_DYNAMIC) &&
        (stacker_id_com == routing->routing_request().operation_stacker_id()) &&
        !routing->routing_request().operation_stacker_id().empty();
    if (is_stacker_operating) {
      AINFO << "stacker operating";
      return true;
    }
  }

  auto *find_obs = reference_line_info_->path_decision()->Find(stacker_id);
  if (find_obs == nullptr) {
    return false;
  }
  common::SLPoint destination_sl;
  const auto &reference_line = reference_line_info->reference_line();
  // bool is_stacker_call_adc = false;
  const auto &routing_request_start_point =
      routing->routing_request().waypoint().at(0);
  const auto &routing_request_end_point =
      routing->routing_request().waypoint().at(
          routing->routing_request().waypoint().size() - 1);
  common::SLPoint start_sl;
  common::SLPoint end_sl;
  bool is_start_point_near = false;
  bool is_end_point_near = false;
  // if (injector_->target_stacker_info_.second.first == "") {
  //   AERROR << "target_stacker_info_.second.first is empty,do nothing";
  //   return false;
  // }
  SLBoundary target_stacker_sl;
  double distance_to_target_point_in_lon = kDistanceToTargetPointInLon;
  double distance_to_target_point_in_lateral = kDistanceToTargetPointInLateral;
  auto routing_requet = routing->routing_request();
  bool is_static_wating_request =
      routing_requet.task_type() == routing::YARD_WAITINGAREA_STATIC;
  target_stacker_sl = find_obs->PerceptionSLBoundary();
  if (reference_line.XYToSL(routing_request_start_point.pose(), &start_sl)) {
    is_start_point_near =
        (start_sl.s() >
             target_stacker_sl.start_s() - distance_to_target_point_in_lon &&
         start_sl.s() <
             target_stacker_sl.end_s() + distance_to_target_point_in_lon) &&
        (std::fabs(start_sl.l() - target_stacker_sl.start_l()) <
             distance_to_target_point_in_lateral ||
         std::fabs(start_sl.l() - target_stacker_sl.end_l()) <
             distance_to_target_point_in_lateral);
  }
  if (reference_line.XYToSL(routing_request_end_point.pose(), &end_sl)) {
    is_end_point_near = (end_sl.s() > target_stacker_sl.start_s() -
                                          distance_to_target_point_in_lon &&
                         end_sl.s() < target_stacker_sl.end_s() +
                                          distance_to_target_point_in_lon) &&
                        (std::fabs(end_sl.l() - target_stacker_sl.start_l()) <
                             distance_to_target_point_in_lateral ||
                         std::fabs(end_sl.l() - target_stacker_sl.end_l()) <
                             distance_to_target_point_in_lateral);
  }
  if (is_static_wating_request) {
    is_end_point_near = (end_sl.s() > target_stacker_sl.start_s() -
                                          FLAGS_change_routing_end_distance &&
                         end_sl.s() < target_stacker_sl.end_s() +
                                          distance_to_target_point_in_lon) &&
                        (std::fabs(end_sl.l() - target_stacker_sl.start_l()) <
                             distance_to_target_point_in_lateral ||
                         std::fabs(end_sl.l() - target_stacker_sl.end_l()) <
                             distance_to_target_point_in_lateral);
  }
  if (is_start_point_near || is_end_point_near) {
    return true;
  }

  return false;
}
bool PathDecider::CheckStartPonitorEndPointNearStacker(
    Frame *frame, ReferenceLineInfo *const reference_line_info) {
  auto stacker_id = injector_->target_stacker_info_.second.first;
      const auto& routing = frame->local_view().routing;
  if (!stacker_id.empty()) {
    std::string prefix = "stacker_";
    if (stacker_id.find("stacker_") != std::string::npos) {
      stacker_id = stacker_id.erase(0, prefix.length());
    }
    bool is_stacker_operating =
        (routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_LEFT ||
         routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_RIGHT ||
         routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_FRONT ||
         routing->routing_request().task_type() ==
             routing::TINY_ADJUSTMENT_BACK ||
         routing->routing_request().task_type() ==
             routing::YARD_OPERATIONAREA_DYNAMIC ||
         routing->routing_request().task_type() ==
             routing::RAILWAY_OPERATIONAREA_DYNAMIC) &&
        (injector_->target_stacker_info_.second.first ==
         routing->routing_request().operation_stacker_id()) &&
        (!injector_->target_stacker_info_.second.first.empty() &&
         !routing->routing_request().operation_stacker_id().empty());
    if (is_stacker_operating) {
      AINFO << "stacker operating";
      return true;
    }
  }
  common::SLPoint destination_sl;
  const auto &reference_line = reference_line_info->reference_line();
  // bool is_stacker_call_adc = false;
  const auto &routing_request_start_point =
      routing->routing_request().waypoint().at(0);
  const auto &routing_request_end_point =
      routing->routing_request().waypoint().at(
          routing->routing_request().waypoint().size() - 1);
  common::SLPoint start_sl;
  common::SLPoint end_sl;
  bool is_start_point_near = false;
  bool is_end_point_near = false;
  if (injector_->target_stacker_info_.second.first == "") {
    AERROR << "target_stacker_info_.second.first is empty,do nothing";
    return false;
  }
  SLBoundary target_stacker_sl;
  const auto &consider_stacker_polygon =
      injector_->target_stacker_info_.second.second;
  injector_->target_stacker_info_.second.second;
  if (!reference_line.GetSLBoundary(consider_stacker_polygon,
                                    &target_stacker_sl)) {
    AERROR << "no get target stacker sl";
  }
  double distance_to_target_point_in_lon = kDistanceToTargetPointInLon;
  double distance_to_target_point_in_lateral = kDistanceToTargetPointInLateral;
  auto routing_requet = routing->routing_request();
  bool is_static_wating_request =
      routing_requet.task_type() == routing::YARD_WAITINGAREA_STATIC;
  if (reference_line.XYToSL(routing_request_start_point.pose(), &start_sl)) {
    is_start_point_near =
        (start_sl.s() >
             target_stacker_sl.start_s() - distance_to_target_point_in_lon &&
         start_sl.s() <
             target_stacker_sl.end_s() + distance_to_target_point_in_lon) &&
        (std::fabs(start_sl.l() - target_stacker_sl.start_l()) <
             distance_to_target_point_in_lateral ||
         std::fabs(start_sl.l() - target_stacker_sl.end_l()) <
             distance_to_target_point_in_lateral);
  }
  if (reference_line.XYToSL(routing_request_end_point.pose(), &end_sl)) {
    is_end_point_near = (end_sl.s() > target_stacker_sl.start_s() -
                                          distance_to_target_point_in_lon &&
                         end_sl.s() < target_stacker_sl.end_s() +
                                          distance_to_target_point_in_lon) &&
                        (std::fabs(end_sl.l() - target_stacker_sl.start_l()) <
                             distance_to_target_point_in_lateral ||
                         std::fabs(end_sl.l() - target_stacker_sl.end_l()) <
                             distance_to_target_point_in_lateral);
    if (is_static_wating_request) {
      is_end_point_near =
          (end_sl.s() >
               target_stacker_sl.start_s() -FLAGS_change_routing_end_distance  &&
           end_sl.s() <
               target_stacker_sl.end_s() + distance_to_target_point_in_lon) &&
          (std::fabs(end_sl.l() - target_stacker_sl.start_l()) <
               distance_to_target_point_in_lateral ||
           std::fabs(end_sl.l() - target_stacker_sl.end_l()) <
               distance_to_target_point_in_lateral);
    }
  }
  if (is_start_point_near || is_end_point_near) {
    return true;
  }

  return false;
}
void PathDecider::CheckHeadbangPath(
    Frame *frame, ReferenceLineInfo *const reference_line_info) {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  double max_kappa_diff = 0.0;  // only view in cyber_monitor
  double current_max_kappa = 0.0;
  double current_max_centripetal_acceleration = 0.0;
  GetKappaAndCentripetalAcceleration(reference_line_info, &max_kappa_diff,
                                     &current_max_kappa,
                                     &current_max_centripetal_acceleration);

  // Get previous max kappa and max centripetal acceleration
  auto *mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();

  double previous_max_kappa = mutable_path_decider_status->previous_max_kappa();
  double previous_max_centripetal_acceleration =
      mutable_path_decider_status->previous_max_centripetal_acceleration();

  double current_max_kappa_diff = current_max_kappa - previous_max_kappa;
  double current_max_centripetal_acceleration_diff =
      current_max_centripetal_acceleration -
      previous_max_centripetal_acceleration;

  if (adc_speed > config_.path_decider_config().min_adc_speed() &&
      std::abs(current_max_centripetal_acceleration_diff) >
          config_.path_decider_config()
              .max_centripetal_acceleration_threshold()) {
    // trim previous path data
    AWARN << "The current path is a head shaking path, using the previous "
             "path!";
    TrimLastPath(frame, reference_line_info);

    // Trimmed path need to  recalculate max kappa & max centripetal
    // acceleration
    double trim_max_kappa_diff = 0.0;
    double trim_max_kappa = 0.0;
    double trim_max_centripetal_acceleration = 0.0;
    GetKappaAndCentripetalAcceleration(reference_line_info,
                                       &trim_max_kappa_diff, &trim_max_kappa,
                                       &trim_max_centripetal_acceleration);

    // set previous info
    mutable_path_decider_status->set_previous_max_kappa(trim_max_kappa);
    mutable_path_decider_status->set_previous_max_centripetal_acceleration(
        trim_max_centripetal_acceleration);

    // add debug info
    reference_line_info->mutable_debug()
        ->mutable_planning_data()
        ->mutable_path_decider()
        ->set_is_headbang_path(true);
  } else {
    // add debug info
    reference_line_info->mutable_debug()
        ->mutable_planning_data()
        ->mutable_path_decider()
        ->set_is_headbang_path(false);

    // set previous info, if headbang path need to recalculate
    mutable_path_decider_status->set_previous_max_kappa(current_max_kappa);
    mutable_path_decider_status->set_previous_max_centripetal_acceleration(
        current_max_centripetal_acceleration);
  }

  // add debug info
  planning_internal::PathDeciderDebug *path_decider_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_path_decider();
  path_decider_debug->set_reference_line_path_max_kappa_diff(max_kappa_diff);
  path_decider_debug->set_previous_max_kappa(previous_max_kappa);
  path_decider_debug->set_previous_max_centripetal_acceleration(
      previous_max_centripetal_acceleration);
  path_decider_debug->set_current_max_kappa(current_max_kappa);
  path_decider_debug->set_current_max_centripetal_acceleration(
      current_max_centripetal_acceleration);
  path_decider_debug->set_adc_speed(adc_speed);
  path_decider_debug->set_max_kappa_diff(current_max_kappa_diff);
  path_decider_debug->set_max_centripetal_acceleration_diff(
      current_max_centripetal_acceleration_diff);
}

void PathDecider::TrimLastPath(Frame *frame,
                               ReferenceLineInfo *const reference_line_info) {
  const ReferenceLine &reference_line = reference_line_info->reference_line();

  const common::TrajectoryPoint &planning_start_point =
      frame->PlanningStartPoint();
  const common::PathPoint &init_path_point = planning_start_point.path_point();
  ADEBUG << "init_path_point x:[" << std::setprecision(9) << init_path_point.x()
         << "], y[" << init_path_point.y() << "], s: [" << init_path_point.s()
         << "]";

  const DiscretizedPath &last_path =
      injector_->last_path_data().discretized_path();
  if (last_path.empty()) {
    AERROR << "last path data is empty, can not trim path.";
    return;
  }

  size_t path_start_index = 0;
  for (size_t i = 0; i < last_path.size(); ++i) {
    // find previous init point
    if (last_path[i].s() > 0) {
      path_start_index = i;
      break;
    }
  }

  // get current s=0
  common::SLPoint init_path_position_sl;
  reference_line.XYToSL(init_path_point, &init_path_position_sl);
  ADEBUG << "init_path_position_sl.s(): " << init_path_position_sl.s();

  DiscretizedPath trimmed_path;
  bool inserted_init_point = false;
  for (size_t i = path_start_index; i < last_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {last_path[i].x(), last_path[i].y()};

    reference_line.XYToSL(path_position, &path_position_sl);

    double updated_s = path_position_sl.s() - init_path_position_sl.s();
    // insert init point
    if (updated_s > 0 && !inserted_init_point) {
      trimmed_path.emplace_back(init_path_point);
      trimmed_path.back().set_s(0);
      inserted_init_point = true;
    }

    trimmed_path.emplace_back(last_path[i]);
    trimmed_path.back().set_s(updated_s);
  }

  // set path
  auto *path_data = reference_line_info->mutable_path_data();
  ADEBUG << "previous path_data size: " << last_path.size();
  path_data->SetReferenceLine(&reference_line);
  path_data->SetDiscretizedPath(std::move(trimmed_path));
  ADEBUG << "current path_data size: " << trimmed_path.size();
}

void PathDecider::GetKappaAndCentripetalAcceleration(
    ReferenceLineInfo *const reference_line_info, double *max_kappa_diff,
    double *max_kappa, double *max_centripetal_acceleration) {
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  for (auto &path_point : reference_line_info->path_data().discretized_path()) {
    if (path_point.s() >
        config_.path_decider_config().path_distance_threshold()) {
      break;
    }

    const auto &reference_point =
        reference_line_info->reference_line().GetNearestReferencePoint(
            common::math::Vec2d(path_point.x(), path_point.y()));
    double kappa_diff = std::abs(path_point.kappa() - reference_point.kappa());
    if (std::abs(kappa_diff) > *max_kappa_diff) {
      *max_kappa_diff = kappa_diff;
    }

    // Get current max kappa
    if (std::abs(path_point.kappa()) > std::abs(*max_kappa)) {
      *max_kappa = path_point.kappa();
    }

    // Get current max centripetal acceleration
    double centripetal_acceleration =
        std::fabs(path_point.kappa()) * std::pow(adc_speed, 2);
    if (centripetal_acceleration > *max_centripetal_acceleration) {
      *max_centripetal_acceleration = centripetal_acceleration;
    }
  }
}

double PathDecider::GetStopDistance() {
  double stop_distance = FLAGS_max_stop_distance_obstacle;
  if (injector_->near_traffic_line_) {
    stop_distance = config_.path_decider_config()
                        .min_distance_to_obstacle_near_traffic_light();
  }
  if (FLAGS_enable_use_radical_decision &&
      injector_->is_can_enter_mixed_flow_) {
    stop_distance = injector_->vehicle_state()->linear_velocity() *
                        injector_->vehicle_state()->linear_velocity() * 0.5 /
                        kComfortableAcc +
                    kBreakBuff;
    ADEBUG << "stop_distance = " << stop_distance;
  }
  return stop_distance;
}

double PathDecider::GetObstacleStopDistance(const Obstacle &obstacle,
                                            const double stop_distance) {
  const auto &vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double half_width = vehicle_param.width() * 0.5;
  const auto &sl_boundary = obstacle.PerceptionSLBoundary();
  double stop_distance_modify = stop_distance;
  double obs_s = 0.5 * (sl_boundary.start_s() + sl_boundary.end_s());
  if (reference_line_info_->IsObstacleInGateArea(obstacle) &&
      reference_line_info_->IsInPlayStreet(obs_s) &&
      injector_->vehicle_state()->linear_velocity() <
          FLAGS_lower_speed_in_public_road) {
    stop_distance_modify = FLAGS_stop_distance_in_common_junction;
  }
  stop_distance_modify =
      std::min(obstacle.MinRadiusStopDistance(vehicle_param, half_width),
               stop_distance_modify);
  // change stop distance for obs

  const auto &routing = frame_->local_view().routing;
  const auto &routing_end = *(routing->routing_request().waypoint().rbegin());

  common::SLPoint dest_sl;

  reference_line_info_->reference_line().XYToSL(routing_end.pose(), &dest_sl);

  double half_lane_width =
      (reference_line_info_->GetLaneWidthByS(dest_sl.s()).first +
       reference_line_info_->GetLaneWidthByS(dest_sl.s()).second) *
      0.5;

  stop_distance_modify = FLAGS_stop_distance_to_obstacle;
  if (obstacle.IsIgv()) {
    stop_distance_modify = FLAGS_distance_to_igv;
  }
  auto path_data = reference_line_info_->path_data();
  bool is_D_zone_to_yuan2 = IsDZoneToYuan2(path_data);

  if (is_D_zone_to_yuan2) {
    stop_distance_modify = FLAGS_license_plate_recognition_distance;
  }

  if (obstacle.Perception().type() == perception::PerceptionObstacle::STACKER ||
      obstacle.Perception().type() ==
          perception::PerceptionObstacle::FORKLIFT_STACKER) {
    stop_distance_modify = FLAGS_stop_distance_to_stacker;
  }
  double center_s = (reference_line_info_->AdcSlBoundary().end_s() +
                     reference_line_info_->AdcSlBoundary().start_s()) *
                    0.5;
  if (dest_sl.s() < sl_boundary.start_s() && center_s < dest_sl.s() &&
      std::fabs(dest_sl.l()) < half_lane_width) {
    // AINFO << "USE DESTINATION STOP DISTANCE";
    stop_distance_modify = FLAGS_stop_distance_to_obstacle_far;
  }
  if (injector_->is_adc_in_gate_junction_ &&
      obstacle.Perception().type() == perception::PerceptionObstacle::UNKNOWN) {
    stop_distance_modify = FLAGS_weighing_stop_distance;
  }
  return stop_distance_modify;
}

bool PathDecider::IsNeedToSkipDecisionMaking(
    const PathData &path_data, const std::string &blocking_obstacle_id,
    const Obstacle &obstacle, const double stop_distance,
    PathDecision *const path_decision) {
  if (!obstacle.IsStatic() || obstacle.IsVirtual()) {
    return true;
  }
  // - skip decision making for obstacles with IGNORE/STOP decisions already.
  if (obstacle.HasLongitudinalDecision() &&
      obstacle.LongitudinalDecision().has_ignore() &&
      obstacle.HasLateralDecision() &&
      obstacle.LateralDecision().has_ignore()) {
    return true;
  }
  if (obstacle.HasLongitudinalDecision() &&
      obstacle.LongitudinalDecision().has_stop()) {
    // STOP decision
    return true;
  }
  // - add STOP decision for blocking obstacles.
  if (obstacle.Id() == blocking_obstacle_id &&
      !injector_->planning_context()
           ->planning_status()
           .path_decider()
           .is_in_path_lane_borrow_scenario()) {
    // Add stop decision
    AWARN << "Blocking obstacle = " << blocking_obstacle_id;
    const std::string stop_tag = "PathDecider/blocking_obstacle";
    ObjectDecisionType object_decision;
    // util::GenerateObjectStopDecision(
    //     GetObstacleNearestPointS(path_data, obstacle),
    //     -GetObstacleStopDistance(obstacle, stop_distance), stop_tag,
    //     *reference_line_info_, object_decision.mutable_stop());
        util::GenerateObjectStopDecision(
        obstacle, -GetObstacleStopDistance(obstacle, stop_distance), stop_tag,
        *reference_line_info_, object_decision.mutable_stop());
    path_decision->AddLongitudinalDecision(stop_tag, obstacle.Id(),
                                           object_decision);
    return true;
  }
  // - skip decision making for clear-zone obstacles.
  if (STBoundary::BoundaryType::KEEP_CLEAR ==
      obstacle.reference_line_st_boundary().boundary_type()) {
    return true;
  }
  return false;
}

bool PathDecider::IsCollisionWithObstacle(const PathData &path_data,
                                          const Obstacle &obstacle,
                                          const double adc_lateral_safe_buffer,
                                          double *const stop_limit_s) {
  auto adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double center_s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  const auto& reference_point =
      reference_line_info_->reference_line().GetReferencePoint(center_s);
  double heading = reference_line_info_->vehicle_state().heading();
  if (reference_line_info_->NeedDiagonal()) {
    if (reference_line_info_->IsInDiagonalRoad()) {
      heading = reference_line_info_->DiagonalRoadHeading();
      // AINFO<<"diagonal_heading_==="<<diagonal_heading_;
    } else {
        heading = reference_point.heading();
    }
  }
  for (const auto &path_point : path_data.discretized_path()) {
    //  AINFO << "====>" << path_point.theta();
    if (reference_line_info_->NeedDiagonal() &&
        !reference_line_info_->IsInDiagonalRoad()) {
      if (reference_line_info_->FindClosestPointInTurn(path_point.s() +
                                                       center_s)) {
        heading = path_point.theta();
        // AINFO << "path_point.s() = " << path_point.s() << "  in trun lane";
      } else {
        // AINFO << "path_point.s() = " << path_point.s() << "  no in trun lane";
      }
    }
    common::math::Box2d adc_box;
    if (reference_line_info_->NeedDiagonal()) {
      adc_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
          path_point.x(), path_point.y(), heading, 0.0,
          adc_lateral_safe_buffer);
    } else {
      adc_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
          path_point, adc_lateral_safe_buffer);
    }
    if (obstacle.PerceptionPolygon().HasOverlap(
            common::math::Polygon2d(adc_box))) {
      common::SLPoint nearest_point_sl;
      if (reference_line_info_->reference_line().XYToSL(path_point,
                                                        &nearest_point_sl)) {
        *stop_limit_s =
            nearest_point_sl.s() + common::VehicleConfigHelper::GetConfig()
                                       .vehicle_param()
                                       .front_edge_to_center();
      }
      return true;
    }
  }
  return false;
}

bool PathDecider::IsInLaneTurn(const common::PathPoint &path_point) {
  auto adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double center_s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  hdmap::LaneInfoConstPtr locate_lane =
      reference_line_info_->LocateLaneInfo(center_s + path_point.s());
  if (nullptr != locate_lane) {
    common::SLPoint sl;
    sl.set_l(0);
    sl.set_s(center_s + path_point.s());
    common::math::Vec2d xy;
    if (!reference_line_info_->reference_line().SLToXY(sl, &xy)) {
      return false;
    }
    double s_projection = center_s + path_point.s();
    const auto &reference_point =
        reference_line_info_->reference_line().GetNearestReferencePoint(
            s_projection);
    // AINFO<<"reference_point kappa = "<<reference_point.kappa();
    double minddle_kappa = reference_point.kappa();
    if (std::fabs(minddle_kappa) > kMinKappa ||
        locate_lane->lane().turn() != hdmap::Lane::NO_TURN) {
      return true;
    }
  }
  return false;
}

bool PathDecider::IsInDrivableArea(const PathData &path_data,
                                   double &collision_distance,
                                   std::string &msg) {
  if (injector_->IsVehCollisionElectricFence()) {
    return true;
  }
  // Diagonal heading
  double heading = reference_line_info_->vehicle_state().heading();
  auto adc_sl_boundary = reference_line_info_->AdcSlBoundary();
  double center_s = (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) * 0.5;
  if (reference_line_info_->NeedDiagonal()) {
    if (reference_line_info_->IsInDiagonalRoad()) {
      heading = reference_line_info_->DiagonalRoadHeading();
    } else {
      const auto &reference_point =
          reference_line_info_->reference_line().GetReferencePoint(center_s);
      heading = reference_point.heading();
    }
  }
  // routing end
  bool is_endpoint_sl_usable = false;
  const auto routing_end = reference_line_info_->get_routing_end();
  common::SLPoint routing_end_sl;
  if (reference_line_info_->reference_line().XYToSL(routing_end.pose(),
                                                    &routing_end_sl)) {
    is_endpoint_sl_usable = true;
  }
  // path point
  double adc_speed = reference_line_info_->vehicle_state().linear_velocity();
  double brake_distance = adc_speed * adc_speed / (2.0 * kBrakeDeceleration);
  double check_length = std::max(kMinElectricFenceCollisionLength,
      std::min(brake_distance, kMaxElectricFenceCollisionLength));
  int iter = 0;
  bool is_begin_lane_trun = false, is_diagonal_into_ref_lane = false;
  bool is_bypass_obstacle = false;
  int cnt_in_ref_line = 0;
  for (const auto &path_point : path_data.discretized_path()) {
    bool is_pathpoint_sl_usable = false;
    common::SLPoint nearest_point_sl;
    if (reference_line_info_->reference_line().XYToSL(path_point,
                                                      &nearest_point_sl)) {
      is_pathpoint_sl_usable = true;
      if (0 == iter && nearest_point_sl.s() - center_s < kVehLongBuffer) {
        ++iter;
        continue;
      }
      if (nearest_point_sl.s() - center_s > check_length) {
        return true;
      }
      if (!is_diagonal_into_ref_lane && reference_line_info_->NeedDiagonal()) {
        if (is_bypass_obstacle &&
            std::fabs(nearest_point_sl.l()) < kMinLatOffest) {
          cnt_in_ref_line++;
          if (cnt_in_ref_line > kCntInRefLine &&
              nearest_point_sl.s() - center_s >
                  kMinElectricFenceCollisionLength) {
            is_diagonal_into_ref_lane = true;
          }
        }
        if (0 == iter && std::fabs(nearest_point_sl.l()) > kMaxLatOffest) {
          is_bypass_obstacle = false;
        }
      }
    }
    bool is_in_lane_turn = IsInLaneTurn(path_point);
    is_begin_lane_trun =
        is_begin_lane_trun ||
        (is_in_lane_turn && reference_line_info_->NeedDiagonal() &&
         !reference_line_info_->IsInDiagonalRoad());

    if (!reference_line_info_->NeedDiagonal() || is_begin_lane_trun ||
        is_diagonal_into_ref_lane) {
      heading = path_point.theta();
    }
    if (IsCollisionWithElectricFence(path_point.x(), path_point.y(), heading)) {
      if (is_pathpoint_sl_usable && is_endpoint_sl_usable) {
        if (fabs(nearest_point_sl.l()) < kPathPointOffLineLatThreshold &&
            fabs(routing_end_sl.l()) < kPathPointOffLineLatThreshold &&
            nearest_point_sl.s() + kPathPointOffEndPointLongThreshold >
                routing_end_sl.s()) {
          AINFO << "path_point far away routing_end"
                << "->path_point sl:" << nearest_point_sl.l() << ","
                << nearest_point_sl.s()
                << "; routing_end_sl:" << routing_end_sl.l() << ","
                << routing_end_sl.s();
          return true;
        }
      }
      collision_distance =
          nearest_point_sl.s() + common::VehicleConfigHelper::GetConfig()
                                     .vehicle_param()
                                     .front_edge_to_center();
      msg = "path_data point is not within the drivable area";
      DumpCollisionDebug(path_data, path_point, heading, iter,
                         collision_distance);
      return false;
    }
    iter++;
  }
  return true;
}

bool PathDecider::IsPointInJunction(const double &x, const double &y, double search_radius, int junction_type) {
  const hdmap::HDMap *base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  common::PointENU veh_point_enu;
  veh_point_enu.set_x(x);
  veh_point_enu.set_y(y);
  if (0 == base_map_ptr->GetJunctions(veh_point_enu, search_radius, &junctions)) {
    for (const auto &ptr_junction : junctions) {
      if (junction_type == ptr_junction->junction().type()) {
        // AINFO << "YT---------------"
        //       << ptr_junction->junction().Type_Name(
        //              ptr_junction->junction().type());
        return true;
      }
    }
  }
  return false;
}

bool PathDecider::IsCollisionWithElectricFence(const double &x, const double &y,
                                               const double &heading,
                                               bool is_path_point) {
  const hdmap::HDMap *base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<hdmap::ElectricFenceInfoConstPtr> electric_fences;
  std::vector<JunctionInfoConstPtr> junctions;
  bool is_in_drivable_area = false, is_collision = true;
  common::PointENU veh_point_enu;
  veh_point_enu.set_x(x);
  veh_point_enu.set_y(y);
  double LongBuffer = kPathPointLongBuffer;
  double LatBuffer = kPathPointLatBuffer;
  if (!is_path_point) {
    LatBuffer = kVehLatBuffer;
    LongBuffer = kVehLongBuffer;
  }
  // AINFO << "LongBuffer:" << LongBuffer << ",LatBuffer:" << LatBuffer;
  if (0 == base_map_ptr->GetElectricFences(
               veh_point_enu, kElectricFenceSearchRadius, &electric_fences)) {
    if (FLAGS_allow_narrow_pass &&
        0 == base_map_ptr->GetJunctions(veh_point_enu, 0, &junctions)) {
      for (const auto &ptr_junction : junctions) {
        if (Junction::DongJiaZhen_Gate == ptr_junction->junction().type()) {
          // LongBuffer = 0.0;
          // LatBuffer = kNarrowPassLatBuffer;
          return false;
        }
      }
    }
    auto veh_box = common::VehicleConfigHelper::Instance()->GetBoundingBox(
        x, y, heading, LongBuffer, LatBuffer);
    // Check whether electric fence's polygon contain ADC bounding box.
    for (const auto &ele : electric_fences) {
      const auto &polygon = ele->polygon();  // ele->electric_fence().polygon();
      // AINFO << "electric fence id:" << ele->electric_fence().id().id();
      if (ele->electric_fence().type() == hdmap::ElectricFence::DRIVABLE) {
        if (polygon.Contains(Polygon2d(veh_box))) {
          // AINFO << "type:"
          //       <<
          //       ele->electric_fence().Type_Name(ele->electric_fence().type())
          //       << " VVVVVV";
          is_in_drivable_area = true;
        } else {
          AERROR << "electric fence id:" << ele->electric_fence().id().id()
                 << "type:"
                 << ele->electric_fence().Type_Name(
                        ele->electric_fence().type())
                 << ",x" << x << ",y:" << y << ",heading:" << heading
                 << " Veh is not in drivable area";
          is_in_drivable_area = false;
          break;
        }
      } else if (ele->electric_fence().type() ==
                 hdmap::ElectricFence::NOT_DRIVABLE) {
        if (polygon.HasOverlap(Polygon2d(veh_box))) {
          is_collision = true;
          AERROR << "electric fence id:" << ele->electric_fence().id().id()
                 << "type:"
                 << ele->electric_fence().Type_Name(
                        ele->electric_fence().type())
                 << ",x" << x << ",y:" << y << ",heading:" << heading
                 << " Veh is collision";
          break;
        } else {
          // AINFO << "type:"
          //       <<
          //       ele->electric_fence().Type_Name(ele->electric_fence().type())
          //       << " VVVVVV";
          is_collision = false;
        }
      }
    }
  } else {
    AERROR << "electric_fence_info is empty.";
  }
  return is_collision || !is_in_drivable_area;
}

double PathDecider::GetObstacleNearestPointS(const PathData &path_data,
                                             const Obstacle &obstacle) {
  const auto &config = config_.path_decider_config();
  const auto &obs_sl = obstacle.PerceptionSLBoundary();
  double stop_limit_s = obs_sl.start_s();
  double remain_dis =
      obs_sl.start_s() - reference_line_info_->AdcSlBoundary().start_s();
  if (path_data.discretized_path().empty() ||
      !config.enable_use_polygon_collision_check() ||
      remain_dis > config.use_obstacle_polygon_check_remain_dis()) {
    return stop_limit_s;
  }

  if (IsCollisionWithObstacle(path_data, obstacle, 0.0, &stop_limit_s)) {
    AINFO << "path will collision with obstacle: " << obstacle.Id()
          << ", stop_limit_s: " << stop_limit_s;
    return stop_limit_s;
  }

  double min_dist = std::numeric_limits<double>::max();
  const auto &discretized_path = path_data.discretized_path();
  auto min_it = discretized_path.begin();

  for (auto it = discretized_path.begin(); it != discretized_path.end(); ++it) {
    const auto adc_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(*it);
    double dis = obstacle.PerceptionPolygon().DistanceTo(adc_box);
    if (dis < min_dist) {
      min_dist = dis;
      min_it = it;
    }
  }

  common::SLPoint nearest_point_sl;
  common::math::Vec2d path_point = {min_it->x(), min_it->y()};
  if (reference_line_info_->reference_line().XYToSL(path_point,
                                                    &nearest_point_sl)) {
    AINFO << "get obs: " << obstacle.Id()
          << " polygon nearest point s: " << nearest_point_sl.s()
          << ", need add front edge to center";
    return nearest_point_sl.s() + common::VehicleConfigHelper::GetConfig()
                                      .vehicle_param()
                                      .front_edge_to_center();
  }

  return stop_limit_s;
}

void PathDecider::DumpCollisionDebug(const PathData &path_data,
                                     const common::PathPoint &path_point,
                                     double heading, int iter,
                                     double collision_distance) {
  if (!FLAGS_enable_dump_collision_debug) {
    return;
  }
  std::ofstream ofs("/century/data/text.txt");
  if (!ofs.is_open()) {
    AERROR << "Failed to open /century/data/text.txt for writing";
    return;
  }
  ofs << "# collision_iter=" << iter << "\n";
  // path points
  ofs << "# PATH_POINTS: x y theta kappa\n";
  for (const auto &pp : path_data.discretized_path()) {
    ofs << pp.x() << " " << pp.y() << " "
        << pp.theta() << " " << pp.kappa() << "\n";
  }
  // vehicle bounding box at collision point
  const auto &veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double front = veh_param.front_edge_to_center();
  double back = veh_param.back_edge_to_center();
  double left = veh_param.left_edge_to_center();
  double right = veh_param.right_edge_to_center();
  double cos_h = std::cos(heading);
  double sin_h = std::sin(heading);
  double cx = path_point.x();
  double cy = path_point.y();
  double corners[4][2] = {
      {cx + front * cos_h - left * sin_h,
       cy + front * sin_h + left * cos_h},
      {cx + front * cos_h + right * sin_h,
       cy + front * sin_h - right * cos_h},
      {cx - back * cos_h + right * sin_h,
       cy - back * sin_h - right * cos_h},
      {cx - back * cos_h - left * sin_h,
       cy - back * sin_h + left * cos_h}};
  ofs << "# VEH_BOX: x y (front_left, front_right, back_right, back_left)"
      << " front=" << front << " back=" << back
      << " left=" << left << " right=" << right << "\n";
  for (const auto &c : corners) {
    ofs << c[0] << " " << c[1] << "\n";
  }
  // only dump the colliding electric fence
  const hdmap::HDMap *base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<hdmap::ElectricFenceInfoConstPtr> fences;
  common::PointENU pt;
  pt.set_x(cx);
  pt.set_y(cy);
  if (nullptr != base_map_ptr &&
      0 == base_map_ptr->GetElectricFences(
          pt, kElectricFenceSearchRadius, &fences)) {
    auto veh_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(
            cx, cy, heading, kPathPointLongBuffer, kPathPointLatBuffer);
    Polygon2d veh_polygon(veh_box);
    for (const auto &ele : fences) {
      const auto &polygon = ele->polygon();
      bool is_collision_fence = false;
      if (ele->electric_fence().type() == hdmap::ElectricFence::DRIVABLE) {
        if (!polygon.Contains(veh_polygon)) {
          is_collision_fence = true;
        }
      } else if (ele->electric_fence().type() ==
                 hdmap::ElectricFence::NOT_DRIVABLE) {
        if (polygon.HasOverlap(veh_polygon)) {
          is_collision_fence = true;
        }
      }
      if (is_collision_fence) {
        ofs << "# FENCE: id=" << ele->electric_fence().id().id()
            << " type=" << ele->electric_fence().type() << "\n";
        for (const auto &p : polygon.points()) {
          ofs << p.x() << " " << p.y() << "\n";
        }
      }
    }
  }
  ofs.close();
  AINFO << "Collision debug data written to /century/data/text.txt";
}

}  // namespace planning
}  // namespace century
