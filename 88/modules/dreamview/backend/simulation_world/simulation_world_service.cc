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

#include "modules/dreamview/backend/simulation_world/simulation_world_service.h"

#include <unordered_set>

#include "absl/strings/str_split.h"
#include "google/protobuf/util/json_util.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/proto/vehicle_signal.pb.h"
#include "modules/dreamview/proto/simulation_world.pb.h"
#include "modules/mcloud/proto/mcloud_info.pb.h"

#include "cyber/common/file.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/points_downsampler.h"
#include "modules/common/util/util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace century {
namespace dreamview {

using century::audio::AudioDetection;
using century::audio::AudioEvent;
using century::canbus::Chassis;
using century::common::DriveEvent;
using century::common::PathPoint;
using century::common::Point3D;
using century::common::PointENU;
using century::common::TrajectoryPoint;
using century::common::VehicleConfigHelper;
using century::common::monitor::MonitorMessage;
using century::common::monitor::MonitorMessageItem;
using century::common::util::DownsampleByAngle;
using century::common::util::FillHeader;
using century::control::ControlCommand;
using century::cyber::Clock;
using century::cyber::common::GetProtoFromFile;
using century::hdmap::Curve;
using century::hdmap::Map;
using century::hdmap::Path;
using century::localization::Gps;
using century::localization::LocalizationEstimate;
using century::perception::PerceptionObstacle;
using century::perception::PerceptionObstacles;
using century::perception::SensorMeasurement;
using century::perception::TrafficLight;
using century::perception::TrafficLightDetection;
using century::perception::V2XInformation;
using century::planning::ADCTrajectory;
using century::planning::DecisionResult;
using century::planning::StopReasonCode;
using century::planning_internal::PlanningData;
using century::prediction::ObstacleInteractiveTag;
using century::prediction::ObstaclePriority;
using century::prediction::PredictionObstacle;
using century::prediction::PredictionObstacles;
using century::relative_map::MapMsg;
using century::relative_map::NavigationInfo;
using century::routing::RoutingRequest;
using century::routing::RoutingResponse;
using century::routing::RoutingResponses;
using century::storytelling::Stories;
using century::task_manager::Task;
using century::mcloud::McloudInfo;
using century::planning::BorrowResponse;
using century::planning::PassStackerResponse;
using century::monitor::MonitoredData;
using century::fas_aeb_backend::FasAebInfo;
using century::dreamview::BarrierCommand;
using century::dreamview::BackgroundMusic;

using Json = nlohmann::json;
using ::google::protobuf::util::MessageToJsonString;

// Angle threshold is about 5.72 degree.
static constexpr double kAngleThreshold = 0.1;

namespace {

double CalculateAcceleration(
    const Point3D &acceleration, const Point3D &velocity,
    const century::canbus::Chassis_GearPosition &gear_location) {
  // Calculates the dot product of acceleration and velocity. The sign
  // of this projection indicates whether this is acceleration or
  // deceleration.
  double projection =
      acceleration.x() * velocity.x() + acceleration.y() * velocity.y();

  // Calculates the magnitude of the acceleration. Negate the value if
  // it is indeed a deceleration.
  double magnitude = std::hypot(acceleration.x(), acceleration.y());
  if (std::signbit(projection)) {
    magnitude = -magnitude;
  }

  // Negate the value if gear is reverse
  if (gear_location == Chassis::GEAR_REVERSE) {
    magnitude = -magnitude;
  }

  return magnitude;
}

double GetDistanceToRect(double ax, double ay, double vx, double vy, double cx, double cy) {
  const double hx = cx / 2.0;
  const double hy = cy / 2.0;

  double x = std::abs(ax - vx);
  double y = std::abs(ay - vy);

  double dx = x - hx;
  double dy = y - hy;

  double externalDistance = std::sqrt(std::max(dx, 0.0) * std::max(dx, 0.0) + 
                                      std::max(dy, 0.0) * std::max(dy, 0.0));

  double internalDistance = std::min(std::max(dx, dy), 0.0);

  return externalDistance + internalDistance;
}

Eigen::Affine3d BuildTransform(
  double tx, double ty, double tz,
  double qw, double qx, double qy, double qz) {
  Eigen::Translation3d translation(tx, ty, tz);
  Eigen::Quaterniond rotation(qw, qx, qy, qz);
  rotation.normalize();
  Eigen::Affine3d affine = translation * rotation;
  return affine;
}

Object::DisengageType DeduceDisengageType(const Chassis &chassis) {
  if (chassis.error_code() != Chassis::NO_ERROR) {
    return Object::DISENGAGE_CHASSIS_ERROR;
  }

  switch (chassis.driving_mode()) {
    case Chassis::COMPLETE_AUTO_DRIVE:
      return Object::DISENGAGE_NONE;
    case Chassis::COMPLETE_MANUAL:
      return Object::DISENGAGE_MANUAL;
    case Chassis::AUTO_STEER_ONLY:
      return Object::DISENGAGE_AUTO_STEER_ONLY;
    case Chassis::AUTO_SPEED_ONLY:
      return Object::DISENGAGE_AUTO_SPEED_ONLY;
    case Chassis::EMERGENCY_MODE:
      return Object::DISENGAGE_EMERGENCY;
    case Chassis::COMPLETE_REMOTE:
      return Object::DISENGAGE_REMOTE;
    case Chassis::COMPLETE_MEDIAN:
      return Object::DISENGAGE_MEDIAN;
    case Chassis::COMPLETE_MAINTENANCE:
      return Object::DISENGAGE_MAINTENANCE;
    default:
      return Object::DISENGAGE_UNKNOWN;
  }
}

void SetObstacleType(const PerceptionObstacle::Type obstacle_type,
                     const PerceptionObstacle::SubType obstacle_subtype,
                     Object *world_object) {
  if (world_object == nullptr) {
    return;
  }

  switch (obstacle_type) {
    case PerceptionObstacle::UNKNOWN:
      world_object->set_type(Object_Type_UNKNOWN);
      break;
    case PerceptionObstacle::UNKNOWN_MOVABLE:
      world_object->set_type(Object_Type_UNKNOWN_MOVABLE);
      break;
    case PerceptionObstacle::UNKNOWN_UNMOVABLE:
      world_object->set_type(Object_Type_UNKNOWN_UNMOVABLE);
      break;
    case PerceptionObstacle::PEDESTRIAN:
      world_object->set_type(Object_Type_PEDESTRIAN);
      break;
    case PerceptionObstacle::BICYCLE:
      world_object->set_type(Object_Type_BICYCLE);
      break;
    case PerceptionObstacle::VEHICLE:
      world_object->set_type(Object_Type_VEHICLE);
      break;
    case PerceptionObstacle::WIDE40FOOT:
      world_object->set_type(Object_Type_WIDE40FOOT);
      break;
    case PerceptionObstacle::NARROW20FOOT:
      world_object->set_type(Object_Type_NARROW20FOOT);
      break;
    case PerceptionObstacle::CONE:
      world_object->set_type(Object_Type_CONE);
      break;
    case PerceptionObstacle::STACKER:
      world_object->set_type(Object_Type_STACKER);
      break;
    case PerceptionObstacle::FORKLIFT_STACKER:
      world_object->set_type(Object_Type_FORKLIFT_STACKER);
      break;
    case PerceptionObstacle::WHEELCRANE:
      world_object->set_type(Object_Type_WHEELCRANE);
      break;
    default:
      world_object->set_type(Object_Type_VIRTUAL);
  }

  world_object->set_sub_type(obstacle_subtype);
}

void SetStopReason(const StopReasonCode &reason_code, Decision *decision) {
  switch (reason_code) {
    case StopReasonCode::STOP_REASON_HEAD_VEHICLE:
      decision->set_stopreason(Decision::STOP_REASON_HEAD_VEHICLE);
      break;
    case StopReasonCode::STOP_REASON_DESTINATION:
      decision->set_stopreason(Decision::STOP_REASON_DESTINATION);
      break;
    case StopReasonCode::STOP_REASON_PEDESTRIAN:
      decision->set_stopreason(Decision::STOP_REASON_PEDESTRIAN);
      break;
    case StopReasonCode::STOP_REASON_OBSTACLE:
      decision->set_stopreason(Decision::STOP_REASON_OBSTACLE);
      break;
    case StopReasonCode::STOP_REASON_SIGNAL:
      decision->set_stopreason(Decision::STOP_REASON_SIGNAL);
      break;
    case StopReasonCode::STOP_REASON_STOP_SIGN:
      decision->set_stopreason(Decision::STOP_REASON_STOP_SIGN);
      break;
    case StopReasonCode::STOP_REASON_YIELD_SIGN:
      decision->set_stopreason(Decision::STOP_REASON_YIELD_SIGN);
      break;
    case StopReasonCode::STOP_REASON_CLEAR_ZONE:
      decision->set_stopreason(Decision::STOP_REASON_CLEAR_ZONE);
      break;
    case StopReasonCode::STOP_REASON_CROSSWALK:
      decision->set_stopreason(Decision::STOP_REASON_CROSSWALK);
      break;
    case StopReasonCode::STOP_REASON_PULL_OVER:
      decision->set_stopreason(Decision::STOP_REASON_PULL_OVER);
      break;
    default:
      AWARN << "Unrecognizable stop reason code:" << reason_code;
  }
}

void UpdateTurnSignal(const century::common::VehicleSignal &signal,
                      Object *auto_driving_car) {
  if (signal.turn_signal() == century::common::VehicleSignal::TURN_LEFT) {
    auto_driving_car->set_current_signal("LEFT");
  } else if (signal.turn_signal() ==
             century::common::VehicleSignal::TURN_RIGHT) {
    auto_driving_car->set_current_signal("RIGHT");
  } else if (signal.emergency_light()) {
    auto_driving_car->set_current_signal("EMERGENCY");
  } else {
    auto_driving_car->set_current_signal("");
  }
}

void DownsampleCurve(Curve *curve) {
  if (curve->segment().empty()) {
    return;
  }

  auto *line_segment = curve->mutable_segment(0)->mutable_line_segment();
  std::vector<PointENU> points(line_segment->point().begin(),
                               line_segment->point().end());
  line_segment->clear_point();

  // Downsample points by angle then by distance.
  std::vector<size_t> sampled_indices =
      DownsampleByAngle(points, kAngleThreshold);
  for (const size_t index : sampled_indices) {
    *line_segment->add_point() = points[index];
  }
}

inline double SecToMs(const double sec) { return sec * 1000.0; }

}  // namespace

constexpr int SimulationWorldService::kMaxMonitorItems;

SimulationWorldService::SimulationWorldService(const MapService *map_service,
                                               bool routing_from_file)
    : node_(cyber::CreateNode("simulation_world")),
      map_service_(map_service),
      monitor_logger_buffer_(MonitorMessageItem::SIMULATOR),
      ready_to_push_(false) {
  InitReaders();
  InitWriters();

  if (routing_from_file) {
    ReadRoutingFromFile(FLAGS_routing_response_file);
  }
  // Populate vehicle parameters.
  Object *auto_driving_car = world_.mutable_auto_driving_car();
  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  auto_driving_car->set_height(vehicle_param.height());
  auto_driving_car->set_width(vehicle_param.width());
  auto_driving_car->set_length(vehicle_param.length());
}

void SimulationWorldService::InitReaders() {
  fas_aeb_backend_client_ =
      node_->CreateClient<century::fas_aeb_backend::Request,
                          century::fas_aeb_backend::Response>(
          "fas_aeb_backend");

  routing_request_reader_ =
      node_->CreateReader<RoutingRequest>(
          FLAGS_routing_request_topic,
          [this](const std::shared_ptr<RoutingRequest>& request) {
            if (request && request->has_header() &&
                "cloud" == request->header().module_name()) {
              last_external_routing_request_ =
                  std::make_shared<RoutingRequest>(*request);
            }
          });

  routing_response_reader_ =
      node_->CreateReader<RoutingResponse>(FLAGS_routing_response_topic);

  routing_responses_reader_ =
      node_->CreateReader<RoutingResponses>("/century/routing_responses");      
      
  stackers_info_reader_ = 
      node_->CreateReader<century::planning::StackersInfo>("/century/stackers_info");
      
  chassis_reader_ = node_->CreateReader<Chassis>(FLAGS_chassis_topic);
  gps_reader_ = node_->CreateReader<Gps>(FLAGS_gps_topic);
  localization_reader_ =
      node_->CreateReader<LocalizationEstimate>(FLAGS_localization_topic);
  perception_obstacle_reader_ =
      node_->CreateReader<PerceptionObstacles>(FLAGS_perception_obstacle_topic);
  perception_aound_ego_obstacle_reader_ =
      node_->CreateReader<PerceptionObstacles>(FLAGS_perception_around_ego_obstacle_topic);
  perception_traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      FLAGS_traffic_light_detection_topic);
  prediction_obstacle_reader_ =
      node_->CreateReader<PredictionObstacles>(FLAGS_prediction_topic);
  planning_reader_ =
      node_->CreateReader<ADCTrajectory>(FLAGS_planning_trajectory_topic,
        [this](const std::shared_ptr<ADCTrajectory>& trajectory) {
          world_.set_enable_auto_drive(false);
          if (trajectory->has_enable_auto_drive()) {
            world_.set_enable_auto_drive(trajectory->enable_auto_drive());
          } 
          if (ADCTrajectory::RAILWAY_CROSSING ==  trajectory->warning_type()) {
            PublishMonitorMessage(MonitorMessageItem::WARN, "Planning Warning : RAILWAY_CROSSING");
          }
        }
      );

  fas_aeb_info_reader_ =
      node_->CreateReader<FasAebInfo>(FLAGS_fas_aeb_info_topic);
  control_command_reader_ =
      node_->CreateReader<ControlCommand>(FLAGS_control_command_topic);
  navigation_reader_ =
      node_->CreateReader<NavigationInfo>(FLAGS_navigation_topic);
  relative_map_reader_ = node_->CreateReader<MapMsg>(FLAGS_relative_map_topic);
  storytelling_reader_ = node_->CreateReader<Stories>(FLAGS_storytelling_topic);
  audio_detection_reader_ =
      node_->CreateReader<AudioDetection>(FLAGS_audio_detection_topic);

#if defined __aarch64__
constexpr const char* kServiceName = "/century/monitor/monitor_data_aarch";
#else
constexpr const char* kServiceName = "/century/monitor/monitor_data_x86";
#endif

  system_monitor_reader_ = node_->CreateReader<MonitoredData>(
      kServiceName);

  audio_event_reader_ = node_->CreateReader<AudioEvent>(
      FLAGS_audio_event_topic,
      [this](const std::shared_ptr<AudioEvent> &audio_event) {
        this->PublishMonitorMessage(
            MonitorMessageItem::WARN,
            century::audio::AudioType_Name(audio_event->audio_type()));
      });
  drive_event_reader_ = node_->CreateReader<DriveEvent>(
      FLAGS_drive_event_topic,
      [this](const std::shared_ptr<DriveEvent> &drive_event) {
        this->PublishMonitorMessage(MonitorMessageItem::WARN,
                                    drive_event->event());
      });
  cyber::ReaderConfig monitor_message_reader_config;
  monitor_message_reader_config.channel_name = FLAGS_monitor_topic;
  monitor_message_reader_config.pending_queue_size =
      FLAGS_monitor_msg_pending_queue_size;
  monitor_reader_ = node_->CreateReader<MonitorMessage>(
      monitor_message_reader_config,
      [this](const std::shared_ptr<MonitorMessage> &monitor_message) {
        std::unique_lock<std::mutex> lock(monitor_msgs_mutex_);
        monitor_msgs_.push_back(monitor_message);
      });
  task_reader_ = node_->CreateReader<Task>(FLAGS_task_topic);
}

nlohmann::json SimulationWorldService::GetSystemInfo() {
  Json ret;
  if (system_monitor_reader_ == nullptr) {
    return ret;
  }
  system_monitor_reader_->Observe();
  auto system_info = system_monitor_reader_->GetLatestObserved();
  if (system_info == nullptr) {
    return ret;
  }

  ret["cpu"] = system_info->system_monitor_data().system_data().cpu_usage();
  ret["memory"] = system_info->system_monitor_data().system_data().mem_usage();
  ret["disk"] = system_info->system_monitor_data().system_data().disk_usage();

  if (system_info->fault_data().size()){
    int max_level = -1;
    for(const auto &faultdata : system_info->fault_data()){
      if(faultdata.level() > max_level){
        max_level = faultdata.level();
      }
    }
    if (1 == max_level) {
      ret["status"] = "ok";
    }else if (2 == max_level){
      ret["status"] = "warning";
    }else{
      ret["status"] = "error";
    }
  }else{
    ret["status"] = "ok";
  }

  return ret;
}


void SimulationWorldService::InitWriters() {
  navigation_writer_ =
      node_->CreateWriter<NavigationInfo>(FLAGS_navigation_topic);

  {  // configure QoS for routing request writer
    century::cyber::proto::RoleAttributes routing_request_attr;
    routing_request_attr.set_channel_name(FLAGS_routing_request_topic);
    auto qos = routing_request_attr.mutable_qos_profile();
    // only keeps the last message in history
    qos->set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    // reliable transfer
    qos->set_reliability(
        century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    // when writer find new readers, send all its history messsage
    qos->set_durability(
        century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
    routing_request_writer_ =
        node_->CreateWriter<RoutingRequest>(routing_request_attr);
  }

  {  // configure QoS for routing response writer
    century::cyber::proto::RoleAttributes borrow_response_attr;
    borrow_response_attr.set_channel_name("/century/borrow_response");
    auto qos = borrow_response_attr.mutable_qos_profile();
    // only keeps the last message in history
    qos->set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    // reliable transfer
    qos->set_reliability(
        century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    // when writer find new readers, send all its history messsage
    qos->set_durability(
        century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
    planning_borrow_response_writer_ =
        node_->CreateWriter<BorrowResponse>(borrow_response_attr);
  }


  {  // configure QoS for routing response writer
    century::cyber::proto::RoleAttributes pass_stacker_response;
    pass_stacker_response.set_channel_name("/century/pass_stacker_response");
    auto qos = pass_stacker_response.mutable_qos_profile();
    // only keeps the last message in history
    qos->set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    // reliable transfer
    qos->set_reliability(
        century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    // when writer find new readers, send all its history messsage
    qos->set_durability(
        century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
        planning_pass_stacker_writer_ =
        node_->CreateWriter<PassStackerResponse>(pass_stacker_response);
  }

  routing_response_writer_ =
      node_->CreateWriter<RoutingResponse>(FLAGS_routing_response_topic);
  task_writer_ = node_->CreateWriter<Task>(FLAGS_task_topic);

  {
    century::cyber::proto::RoleAttributes immediately_attr;
    immediately_attr.set_channel_name("/century/mcloud");
    auto qos = immediately_attr.mutable_qos_profile();
    qos->set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    qos->set_reliability(century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    qos->set_durability(century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);

    cloud_info_writer_ = node_->CreateWriter<McloudInfo>(immediately_attr);
  }

  {
    century::cyber::proto::RoleAttributes barrier_command_attr;
    barrier_command_attr.set_channel_name(FLAGS_barrier_command_topic);
    auto qos = barrier_command_attr.mutable_qos_profile();
    qos->set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    qos->set_reliability(
        century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    qos->set_durability(
        century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
    barrier_command_writer_ =
        node_->CreateWriter<BarrierCommand>(barrier_command_attr);
  }

  bck_music_writer_ = node_->CreateWriter<BackgroundMusic>("/century/background_music");
}

void SimulationWorldService::Update() {
  if (to_clear_) {
    // Clears received data.
    node_->ClearData();

    // Clears simulation world except for the car information.
    auto car = world_.auto_driving_car();
    world_.Clear();
    *world_.mutable_auto_driving_car() = car;

    {
      boost::unique_lock<boost::shared_mutex> writer_lock(route_paths_mutex_);
      route_paths_.clear();
    }

    to_clear_ = false;
  }

  node_->Observe();

  UpdateBarrierCommand();
  UpdateMonitorMessages();
  UpdateBorrowRequest();
  UpdateWithLatestObserved(routing_response_reader_.get(), false);
  UpdateWithLatestObserved(routing_responses_reader_.get(), false);

  UpdateWithLatestObserved(chassis_reader_.get());
  UpdateWithLatestObserved(gps_reader_.get());
  UpdateWithLatestObserved(localization_reader_.get());

  // Clear objects received from last frame and populate with the new objects.
  // TODO(siyangy, unacao): For now we are assembling the simulation_world with
  // latest received perception, prediction and planning message. However, they
  // may not always be perfectly aligned and belong to the same frame.
  obj_map_.clear();
  world_.clear_object();
  world_.clear_sensor_measurements();
  UpdateWithLatestObserved(audio_detection_reader_.get());
  UpdateWithLatestObserved(storytelling_reader_.get());
  UpdateWithLatestObserved(perception_obstacle_reader_.get());
  UpdateAroundEgoWithLatestObserved(perception_aound_ego_obstacle_reader_.get());
  UpdateWithLatestObserved(perception_traffic_light_reader_.get(), false);
  UpdateWithLatestObserved(prediction_obstacle_reader_.get());
  UpdateWithLatestObserved(planning_reader_.get());
  UpdateWithLatestObserved(control_command_reader_.get());
  UpdateWithLatestObserved(navigation_reader_.get(), FLAGS_use_navigation_mode);
  UpdateWithLatestObserved(relative_map_reader_.get(),
                           FLAGS_use_navigation_mode);
  UpdateWithLatestObserved(fas_aeb_info_reader_.get());

  for (const auto &kv : obj_map_) {
    *world_.add_object() = kv.second;
  }

  UpdateDelays();
  UpdateLatencies();

  world_.set_sequence_num(world_.sequence_num() + 1);
  world_.set_timestamp(Clock::Now().ToSecond() * 1000);
}

void SimulationWorldService::UpdateDelays() {
  auto *delays = world_.mutable_delay();
  delays->set_chassis(SecToMs(chassis_reader_->GetDelaySec()));
  delays->set_localization(SecToMs(localization_reader_->GetDelaySec()));
  delays->set_perception_obstacle(
      SecToMs(perception_obstacle_reader_->GetDelaySec()));
  delays->set_planning(SecToMs(planning_reader_->GetDelaySec()));
  delays->set_prediction(SecToMs(prediction_obstacle_reader_->GetDelaySec()));
  delays->set_traffic_light(
      SecToMs(perception_traffic_light_reader_->GetDelaySec()));
  delays->set_control(SecToMs(control_command_reader_->GetDelaySec()));
}

void SimulationWorldService::UpdateLatencies() {
  UpdateLatency("chassis", chassis_reader_.get());
  UpdateLatency("localization", localization_reader_.get());
  UpdateLatency("perception", perception_obstacle_reader_.get());
  UpdateLatency("planning", planning_reader_.get());
  UpdateLatency("prediction", prediction_obstacle_reader_.get());
  UpdateLatency("control", control_command_reader_.get());
}

void SimulationWorldService::GetWireFormatString(
    double radius, std::string *sim_world,
    std::string *sim_world_with_planning_data) {
  PopulateMapInfo(radius);

  world_.SerializeToString(sim_world_with_planning_data);
  world_.clear_planning_data();
  world_.SerializeToString(sim_world);
}

Json SimulationWorldService::GetUpdateAsJson(double radius) const {
  std::string sim_world_json_string;
  MessageToJsonString(world_, &sim_world_json_string);

  Json update;
  update["type"] = "SimWorldUpdate";
  update["timestamp"] = Clock::Now().ToSecond() * 1000;
  update["world"] = sim_world_json_string;

  return update;
}

void SimulationWorldService::GetMapElementIds(double radius,
                                              MapElementIds *ids) const {
  // Gather required map element ids based on current location.
  century::common::PointENU point;
  const auto &adc = world_.auto_driving_car();
  point.set_x(adc.position_x());
  point.set_y(adc.position_y());
  map_service_->CollectMapElementIds(point, radius, ids);
}

void SimulationWorldService::PopulateMapInfo(double radius) {
  world_.clear_map_element_ids();
  GetMapElementIds(radius, world_.mutable_map_element_ids());
  world_.set_map_hash(map_service_->CalculateMapHash(world_.map_element_ids()));
  world_.set_map_radius(radius);
}

const Map &SimulationWorldService::GetRelativeMap() const {
  return relative_map_;
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const FasAebInfo &fas_aeb_info) {
  std::cout << "fas_aeb_info: " << fas_aeb_info.fas_aeb_switch() << std::endl;
  world_.set_fas_aeb_switch(fas_aeb_info.fas_aeb_switch());
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const LocalizationEstimate &localization) {
  Object *auto_driving_car = world_.mutable_auto_driving_car();
  const auto &pose = localization.pose();

  // Updates position with the input localization message.
  auto_driving_car->set_position_x(pose.position().x() +
                                   map_service_->GetXOffset());
  auto_driving_car->set_position_y(pose.position().y() +
                                   map_service_->GetYOffset());
  auto_driving_car->set_heading(pose.heading());

  // Updates acceleration with the input localization message.
  auto_driving_car->set_speed_acceleration(CalculateAcceleration(
      pose.linear_acceleration(), pose.linear_velocity(), gear_location_));

  // Updates the timestamp with the timestamp inside the localization
  // message header. It is done on both the SimulationWorld object
  // itself and its auto_driving_car() field.
  auto_driving_car->set_timestamp_sec(localization.header().timestamp_sec());
  MaybePrintNearestObstacleWhenNearDestination(localization);
  ready_to_push_.store(true);
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Gps &gps) {
  if (gps.header().module_name() == "ShadowLocalization") {
    Object *shadow_localization_position = world_.mutable_shadow_localization();
    const auto &pose = gps.localization();
    shadow_localization_position->set_position_x(pose.position().x() +
                                                 map_service_->GetXOffset());
    shadow_localization_position->set_position_y(pose.position().y() +
                                                 map_service_->GetYOffset());
    shadow_localization_position->set_heading(pose.heading());
  } else {
    Object *gps_position = world_.mutable_gps();
    gps_position->set_timestamp_sec(gps.header().timestamp_sec());

    const auto &pose = gps.localization();
    gps_position->set_position_x(pose.position().x() +
                                 map_service_->GetXOffset());
    gps_position->set_position_y(pose.position().y() +
                                 map_service_->GetYOffset());

    double heading = century::common::math::QuaternionToHeading(
        pose.orientation().qw(), pose.orientation().qx(),
        pose.orientation().qy(), pose.orientation().qz());
    gps_position->set_heading(heading);
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Chassis &chassis) {
  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  Object *auto_driving_car = world_.mutable_auto_driving_car();

  double speed = chassis.speed_mps();
  gear_location_ = chassis.gear_location();
  if (gear_location_ == Chassis::GEAR_REVERSE) {
    speed = -speed;
  }
  auto_driving_car->set_speed(speed);
  auto_driving_car->set_throttle_percentage(chassis.throttle_percentage());
  auto_driving_car->set_brake_percentage(chassis.brake_percentage());

  // In case of out-of-range percentages, reduces it to zero.
  double angle_percentage = chassis.steering_percentage();
  if (angle_percentage > 100 || angle_percentage < -100) {
    angle_percentage = 0;
  }
  auto_driving_car->set_steering_percentage(angle_percentage);

  double steering_angle =
      angle_percentage / 100.0 * vehicle_param.max_steer_angle();
  auto_driving_car->set_steering_angle(steering_angle);

  double kappa = std::tan(steering_angle / vehicle_param.steer_ratio()) /
                 vehicle_param.wheel_base();
  auto_driving_car->set_kappa(kappa);

  UpdateTurnSignal(chassis.signal(), auto_driving_car);

  auto_driving_car->set_disengage_type(DeduceDisengageType(chassis));

  auto_driving_car->set_battery_percentage(chassis.battery_soc_percentage());
  auto_driving_car->set_gear_location(chassis.gear_location());
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const Stories &stories) {
  world_.clear_stories();
  auto *world_stories = world_.mutable_stories();

  const google::protobuf::Descriptor *descriptor = stories.GetDescriptor();
  const google::protobuf::Reflection *reflection = stories.GetReflection();
  const int field_count = descriptor->field_count();
  for (int i = 0; i < field_count; ++i) {
    const google::protobuf::FieldDescriptor *field = descriptor->field(i);
    if (field->name() != "header") {
      (*world_stories)[field->name()] = reflection->HasField(stories, field);
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const AudioDetection &audio_detection) {
  world_.set_is_siren_on(audio_detection.is_siren());
}

Object &SimulationWorldService::CreateWorldObjectIfAbsent(
    const PerceptionObstacle &obstacle) {
  const std::string id = std::to_string(obstacle.id());
  // Create a new world object and put it into object map if the id does not
  // exist in the map yet.
  if (!century::common::util::ContainsKey(obj_map_, id)) {
    Object &world_obj = obj_map_[id];
    SetObstacleInfo(obstacle, &world_obj);
    SetObstaclePolygon(obstacle, &world_obj);
    SetObstacleType(obstacle.type(), obstacle.sub_type(), &world_obj);
    SetObstacleSensorMeasurements(obstacle, &world_obj);
    SetObstacleSource(obstacle, &world_obj);
  }
  return obj_map_[id];
}

void SimulationWorldService::CreateWorldObjectFromSensorMeasurement(
    const SensorMeasurement &sensor, Object *world_object) {
  world_object->set_id(std::to_string(sensor.id()));
  world_object->set_position_x(sensor.position().x());
  world_object->set_position_y(sensor.position().y());
  world_object->set_heading(sensor.theta());
  world_object->set_length(sensor.length());
  world_object->set_width(sensor.width());
  world_object->set_height(sensor.height());
  SetObstacleType(sensor.type(), sensor.sub_type(), world_object);
}

void SimulationWorldService::SetObstacleInfo(const PerceptionObstacle &obstacle,
                                             Object *world_object) {
  if (world_object == nullptr) {
    return;
  }

  world_object->set_id(std::to_string(obstacle.id()));
  world_object->set_position_x(obstacle.position().x() +
                               map_service_->GetXOffset());
  world_object->set_position_y(obstacle.position().y() +
                               map_service_->GetYOffset());
  world_object->set_heading(obstacle.theta());
  world_object->set_length(obstacle.length());
  world_object->set_width(obstacle.width());
  world_object->set_height(obstacle.height());
  world_object->set_speed(
      std::hypot(obstacle.velocity().x(), obstacle.velocity().y()));
  world_object->set_speed_heading(
      std::atan2(obstacle.velocity().y(), obstacle.velocity().x()));
  world_object->set_timestamp_sec(obstacle.timestamp());
  world_object->set_confidence(obstacle.has_confidence() ? obstacle.confidence()
                                                         : 1);
}

void SimulationWorldService::SetObstaclePolygon(
    const PerceptionObstacle &obstacle, Object *world_object) {
  if (world_object == nullptr) {
    return;
  }

  using century::common::util::PairHash;
  std::unordered_set<std::pair<double, double>, PairHash> seen_points;
  world_object->clear_polygon_point();
  for (const auto &point : obstacle.polygon_point()) {
    // Filter out duplicate xy pairs.
    std::pair<double, double> xy_pair = {point.x(), point.y()};
    if (seen_points.count(xy_pair) == 0) {
      PolygonPoint *poly_pt = world_object->add_polygon_point();
      poly_pt->set_x(point.x() + map_service_->GetXOffset());
      poly_pt->set_y(point.y() + map_service_->GetYOffset());
      seen_points.insert(xy_pair);
    }
  }
}

void SimulationWorldService::SetObstacleSensorMeasurements(
    const PerceptionObstacle &obstacle, Object *world_object) {
  if (world_object == nullptr) {
    return;
  }
  for (const auto &sensor : obstacle.measurements()) {
    Object *obj = (*(world_.mutable_sensor_measurements()))[sensor.sensor_id()]
                      .add_sensor_measurement();
    CreateWorldObjectFromSensorMeasurement(sensor, obj);
  }
}

void SimulationWorldService::SetObstacleSource(
    const century::perception::PerceptionObstacle &obstacle,
    Object *world_object) {
  if (world_object == nullptr || !obstacle.has_source()) {
    return;
  }
  const PerceptionObstacle::Source obstacle_source = obstacle.source();
  world_object->set_source(obstacle_source);
  world_object->clear_v2x_info();
  if (obstacle_source == PerceptionObstacle::V2X && obstacle.has_v2x_info()) {
    world_object->mutable_v2x_info()->CopyFrom(obstacle.v2x_info());
  }
  return;
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const PerceptionObstacles &obstacles) {
  for (const auto &obstacle : obstacles.perception_obstacle()) {
    auto &world_obj = CreateWorldObjectIfAbsent(obstacle);
    if (obstacles.has_cipv_info() &&
        (obstacles.cipv_info().cipv_id() == obstacle.id())) {
      world_obj.set_type(Object_Type_CIPV);
    }
    world_obj.set_is_aeb_type("false");
  }

  if (obstacles.has_lane_marker()) {
    world_.mutable_lane_marker()->CopyFrom(obstacles.lane_marker());
  }
}

template <>
void SimulationWorldService::UpdateSimulationAroundEgoWorld(
    const PerceptionObstacles& obstacles) {
  const auto& lidar2world = obstacles.lidar2world();
  const auto& adc = world_.auto_driving_car();
  const double adc_x = adc.position_x();
  const double adc_y = adc.position_y();
  const double vehicle_length = adc.length();
  const double vehicle_width = adc.width();
  constexpr double kMaxDistance = 5.0;

  Eigen::Affine3d T = BuildTransform(
    lidar2world.tx(), lidar2world.ty(), lidar2world.tz(),
    lidar2world.qw(), lidar2world.qx(), lidar2world.qy(), lidar2world.qz()
  );

  for (const auto& obstacle : obstacles.perception_obstacle()) {
    PerceptionObstacle adjusted = obstacle;
    const auto p_old = Eigen::Vector3d(obstacle.position().x(), obstacle.position().y(), obstacle.position().z());
    const auto p_new = T * p_old;
    adjusted.mutable_position()->set_x(p_new.x());
    adjusted.mutable_position()->set_y(p_new.y());
    adjusted.mutable_position()->set_z(p_new.z());

    const auto distance = 
      GetDistanceToRect(
        adjusted.position().x(),
        adjusted.position().y(),
        adc_x,
        adc_y,
        vehicle_length,
        vehicle_width);
    if (distance > kMaxDistance) {
      continue;
    }

    auto* polygon = adjusted.mutable_polygon_point();
    for (auto& point : *polygon) {
      const auto p_old = Eigen::Vector3d(point.x(), point.y(), point.z());
      const auto p_new = T * p_old;
      point.set_x(p_new.x());
      point.set_y(p_new.y());
      point.set_z(p_new.z());
    }
    auto& world_obj = CreateWorldObjectIfAbsent(adjusted);

    if (obstacles.has_cipv_info() &&
        obstacles.cipv_info().cipv_id() == obstacle.id()) {
      world_obj.set_type(Object_Type_CIPV);
    }

    world_obj.set_is_aeb_type("true");
  }

  if (obstacles.has_lane_marker()) {
    world_.mutable_lane_marker()->CopyFrom(obstacles.lane_marker());
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const TrafficLightDetection &traffic_light_detection) {
  world_.clear_perceived_signal();
  for (const auto &traffic_light : traffic_light_detection.traffic_light()) {
    Object *signal = world_.add_perceived_signal();
    signal->set_id(traffic_light.id());
    signal->set_current_signal(TrafficLight_Color_Name(traffic_light.color()));
  }
}

void SimulationWorldService::UpdatePlanningTrajectory(
    const ADCTrajectory &trajectory) {
  // Collect trajectory
  world_.clear_planning_trajectory();
  const double base_time = trajectory.header().timestamp_sec();
  for (const TrajectoryPoint &point : trajectory.trajectory_point()) {
    Object *trajectory_point = world_.add_planning_trajectory();
    trajectory_point->set_timestamp_sec(point.relative_time() + base_time);
    trajectory_point->set_position_x(point.path_point().x() +
                                     map_service_->GetXOffset());
    trajectory_point->set_position_y(point.path_point().y() +
                                     map_service_->GetYOffset());
    trajectory_point->set_speed(point.v());
    trajectory_point->set_speed_acceleration(point.a());
    trajectory_point->set_kappa(point.path_point().kappa());
    trajectory_point->set_dkappa(point.path_point().dkappa());
    trajectory_point->set_heading(point.path_point().theta());
  }

  // Update engage advice.
  // This is a temporary solution, the advice will come from monitor later
  if (trajectory.has_engage_advice()) {
    world_.set_engage_advice(
        EngageAdvice_Advice_Name(trajectory.engage_advice().advice()));
  }

  // Update background_music_switch
  if (trajectory.has_background_music_enable()) {
    world_.set_background_music_switch(trajectory.background_music_enable() ? "true" : "false");
  }
}

std::string formatDoubleToString(const double data) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << data;
  return ss.str();
}

void SimulationWorldService::UpdateRSSInfo(const ADCTrajectory &trajectory) {
  if (trajectory.has_rss_info()) {
    if (trajectory.rss_info().is_rss_safe()) {
      if (!world_.is_rss_safe()) {
        this->PublishMonitorMessage(MonitorMessageItem::INFO, "RSS safe.");
        world_.set_is_rss_safe(true);
      }
    } else {
      const double next_real_dist = trajectory.rss_info().cur_dist_lon();
      const double next_rss_safe_dist =
          trajectory.rss_info().rss_safe_dist_lon();
      // Not update RSS message if data keeps same.
      if (std::fabs(current_real_dist_ - next_real_dist) <
              common::math::kMathEpsilon &&
          std::fabs(current_rss_safe_dist_ - next_rss_safe_dist) <
              common::math::kMathEpsilon) {
        return;
      }
      this->PublishMonitorMessage(
          MonitorMessageItem::ERROR,
          "RSS unsafe: \ncurrent distance: " +
              formatDoubleToString(trajectory.rss_info().cur_dist_lon()) +
              "\nsafe distance: " +
              formatDoubleToString(trajectory.rss_info().rss_safe_dist_lon()));
      world_.set_is_rss_safe(false);
      current_real_dist_ = next_real_dist;
      current_rss_safe_dist_ = next_rss_safe_dist;
    }
  }
}

void SimulationWorldService::UpdateMainStopDecision(
    const century::planning::MainDecision &main_decision,
    double update_timestamp_sec, Object *world_main_decision) {
  century::common::math::Vec2d stop_pt;
  double stop_heading = 0.0;
  auto decision = world_main_decision->add_decision();
  decision->set_type(Decision::STOP);
  if (main_decision.has_not_ready()) {
    // The car is not ready!
    // Use the current ADC pose since it is better not to self-drive.
    stop_pt.set_x(world_.auto_driving_car().position_x());
    stop_pt.set_y(world_.auto_driving_car().position_y());
    stop_heading = world_.auto_driving_car().heading();
    decision->set_stopreason(Decision::STOP_REASON_NOT_READY);
  } else if (main_decision.has_estop()) {
    // Emergency stop.
    // Use the current ADC pose since it is better to stop immediately.
    stop_pt.set_x(world_.auto_driving_car().position_x());
    stop_pt.set_y(world_.auto_driving_car().position_y());
    stop_heading = world_.auto_driving_car().heading();
    decision->set_stopreason(Decision::STOP_REASON_EMERGENCY);
    world_.mutable_auto_driving_car()->set_current_signal("EMERGENCY");
  } else {
    // Normal stop.
    const century::planning::MainStop &stop = main_decision.stop();
    stop_pt.set_x(stop.stop_point().x() + map_service_->GetXOffset());
    stop_pt.set_y(stop.stop_point().y() + map_service_->GetYOffset());
    stop_heading = stop.stop_heading();
    if (stop.has_reason_code()) {
      SetStopReason(stop.reason_code(), decision);
    }
  }

  decision->set_position_x(stop_pt.x());
  decision->set_position_y(stop_pt.y());
  decision->set_heading(stop_heading);
}

bool SimulationWorldService::LocateMarker(
    const century::planning::ObjectDecisionType &decision,
    Decision *world_decision) {
  century::common::PointENU fence_point;
  double heading;
  if (decision.has_stop() && decision.stop().has_stop_point()) {
    world_decision->set_type(Decision_Type_STOP);
    fence_point = decision.stop().stop_point();
    heading = decision.stop().stop_heading();
  } else if (decision.has_follow() && decision.follow().has_fence_point()) {
    world_decision->set_type(Decision_Type_FOLLOW);
    fence_point = decision.follow().fence_point();
    heading = decision.follow().fence_heading();
  } else if (decision.has_yield() && decision.yield().has_fence_point()) {
    world_decision->set_type(Decision_Type_YIELD);
    fence_point = decision.yield().fence_point();
    heading = decision.yield().fence_heading();
  } else if (decision.has_overtake() && decision.overtake().has_fence_point()) {
    world_decision->set_type(Decision_Type_OVERTAKE);
    fence_point = decision.overtake().fence_point();
    heading = decision.overtake().fence_heading();
  } else {
    return false;
  }

  world_decision->set_position_x(fence_point.x() + map_service_->GetXOffset());
  world_decision->set_position_y(fence_point.y() + map_service_->GetYOffset());
  world_decision->set_heading(heading);
  return true;
}

void SimulationWorldService::FindNudgeRegion(
    const century::planning::ObjectDecisionType &decision,
    const Object &world_obj, Decision *world_decision) {
  std::vector<century::common::math::Vec2d> points;
  for (auto &polygon_pt : world_obj.polygon_point()) {
    points.emplace_back(polygon_pt.x(), polygon_pt.y());
  }
  const century::common::math::Polygon2d obj_polygon(points);
  const century::common::math::Polygon2d &nudge_polygon =
      obj_polygon.ExpandByDistance(std::fabs(decision.nudge().distance_l()));
  const std::vector<century::common::math::Vec2d> &nudge_points =
      nudge_polygon.points();
  for (auto &nudge_pt : nudge_points) {
    PolygonPoint *poly_pt = world_decision->add_polygon_point();
    poly_pt->set_x(nudge_pt.x());
    poly_pt->set_y(nudge_pt.y());
  }
  world_decision->set_type(Decision_Type_NUDGE);
}

void SimulationWorldService::UpdateDecision(const DecisionResult &decision_res,
                                            double header_time) {
  // Update turn signal.
  UpdateTurnSignal(decision_res.vehicle_signal(),
                   world_.mutable_auto_driving_car());

  const auto &main_decision = decision_res.main_decision();

  // Update speed limit.
  if (main_decision.target_lane_size() > 0) {
    world_.set_speed_limit(main_decision.target_lane(0).speed_limit());
  }

  // Update relevant main stop with reason and change lane.
  world_.clear_main_decision();
  Object *world_main_decision = world_.mutable_main_decision();
  if (main_decision.has_not_ready() || main_decision.has_estop() ||
      main_decision.has_stop()) {
    UpdateMainStopDecision(main_decision, header_time, world_main_decision);
  }
  if (main_decision.has_stop()) {
    UpdateMainChangeLaneDecision(main_decision.stop(), world_main_decision);
  } else if (main_decision.has_cruise()) {
    UpdateMainChangeLaneDecision(main_decision.cruise(), world_main_decision);
  }
  if (world_main_decision->decision_size() > 0) {
    // set default position
    const auto &adc = world_.auto_driving_car();
    world_main_decision->set_position_x(adc.position_x());
    world_main_decision->set_position_y(adc.position_y());
    world_main_decision->set_heading(adc.heading());
    world_main_decision->set_timestamp_sec(header_time);
  }

  // Update obstacle decision.
  for (const auto &obj_decision : decision_res.object_decision().decision()) {
    if (obj_decision.has_perception_id()) {
      int id = obj_decision.perception_id();
      Object &world_obj = obj_map_[std::to_string(id)];
      if (!world_obj.has_type()) {
        world_obj.set_type(Object_Type_VIRTUAL);
        ADEBUG << id << " is not a current perception object";
      }

      for (const auto &decision : obj_decision.object_decision()) {
        Decision *world_decision = world_obj.add_decision();
        world_decision->set_type(Decision_Type_IGNORE);
        if (decision.has_stop() || decision.has_follow() ||
            decision.has_yield() || decision.has_overtake()) {
          if (!LocateMarker(decision, world_decision)) {
            AWARN << "No decision marker position found for object id=" << id;
            continue;
          }
          if (decision.has_stop()) {
            // flag yielded obstacles
            for (auto obstacle_id : decision.stop().wait_for_obstacle()) {
              const std::vector<std::string> id_segments =
                  absl::StrSplit(obstacle_id, '_');
              if (id_segments.size() > 0) {
                obj_map_[id_segments[0]].set_yielded_obstacle(true);
              }
            }
          }
        } else if (decision.has_nudge()) {
          if (world_obj.polygon_point().empty()) {
            if (world_obj.type() == Object_Type_VIRTUAL) {
              AWARN << "No current perception object with id=" << id
                    << " for nudge decision";
            } else {
              AWARN << "No polygon points found for object id=" << id;
            }
            continue;
          }
          FindNudgeRegion(decision, world_obj, world_decision);
        }
      }

      world_obj.set_timestamp_sec(
          std::max(world_obj.timestamp_sec(), header_time));
    }
  }
}

void SimulationWorldService::DownsamplePath(const common::Path &path,
                                            common::Path *downsampled_path) {
  auto sampled_indices = DownsampleByAngle(path.path_point(), kAngleThreshold);

  downsampled_path->set_name(path.name());
  for (const size_t index : sampled_indices) {
    *downsampled_path->add_path_point() =
        path.path_point(static_cast<int>(index));
  }
}

void SimulationWorldService::UpdatePlanningData(const PlanningData &data) {
  auto *planning_data = world_.mutable_planning_data();

  size_t max_interval = 10;

  // Update scenario
  if (data.has_scenario()) {
    planning_data->mutable_scenario()->CopyFrom(data.scenario());
  }

  // Update init point
  if (data.has_init_point()) {
    auto &planning_path_point = data.init_point().path_point();
    auto *world_obj_path_point =
        planning_data->mutable_init_point()->mutable_path_point();
    world_obj_path_point->set_x(planning_path_point.x() +
                                map_service_->GetXOffset());
    world_obj_path_point->set_y(planning_path_point.y() +
                                map_service_->GetYOffset());
    world_obj_path_point->set_theta(planning_path_point.theta());
  }

  // Update Chart
  planning_data->mutable_chart()->CopyFrom(data.chart());

  // Update SL Frame
  planning_data->mutable_sl_frame()->CopyFrom(data.sl_frame());

  // Update DP path
  if (data.has_dp_poly_graph()) {
    planning_data->mutable_dp_poly_graph()->CopyFrom(data.dp_poly_graph());
  }

  // Update ST Graph
  planning_data->clear_st_graph();
  for (auto &graph : data.st_graph()) {
    auto *st_graph = planning_data->add_st_graph();
    st_graph->set_name(graph.name());
    st_graph->mutable_boundary()->CopyFrom(graph.boundary());
    if (graph.has_kernel_cruise_ref()) {
      st_graph->mutable_kernel_cruise_ref()->CopyFrom(
          graph.kernel_cruise_ref());
    }
    if (graph.has_kernel_follow_ref()) {
      st_graph->mutable_kernel_follow_ref()->CopyFrom(
          graph.kernel_follow_ref());
    }
    if (graph.has_speed_constraint()) {
      st_graph->mutable_speed_constraint()->CopyFrom(graph.speed_constraint());
    }

    // downsample speed_profile and speed_limit
    // The x-axis range is always [-10, 200], downsample to ~200 points but skip
    // max 10 points
    size_t profile_downsample_interval =
        std::max(1, (graph.speed_profile_size() / 200));
    profile_downsample_interval =
        std::min(profile_downsample_interval, max_interval);
    DownsampleSpeedPointsByInterval(graph.speed_profile(),
                                    profile_downsample_interval,
                                    st_graph->mutable_speed_profile());

    size_t limit_downsample_interval =
        std::max(1, (graph.speed_limit_size() / 200));
    limit_downsample_interval =
        std::min(limit_downsample_interval, max_interval);
    DownsampleSpeedPointsByInterval(graph.speed_limit(),
                                    limit_downsample_interval,
                                    st_graph->mutable_speed_limit());
  }

  // Update Speed Plan
  planning_data->clear_speed_plan();
  for (auto &plan : data.speed_plan()) {
    if (plan.speed_point_size() > 0) {
      auto *downsampled_plan = planning_data->add_speed_plan();
      downsampled_plan->set_name(plan.name());

      // Downsample the speed plan for frontend display.
      // The x-axis range is always [-2, 10], downsample to ~80 points
      size_t interval = std::max(1, (plan.speed_point_size() / 80));
      interval = std::min(interval, max_interval);
      DownsampleSpeedPointsByInterval(plan.speed_point(), interval,
                                      downsampled_plan->mutable_speed_point());
    }
  }

  // Update path
  planning_data->clear_path();
  for (auto &path : data.path()) {
    DownsamplePath(path, planning_data->add_path());
  }

  // Update pull over status
  planning_data->clear_pull_over();
  if (data.has_pull_over()) {
    planning_data->mutable_pull_over()->CopyFrom(data.pull_over());
  }

  // Update planning signal
  world_.clear_traffic_signal();
  if (data.has_signal_light() && data.signal_light().signal_size() > 0) {
    TrafficLight::Color current_signal = TrafficLight::UNKNOWN;
    int green_light_count = 0;

    for (auto &signal : data.signal_light().signal()) {
      switch (signal.color()) {
        case TrafficLight::RED:
        case TrafficLight::YELLOW:
        case TrafficLight::BLACK:
          current_signal = signal.color();
          break;
        case TrafficLight::GREEN:
          green_light_count++;
          break;
        default:
          break;
      }
    }

    if (green_light_count == data.signal_light().signal_size()) {
      current_signal = TrafficLight::GREEN;
    }

    world_.mutable_traffic_signal()->set_current_signal(
        TrafficLight_Color_Name(current_signal));
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const ADCTrajectory &trajectory) {
  const double header_time = trajectory.header().timestamp_sec();

  UpdatePlanningTrajectory(trajectory);

  UpdateRSSInfo(trajectory);

  UpdateDecision(trajectory.decision(), header_time);

  UpdatePlanningData(trajectory.debug().planning_data());

  Latency latency;
  latency.set_timestamp_sec(header_time);
  latency.set_total_time_ms(trajectory.latency_stats().total_time_ms());
  (*world_.mutable_latency())["planning"] = latency;
}

void SimulationWorldService::CreatePredictionTrajectory(
    const PredictionObstacle &obstacle, Object *world_object) {
  for (const auto &traj : obstacle.trajectory()) {
    Prediction *prediction = world_object->add_prediction();
    prediction->set_probability(traj.probability());

    std::vector<PathPoint> points;
    for (const auto &point : traj.trajectory_point()) {
      points.push_back(point.path_point());
    }
    auto sampled_indices = DownsampleByAngle(points, kAngleThreshold);

    for (auto index : sampled_indices) {
      const auto &point = points[index];
      PolygonPoint *world_point = prediction->add_predicted_trajectory();
      world_point->set_x(point.x() + map_service_->GetXOffset());
      world_point->set_y(point.y() + map_service_->GetYOffset());

      const TrajectoryPoint &traj_point = traj.trajectory_point(index);
      if (traj_point.has_gaussian_info()) {
        const century::common::GaussianInfo &gaussian =
            traj_point.gaussian_info();

        auto *ellipse = world_point->mutable_gaussian_info();
        ellipse->set_ellipse_a(gaussian.ellipse_a());
        ellipse->set_ellipse_b(gaussian.ellipse_b());
        ellipse->set_theta_a(gaussian.theta_a());
      }
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const PredictionObstacles &obstacles) {
  for (const auto &obstacle : obstacles.prediction_obstacle()) {
    // Note: There's a perfect one-to-one mapping between the perception
    // obstacles and prediction obstacles within the same frame. Creating a new
    // world object here is only possible when we happen to be processing a
    // perception and prediction message from two frames.
    auto &world_obj = CreateWorldObjectIfAbsent(obstacle.perception_obstacle());

    // Add prediction trajectory to the object.
    CreatePredictionTrajectory(obstacle, &world_obj);

    // Add prediction priority
    if (obstacle.has_priority()) {
      world_obj.mutable_obstacle_priority()->CopyFrom(obstacle.priority());
    }

    // Add prediction interactive tag
    if (obstacle.has_interactive_tag()) {
      world_obj.mutable_interactive_tag()->CopyFrom(obstacle.interactive_tag());
    }

    world_obj.set_timestamp_sec(
        std::max(obstacle.timestamp(), world_obj.timestamp_sec()));

    world_obj.set_is_aeb_type("false");
  }
}


template <>
void SimulationWorldService::UpdateSimulationWorld(
    const RoutingResponses &routing_responses) {
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(route_paths_mutex_);
    if (world_.has_routing_time() &&
        world_.routing_time() >= routing_responses.header().timestamp_sec()) {
      // This routing response has been processed.
      return;
    }
  }
  route_paths_.clear();
  std::vector<std::vector<RoutePath>> route_path_reponses;
  for (const auto &routing_response : routing_responses.routing_response()) {
    std::vector<RoutePath> route_paths;
    std::vector<Path> paths;
    if (!map_service_->GetPathsFromRouting(routing_response, &paths)) {
      return;
    }

    world_.clear_route_path();

    for (const Path &path : paths) {
      // Downsample the path points for frontend display.
      auto sampled_indices =
          DownsampleByAngle(path.path_points(), kAngleThreshold);

      route_paths.emplace_back();
      RoutePath *route_path = &route_paths.back();
      for (const size_t index : sampled_indices) {
        const auto &path_point = path.path_points()[index];
        PolygonPoint *route_point = route_path->add_point();
        route_point->set_x(path_point.x() + map_service_->GetXOffset());
        route_point->set_y(path_point.y() + map_service_->GetYOffset());
      }

      // Populate route path
      if (FLAGS_sim_world_with_routing_path) {
        std::cout << "world_ route path size : " << world_.route_path().size() << std::endl;
        auto *new_path = world_.add_route_path();
        *new_path = *route_path;
      }
    }
    route_path_reponses.emplace_back(route_paths);
  }
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(route_paths_mutex_);
    std::swap(route_path_reponses, route_paths_);
    world_.set_routing_time(routing_responses.header().timestamp_sec());
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const RoutingResponse &routing_response) {
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(route_paths_mutex_);
    if (world_.has_routing_time() &&
        world_.routing_time() >= routing_response.header().timestamp_sec()) {
      // This routing response has been processed.
      return;
    }
  }
  route_paths_.clear();
  std::vector<Path> paths;
  if (!map_service_->GetPathsFromRouting(routing_response, &paths)) {
    return;
  }

  world_.clear_route_path();

  std::vector<RoutePath> route_paths;
  for (const Path &path : paths) {
    // Downsample the path points for frontend display.
    auto sampled_indices =
        DownsampleByAngle(path.path_points(), kAngleThreshold);

    route_paths.emplace_back();
    RoutePath *route_path = &route_paths.back();
    for (const size_t index : sampled_indices) {
      const auto &path_point = path.path_points()[index];
      PolygonPoint *route_point = route_path->add_point();
      route_point->set_x(path_point.x() + map_service_->GetXOffset());
      route_point->set_y(path_point.y() + map_service_->GetYOffset());
    }

    // Populate route path
    if (FLAGS_sim_world_with_routing_path) {
      auto *new_path = world_.add_route_path();
      *new_path = *route_path;
    }
  }
  {
    boost::unique_lock<boost::shared_mutex> writer_lock(route_paths_mutex_);
    route_paths_.emplace_back(route_paths);
    // std::swap(route_paths, route_paths_);
    world_.set_routing_time(routing_response.header().timestamp_sec());
  }
}

Json SimulationWorldService::GetRoutePathAsJson() const {
  Json response;
  response["routePath"] = Json::array();
  std::vector<std::vector<RoutePath>> route_paths;
  {
    boost::shared_lock<boost::shared_mutex> reader_lock(route_paths_mutex_);
    response["routingTime"] = world_.routing_time();
    route_paths = route_paths_;
  }
  size_t idx = 0;
  for (const auto &route_path : route_paths) {
    for (const auto &paths : route_path) {
      Json path;
      path["point"] = Json::array();
      for (const auto &route_point : paths.point()) {
        path["point"].push_back({{"x", route_point.x()},
                                 {"y", route_point.y()},
                                 {"z", route_point.z()}});
      }
      response["routePath"][idx]["id"] = idx;
      response["routePath"][idx]["Path"].push_back(path);
    }
    idx++;
  }
  std::cout << response.dump() << std::endl;
  return response;
}

void SimulationWorldService::ReadRoutingFromFile(
    const std::string &routing_response_file) {
  auto routing_response = std::make_shared<RoutingResponse>();
  if (!GetProtoFromFile(routing_response_file, routing_response.get())) {
    AWARN << "Unable to read routing response from file: "
          << routing_response_file;
    return;
  }
  AINFO << "Loaded routing from " << routing_response_file;

  sleep(1);  // Wait to make sure the connection has been established before
             // publishing.
  routing_response_writer_->Write(routing_response);
  AINFO << "Published RoutingResponse read from file.";
}

void SimulationWorldService::SendRoutingResponse(size_t idx) {
  auto message = routing_responses_reader_->GetLatestObserved();
  routing_response_writer_->Write(message->routing_response(idx));
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const ControlCommand &control_command) {
  auto *control_data = world_.mutable_control_data();
  const double header_time = control_command.header().timestamp_sec();
  control_data->set_timestamp_sec(header_time);

  Latency latency;
  latency.set_timestamp_sec(header_time);
  latency.set_total_time_ms(control_command.latency_stats().total_time_ms());
  (*world_.mutable_latency())["control"] = latency;

  if (control_command.has_debug()) {
    auto &debug = control_command.debug();
    if (debug.has_simple_lon_debug() && debug.has_simple_lat_debug()) {
      auto &simple_lon = debug.simple_lon_debug();
      if (simple_lon.has_station_error()) {
        control_data->set_station_error(simple_lon.station_error());
      }
      auto &simple_lat = debug.simple_lat_debug();
      if (simple_lat.has_heading_error()) {
        control_data->set_heading_error(simple_lat.heading_error());
      }
      if (simple_lat.has_lateral_error()) {
        control_data->set_lateral_error(simple_lat.lateral_error());
      }
      if (simple_lat.has_current_target_point()) {
        control_data->mutable_current_target_point()->CopyFrom(
            simple_lat.current_target_point());
      }
    } else if (debug.has_simple_mpc_debug()) {
      auto &simple_mpc = debug.simple_mpc_debug();
      if (simple_mpc.has_station_error()) {
        control_data->set_station_error(simple_mpc.station_error());
      }
      if (simple_mpc.has_heading_error()) {
        control_data->set_heading_error(simple_mpc.heading_error());
      }
      if (simple_mpc.has_lateral_error()) {
        control_data->set_lateral_error(simple_mpc.lateral_error());
      }
    }
  }

  // Update AEB action status.
  if (control_command.has_aeb_enable()) {
    world_.set_aeb_action(
        common::util::IsFloatEqual(control_command.aeb_enable(), 1.0));
  } else {
    world_.set_aeb_action(false);
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const NavigationInfo &navigation_info) {
  world_.clear_navigation_path();
  for (auto &navigation_path : navigation_info.navigation_path()) {
    if (navigation_path.has_path()) {
      DownsamplePath(navigation_path.path(), world_.add_navigation_path());
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(const MapMsg &map_msg) {
  if (map_msg.has_hdmap()) {
    relative_map_.CopyFrom(map_msg.hdmap());
    for (int i = 0; i < relative_map_.lane_size(); ++i) {
      auto *lane = relative_map_.mutable_lane(i);
      lane->clear_left_sample();
      lane->clear_right_sample();
      lane->clear_left_road_sample();
      lane->clear_right_road_sample();

      DownsampleCurve(lane->mutable_central_curve());
      DownsampleCurve(lane->mutable_left_boundary()->mutable_curve());
      DownsampleCurve(lane->mutable_right_boundary()->mutable_curve());
    }
  }
}

template <>
void SimulationWorldService::UpdateSimulationWorld(
    const MonitorMessage &monitor_msg) {
  const int updated_size = std::min(monitor_msg.item_size(),
                                    SimulationWorldService::kMaxMonitorItems);
  // Save the latest messages at the end of the history.
  for (int idx = 0; idx < updated_size; ++idx) {
    auto *notification = world_.add_notification();
    notification->mutable_item()->CopyFrom(monitor_msg.item(idx));
    notification->set_timestamp_sec(monitor_msg.header().timestamp_sec());
  }

  int remove_size =
      world_.notification_size() - SimulationWorldService::kMaxMonitorItems;
  if (remove_size > 0) {
    auto *notifications = world_.mutable_notification();
    notifications->erase(notifications->begin(),
                         notifications->begin() + remove_size);
  }
}

void SimulationWorldService::UpdateMonitorMessages() {
  std::list<std::shared_ptr<MonitorMessage>> monitor_msgs;
  {
    std::unique_lock<std::mutex> lock(monitor_msgs_mutex_);
    monitor_msgs = monitor_msgs_;
    monitor_msgs_.clear();
  }

  for (const auto &monitor_msg : monitor_msgs) {
    UpdateSimulationWorld(*monitor_msg);
  }
}

void SimulationWorldService::DumpMessages() {
  DumpMessageFromReader(chassis_reader_.get());
  DumpMessageFromReader(prediction_obstacle_reader_.get());
  DumpMessageFromReader(routing_request_reader_.get());
  DumpMessageFromReader(routing_response_reader_.get());
  DumpMessageFromReader(routing_responses_reader_.get());
  DumpMessageFromReader(localization_reader_.get());
  DumpMessageFromReader(planning_reader_.get());
  DumpMessageFromReader(control_command_reader_.get());
  DumpMessageFromReader(perception_obstacle_reader_.get());
  DumpMessageFromReader(perception_traffic_light_reader_.get());
  DumpMessageFromReader(relative_map_reader_.get());
  DumpMessageFromReader(navigation_reader_.get());
  DumpMessageFromReader(task_reader_.get());
}

void SimulationWorldService::PublishNavigationInfo(
    const std::shared_ptr<NavigationInfo> &navigation_info) {
  FillHeader(FLAGS_dreamview_module_name, navigation_info.get());
  navigation_writer_->Write(navigation_info);
}

void SimulationWorldService::PublishRoutingRequest(
    const std::shared_ptr<RoutingRequest> &routing_request) {
  FillHeader(FLAGS_dreamview_module_name, routing_request.get());
  routing_request_writer_->Write(routing_request);
}

void SimulationWorldService::PublishRoutingRequestWithoutHeader(
    const std::shared_ptr<RoutingRequest> &routing_request) {
  routing_request_writer_->Write(routing_request);
}

std::shared_ptr<RoutingRequest>
SimulationWorldService::GetLatestRoutingRequest() {
  if (routing_request_reader_) {
    routing_request_reader_->Observe();
    if (!routing_request_reader_->Empty()) {
      return routing_request_reader_->GetLatestObserved();
    }
  }

  return nullptr;
}

void SimulationWorldService::PublishRoutingRequest(const Json& json) {
  auto routing_request = std::make_shared<RoutingRequest>();
  localization_reader_->Observe();

  auto localization = localization_reader_->GetLatestObserved();
  if (localization == nullptr) {
    AERROR << "Failed to get latest localization.";
    return;
  }

  auto begin = routing_request->add_waypoint();
  begin->mutable_pose()->set_x(localization->pose().position().x());
  begin->mutable_pose()->set_y(localization->pose().position().y());
  begin->set_heading(localization->pose().heading());

  auto end = routing_request->add_waypoint();
  end->CopyFrom(*begin);

  century::routing::TaskType type;
  century::routing::TaskType_Parse(json["task_type"], &type);

  routing_request->set_task_type(type);
  routing_request->set_tiny_adjustment_distance(json["distance"].get<double>());
  FillHeader(FLAGS_dreamview_module_name, routing_request.get());
  routing_request_writer_->Write(routing_request);
}


void SimulationWorldService::PublishBorrowResponse(
    const std::shared_ptr<BorrowResponse> &borrow_response){
  FillHeader(FLAGS_dreamview_module_name, borrow_response.get());
  planning_borrow_response_writer_->Write(borrow_response);
}

void SimulationWorldService::PublishPassStackerResponse(
    const std::shared_ptr<century::planning::PassStackerResponse>
        &pass_stacker_response) {
  FillHeader(FLAGS_dreamview_module_name, pass_stacker_response.get());
  planning_pass_stacker_writer_->Write(pass_stacker_response);
}

void SimulationWorldService::UpdateBorrowRequest() {
  if (!planning_reader_) {
    return;
  }
  planning_reader_->Observe();
  auto msg = planning_reader_->GetLatestObserved();

  if (!msg) {
    return;
  }

  world_.clear_borrow_request();
  world_.clear_block_obs_id();

  if ((!msg->has_borrow_request() || !msg->has_block_obs_id()) && (!msg->has_pass_stacker_request())) {
    return;
  }

  if (msg->has_pass_stacker_request() && msg->pass_stacker_request().request_for_pass_stacker()) {
    world_.set_borrow_request(msg->pass_stacker_request().request_for_pass_stacker());
    world_.set_block_obs_id("stacker"+msg->pass_stacker_request().stacker_id());
  } else {
    world_.set_borrow_request(msg->borrow_request());
    world_.set_block_obs_id(msg->block_obs_id());
  }
}

void SimulationWorldService::PublishImmediatelyArrived(const std::shared_ptr<McloudInfo> mcloud_info) {
  // static bool is_immediately = false;
  FillHeader(FLAGS_dreamview_module_name, mcloud_info.get());
  cloud_info_writer_->Write(mcloud_info);
//   if (is_immediately) {
//     is_immediately = false;
//     std::this_thread::sleep_for(std::chrono::milliseconds(200));
//   }
//   is_immediately = true;
//   std::thread([&is_immediately, mcloud_info, this](){
//     while (is_immediately) {
//       this->cloud_info_writer_->Write(mcloud_info);
//       std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
}

void SimulationWorldService::PublishTask(const std::shared_ptr<Task> &task) {
  FillHeader(FLAGS_dreamview_module_name, task.get());
  task_writer_->Write(task);
}

Json SimulationWorldService::ModifyFasAebInfo(const Json& fast_aeb_info) {
  Json result;
  std::cout << "ModifyFasAebInfo: " << fast_aeb_info.dump() << std::endl;
  auto fas_aeb_switch = fast_aeb_info["status"];
  auto password = fast_aeb_info["password"];
  if (!fas_aeb_backend_client_->ServiceIsReady()) {
    result["result"] = false;
    result["message"] = "serice is not ready";
    return result;
  }
  auto request = std::make_shared<century::fas_aeb_backend::Request>();
  request->set_type(century::fas_aeb_backend::Request::MODIFY_CONFIG);
  request->mutable_modify_config()->set_key("switch_fas_aeb");
  request->mutable_modify_config()->set_value(fas_aeb_switch ? "true" : "false");
  request->mutable_modify_config()->set_password(password == nullptr ? "" : password.get<std::string>());
  request->mutable_modify_config()->set_is_short(fast_aeb_info["isShort"]);
  request->mutable_modify_config()->set_password_longth(fast_aeb_info["passwordLong"]);
  auto ret = fas_aeb_backend_client_->SendRequest(request);
  
  if (nullptr == ret) {
    result["result"] = false;
    result["message"] = "serice is not ready";
    return result;
  }
  std::cout << "result: " << ret->result() << std::endl;
  std::cout << "ModifyFasAebInfo ret: " << ret->message() << std::endl;
  result["result"] = ret->result();
  result["message"] = ret->message();

  return result;
}

Json SimulationWorldService::PublishBarrierCommand(
    const Json& barrier_command_info) {
  Json result;
  result["status"] = false;

  if (!barrier_command_info.contains("enabled") ||
      !barrier_command_info["enabled"].is_boolean()) {
    result["error"] = "Barrier command enable flag is invalid.";
    return result;
  }

  const bool enabled = barrier_command_info["enabled"].get<bool>();
  result["enabled"] = enabled;

  if (!enabled) {
    WriteBarrierCommand(false);
    barrier_command_active_ = false;
    barrier_command_type_ = BarrierCommand::COMMAND_TYPE_UNKNOWN;
    barrier_command_stop_time_sec_ = 0.0;
    barrier_command_last_publish_time_sec_ = 0.0;
    result["status"] = true;
    result["command"] = "STOP";
    return result;
  }

  if (!barrier_command_info.contains("password") ||
      !barrier_command_info["password"].is_string()) {
    result["error"] = "Barrier command password is invalid.";
    return result;
  }

  if (barrier_command_info["password"].get<std::string>() !=
      FLAGS_barrier_command_password) {
    result["error"] = "Barrier command password is incorrect.";
    return result;
  }

  if (!barrier_command_info.contains("command") ||
      !barrier_command_info["command"].is_string()) {
    result["error"] = "Barrier command type is invalid.";
    return result;
  }

  BarrierCommand::CommandType command =
      BarrierCommand::COMMAND_TYPE_UNKNOWN;
  if (!BarrierCommand_CommandType_Parse(
          barrier_command_info["command"].get<std::string>(), &command) ||
      BarrierCommand::COMMAND_TYPE_UNKNOWN == command) {
    result["error"] = "Barrier command type is not supported.";
    return result;
  }

  barrier_command_active_ = true;
  barrier_command_type_ = command;
  barrier_command_last_publish_time_sec_ = 0.0;
  const double now_sec = century::cyber::Time::Now().ToSecond();
  if (FLAGS_barrier_command_auto_stop_seconds > 0.0) {
    barrier_command_stop_time_sec_ =
        now_sec + FLAGS_barrier_command_auto_stop_seconds;
  } else {
    barrier_command_stop_time_sec_ = 0.0;
  }
  WriteBarrierCommand(true, command);

  result["status"] = true;
  result["command"] = BarrierCommand_CommandType_Name(command);
  return result;
}

void SimulationWorldService::UpdateBarrierCommand() {
  if (!barrier_command_active_ ||
      BarrierCommand::COMMAND_TYPE_UNKNOWN == barrier_command_type_) {
    return;
  }

  const double now_sec = century::cyber::Time::Now().ToSecond();
  if (barrier_command_stop_time_sec_ > 0.0 &&
      now_sec >= barrier_command_stop_time_sec_) {
    WriteBarrierCommand(false);
    barrier_command_active_ = false;
    barrier_command_type_ = BarrierCommand::COMMAND_TYPE_UNKNOWN;
    barrier_command_stop_time_sec_ = 0.0;
    barrier_command_last_publish_time_sec_ = 0.0;
    PublishMonitorMessage(MonitorMessageItem::INFO,
                          "Barrier command auto stopped by timeout.");
    return;
  }

  const double publish_interval_sec =
      std::max(FLAGS_barrier_command_publish_interval_ms, 0) * 0.001;
  if (publish_interval_sec > 0.0 &&
      barrier_command_last_publish_time_sec_ > 0.0 &&
      now_sec - barrier_command_last_publish_time_sec_ < publish_interval_sec) {
    return;
  }

  WriteBarrierCommand(true, barrier_command_type_);
}

void SimulationWorldService::WriteBarrierCommand(
    bool enabled, BarrierCommand::CommandType command_type) {
  auto barrier_command = std::make_shared<BarrierCommand>();
  barrier_command->set_enabled(enabled);
  barrier_command->set_command(command_type);
  FillHeader(FLAGS_dreamview_module_name, barrier_command.get());
  barrier_command_writer_->Write(barrier_command);
  barrier_command_last_publish_time_sec_ =
      century::cyber::Time::Now().ToSecond();
}

void SimulationWorldService::ChangeBckMusicSwitch(const Json& music_info) {
  if (!music_info.contains("status") ||
      !music_info["status"].is_boolean()) {
    AWARN << "music_info does not contain background_music_switch";
    return;
  }
  
  auto music_msg = std::make_shared<century::dreamview::BackgroundMusic>();
  const auto now_sec = century::cyber::Time::Now().ToSecond();
  const bool switch_on = music_info["status"].get<bool>();
  music_msg->set_background_music_switch(switch_on);
  music_msg->set_last_modify_time(now_sec);
  FillHeader(FLAGS_dreamview_module_name, music_msg.get());

  bck_music_writer_->Write(music_msg);
}

void SimulationWorldService::PublishMonitorMessage(
    century::common::monitor::MonitorMessageItem::LogLevel log_level,
    const std::string &msg) {
  monitor_logger_buffer_.AddMonitorMsgItem(log_level, msg);
  monitor_logger_buffer_.Publish();
}

void SimulationWorldService::MaybePrintNearestObstacleWhenNearDestination(
    const LocalizationEstimate &localization) {
  if (!has_destination_position_) {
    return;
  }

  constexpr double kDestinationDistanceThreshold = 16.0;
  const auto &current_position = localization.pose().position();
  const double distance_to_destination =
      std::hypot(current_position.x() - destination_position_.x(),
                 current_position.y() - destination_position_.y());

  AINFO << "[Dreamview][NearDest] vehicle=(" << current_position.x() << ","
        << current_position.y() << ") dest=(" << destination_position_.x()
        << "," << destination_position_.y()
        << ") dist_to_dest=" << distance_to_destination
        << " threshold=" << kDestinationDistanceThreshold;

  if (distance_to_destination > kDestinationDistanceThreshold) {
    return;
  }

  AINFO << "[Dreamview][NearDest] Triggered. Starting stacker reroute logic.";

  // Only mark as consumed to prevent re-triggering, but keep destination_position_
  // intact so FindNearestPerceptionObstacle / FindNearestStackerFromStackersInfo
  // can use the correct coordinates below.
  has_destination_position_ = false;

  // ── Step 1: PerceptionObstacle ────────────────────────────────────────────
  const PerceptionObstacle *nearest_obstacle = nullptr;
  {
    std::shared_ptr<PerceptionObstacles> latest_obstacles;
    const PerceptionObstacles *obstacles = nullptr;
    if (nullptr == perception_obstacle_reader_) {
      AWARN << "[Dreamview][NearDest] perception_obstacle_reader_ is nullptr.";
    } else if (perception_obstacle_reader_->Empty()) {
      AINFO << "[Dreamview][NearDest] perception_obstacle_reader_ is empty.";
    } else {
      latest_obstacles = perception_obstacle_reader_->GetLatestObserved();
      obstacles = latest_obstacles.get();
    }
    if (nullptr == obstacles) {
      AINFO << "[Dreamview][NearDest] No PerceptionObstacles message available.";
    } else if (obstacles->perception_obstacle().empty()) {
      AINFO << "[Dreamview][NearDest] PerceptionObstacles message has 0 obstacles.";
    } else {
      AINFO << "[Dreamview][NearDest] PerceptionObstacles has "
            << obstacles->perception_obstacle_size() << " obstacles. Searching for STACKER/FORKLIFT/WHEELCRANE.";
      nearest_obstacle =
          FindNearestPerceptionObstacle(destination_position_, *obstacles);
      if (nullptr == nearest_obstacle) {
        AINFO << "[Dreamview][NearDest] PerceptionObstacle: no STACKER/FORKLIFT/WHEELCRANE type found.";
      } else {
        AINFO << "[Dreamview][NearDest] PerceptionObstacle: nearest id=" << nearest_obstacle->id()
              << " type=" << nearest_obstacle->type()
              << " pos=(" << nearest_obstacle->position().x() << ","
              << nearest_obstacle->position().y() << ")";
      }
    }
  }

  // ── Step 2: StackersInfo ──────────────────────────────────────────────────
  const century::planning::StackerInfo *nearest_stacker_from_info =
      FindNearestStackerFromStackersInfo(destination_position_);
  if (nullptr == nearest_stacker_from_info) {
    AWARN << "[Dreamview][NearDest] StackersInfo: no nearest stacker found.";
  } else {
    AINFO << "[Dreamview][NearDest] StackersInfo: nearest id=" << nearest_stacker_from_info->stacker_id()
          << " pos=(" << nearest_stacker_from_info->stacker_point().x() << ","
          << nearest_stacker_from_info->stacker_point().y()
          << ") heading=" << nearest_stacker_from_info->stacker_point().heading();
  }

  // ── Step 3: Decide final obstacle position ────────────────────────────────
  bool has_obstacle = false;
  double obstacle_x = 0.0;
  double obstacle_y = 0.0;
  double obstacle_heading = 0.0;
  bool has_obstacle_heading = false;
  std::string obstacle_id;

  if (nullptr != nearest_stacker_from_info && nullptr != nearest_obstacle) {
    double dist_diff = std::hypot(
        nearest_stacker_from_info->stacker_point().x() - nearest_obstacle->position().x(),
        nearest_stacker_from_info->stacker_point().y() - nearest_obstacle->position().y());
    AINFO << "[Dreamview][NearDest] StackersInfo vs PerceptionObstacle dist_diff=" << dist_diff << "m";
    if (dist_diff > 10.0) {
      AWARN << "[Dreamview][NearDest] dist_diff > 10m. Discarding StackersInfo, using PerceptionObstacle id="
            << nearest_obstacle->id()
            << " pos=(" << nearest_obstacle->position().x() << ","
            << nearest_obstacle->position().y() << ")";
      has_obstacle = true;
      obstacle_x = nearest_obstacle->position().x();
      obstacle_y = nearest_obstacle->position().y();
      obstacle_id = std::to_string(nearest_obstacle->id());
    } else {
      AINFO << "[Dreamview][NearDest] dist_diff <= 10m. Using StackersInfo id="
            << nearest_stacker_from_info->stacker_id()
            << " pos=(" << nearest_stacker_from_info->stacker_point().x() << ","
            << nearest_stacker_from_info->stacker_point().y() << ")";
      has_obstacle = true;
      obstacle_x = nearest_stacker_from_info->stacker_point().x();
      obstacle_y = nearest_stacker_from_info->stacker_point().y();
      obstacle_heading = nearest_stacker_from_info->stacker_point().heading();
      has_obstacle_heading = true;
      obstacle_id = nearest_stacker_from_info->stacker_id();
    }
  } else if (nullptr != nearest_stacker_from_info) {
    AINFO << "[Dreamview][NearDest] Only StackersInfo available. Using id="
          << nearest_stacker_from_info->stacker_id()
          << " pos=(" << nearest_stacker_from_info->stacker_point().x() << ","
          << nearest_stacker_from_info->stacker_point().y() << ")";
    has_obstacle = true;
    obstacle_x = nearest_stacker_from_info->stacker_point().x();
    obstacle_y = nearest_stacker_from_info->stacker_point().y();
    obstacle_heading = nearest_stacker_from_info->stacker_point().heading();
    has_obstacle_heading = true;
    obstacle_id = nearest_stacker_from_info->stacker_id();
  } else if (nullptr != nearest_obstacle) {
    AINFO << "[Dreamview][NearDest] Only PerceptionObstacle available. Using id="
          << nearest_obstacle->id()
          << " pos=(" << nearest_obstacle->position().x() << ","
          << nearest_obstacle->position().y() << ")";
    has_obstacle = true;
    obstacle_x = nearest_obstacle->position().x();
    obstacle_y = nearest_obstacle->position().y();
    obstacle_id = std::to_string(nearest_obstacle->id());
  } else {
    AWARN << "[Dreamview][NearDest] No obstacle source available. "
          << "Check: (1) StackersInfo topic publishing? "
          << "(2) PerceptionObstacle has STACKER/FORKLIFT/WHEELCRANE type?";
  }

  // ── Step 5: Reroute ───────────────────────────────────────────────────────
  if (!last_external_routing_request_) {
    AWARN << "[Dreamview][NearDest] last_external_routing_request_ is null. Cannot reroute.";
    return;
  }
  if (last_external_routing_request_->waypoint_size() < 2) {
    AWARN << "[Dreamview][NearDest] last_external_routing_request_ has only "
          << last_external_routing_request_->waypoint_size()
          << " waypoints (need >= 2). Cannot reroute.";
    return;
  }

  AINFO << "[Dreamview][NearDest] Building reroute request. has_obstacle=" << has_obstacle
        << " obstacle_id=" << obstacle_id
        << " obstacle_pos=(" << obstacle_x << "," << obstacle_y << ")"
        << " has_obstacle_heading=" << has_obstacle_heading
        << " obstacle_heading=" << obstacle_heading
        << " original_waypoint_count=" << last_external_routing_request_->waypoint_size();

  auto reroute_request =
      std::make_shared<RoutingRequest>(*last_external_routing_request_);

  // Replace waypoint[0] with current vehicle position.
  auto* first_wp = reroute_request->mutable_waypoint(0);
  first_wp->mutable_pose()->set_x(current_position.x());
  first_wp->mutable_pose()->set_y(current_position.y());
  if (localization.pose().has_heading()) {
    first_wp->set_heading(localization.pose().heading());
  }
  first_wp->clear_id();

  if (has_obstacle) {
    auto* waypoints = reroute_request->mutable_waypoint();
    century::routing::LaneWaypoint obstacle_wp;
    obstacle_wp.mutable_pose()->set_x(obstacle_x);
    obstacle_wp.mutable_pose()->set_y(obstacle_y);
    if (has_obstacle_heading) {
      obstacle_wp.set_heading(obstacle_heading);
    }
    // Insert at index 1.
    waypoints->Add();
    for (int i = waypoints->size() - 1; i > 1; --i) {
      waypoints->SwapElements(i, i - 1);
    }
    *waypoints->Mutable(1) = obstacle_wp;
    AINFO << "[Dreamview][NearDest] Rerouting with obstacle id=" << obstacle_id
          << " inserted at waypoint[1]. Total waypoints=" << reroute_request->waypoint_size();
  } else {
    AWARN << "[Dreamview][NearDest] Rerouting WITHOUT obstacle insertion.";
  }
  {
    auto last_idx = reroute_request->mutable_waypoint()->size() - 1;
    auto* last_wp = reroute_request->mutable_waypoint(last_idx);
    last_wp->mutable_pose()->set_x(destination_position_.x());
    last_wp->mutable_pose()->set_y(destination_position_.y());
    last_wp->clear_id();
    AWARN << "[Dreamview][NearDest] No obstacle found. Overwriting last waypoint["
          << last_idx << "] with destination_position_=("
          << destination_position_.x() << "," << destination_position_.y() << ").";
  }
  reroute_request->set_dreamview_task_type(century::routing::TaskType::SEARCH_REACH_STACKER);
  routing_request_writer_->Write(*reroute_request);
  AINFO << "[Dreamview][NearDest] RoutingRequest published.";
}

void SimulationWorldService::SetDestinationPosition(
    const PointENU &destination_position) {
  destination_position_ = destination_position;
  has_destination_position_ =
      !std::isnan(destination_position.x()) &&
      !std::isnan(destination_position.y());
}

void SimulationWorldService::ClearDestinationPosition() {
  destination_position_.Clear();
  has_destination_position_ = false;
}
const PerceptionObstacle *SimulationWorldService::FindNearestPerceptionObstacle(
    const PointENU &current_position,
    const PerceptionObstacles &obstacles) const {
  const PerceptionObstacle *nearest_obstacle = nullptr;
  double min_distance = std::numeric_limits<double>::max();
  
  if (obstacles.perception_obstacle().empty()) {
    AWARN << "[Dreamview] PerceptionObstacles message is empty.";
    return nearest_obstacle;
  }
  
  AINFO << "[Dreamview] Checking " << obstacles.perception_obstacle_size() << " Perception obstacles.";
  
  for (const auto &obstacle : obstacles.perception_obstacle()) {
    if (PerceptionObstacle::STACKER == obstacle.type() ||
        PerceptionObstacle::FORKLIFT_STACKER == obstacle.type() ||
        PerceptionObstacle::WHEELCRANE == obstacle.type()) {
      const double distance =
          std::hypot(obstacle.position().x() - current_position.x(),
                     obstacle.position().y() - current_position.y());
      if (distance < min_distance) {
        min_distance = distance;
        nearest_obstacle = &obstacle;
      }
    }
  }
  
  if (nullptr != nearest_obstacle) {
    AINFO << "[Dreamview] FindNearestPerceptionObstacle selected obstacle id " 
          << nearest_obstacle->id() << " of type " << nearest_obstacle->type() << " at dist " << min_distance;
  } else {
    AINFO << "[Dreamview] FindNearestPerceptionObstacle found no valid stackers near destination.";
  }
  
  return nearest_obstacle;
}

const century::planning::StackerInfo *
SimulationWorldService::FindNearestStackerFromStackersInfo(
    const century::common::PointENU &destination_position) const {
  const century::planning::StackerInfo *nearest_stacker = nullptr;
  double min_distance = std::numeric_limits<double>::max();

  std::shared_ptr<century::planning::StackersInfo> stackers_info;
  if (nullptr == stackers_info_reader_) {
    AERROR << "[Dreamview] stackers_info_reader_ is nullptr!";
  } else if (stackers_info_reader_->Empty()) {
    AINFO << "[Dreamview] stackers_info_reader_ is Empty()";
  } else {
    stackers_info_reader_->Observe();
    stackers_info = stackers_info_reader_->GetLatestObserved();
  }

  if (nullptr != stackers_info) {
    AINFO << "[Dreamview] Found " << stackers_info->stacker_info_size() << " stackers in StackersInfo";
    for (const auto &stacker : stackers_info->stacker_info()) {
      const double distance =
          std::hypot(stacker.stacker_point().x() - destination_position.x(),
                     stacker.stacker_point().y() - destination_position.y());
      if (distance < min_distance) {
        min_distance = distance;
        nearest_stacker = &stacker;
      }
    }
  } else {
    AWARN << "[Dreamview] stackers_info is nullptr from reader.";
  }

  if (nullptr != nearest_stacker) {
    AINFO << "[Dreamview] FindNearestStackerFromStackersInfo selected stacker " 
          << nearest_stacker->stacker_id() << " at dist " << min_distance;
  }
  return nearest_stacker;
}

}  // namespace dreamview
}  // namespace century
