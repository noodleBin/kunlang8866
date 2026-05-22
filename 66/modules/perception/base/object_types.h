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

#include <map>
#include <string>

namespace century {
namespace perception {
namespace base {


// UNKNOWN = 0;
// UNKNOWN_MOVABLE = 1;
// UNKNOWN_UNMOVABLE = 2;
// PEDESTRIAN = 3;  // Pedestrian, usually determined by moving behavior.
// BICYCLE = 4;     // bike, motor bike
// VEHICLE = 5;     // Passenger car or truck.
// WIDE40FOOT = 6;
// NARROW20FOOT = 7;
// CONE = 8;
// STACKER = 9;
// FORKLIFT_STACKER=10;

// @brief general object type
enum class ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  NARROW20FOOT = 6,
  DUMMY = 7,
  CONE = 8,
  STACKER = 9,
  FORKLIFT_STACKER = 10,
  WHEELCRANE = 11,
  MAX_OBJECT_TYPE = 12,
};

// @brief internal object type used by lidar perception
enum class InternalObjectType {
  INT_BACKGROUND = 0,
  INT_SMALLMOT = 1,
  INT_PEDESTRIAN = 2,
  INT_NONMOT = 3,
  INT_BIGMOT = 4,
  INT_UNKNOWN = 5,
  INT_MAX_OBJECT_TYPE = 6,
};

// @brief internal object type used by visual perception
enum class VisualObjectType {
  CAR,
  VAN,
  BUS,
  TRUCK,
  BICYCLE,
  TRICYCLE,
  PEDESTRIAN,
  TRAFFICCONE,
  UNKNOWN_MOVABLE,
  UNKNOWN_UNMOVABLE,
  MAX_OBJECT_TYPE,
};

// @brief fine-grained object types
enum class ObjectSubType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  CAR = 4,
  IGV_FULL = 5,
  TRUCK = 6,
  TRAILER_EMPTY = 7,
  TRAILER_FULL = 8,
  IGV_EMPTY = 9,
  CRANE = 10,
  OTHER_VEHICLE = 11,
  CONE = 12,
  CONTAINER_FORKLIFT = 13,
  FORKLIFT = 14,
  LORRY = 15,
  CONSTRUCTION_VEHICLE = 16,
  WHEELCRANE = 17,
  VAN = 18,
  BUS = 19,
  CYCLIST = 20,
  MOTORCYCLIST = 21,
  TRICYCLIST = 22,
  TRAFFICCONE = 23,
  SMALLMOT = 24,
  BIGMOT = 25,
  NONMOT = 26,
  // Attention(lordon): matrix dimensions in type_change_cost file need to be
  // the same
  MAX_OBJECT_TYPE = 27,
};

// 'Pedestrian',
// 'Car', 
// 'IGV-Full', 
// 'Truck', 
// 'Trailer-Empty',
// 'Trailer-Full', 
// 'IGV-Empty',
// 'Crane',
// 'OtherVehicle', 
// 'Cone',
// 'ContainerForklift', 
// 'Forklift', 
// 'Lorry', 
// 'ConstructionVehicle'

// @brief motion state
enum class MotionState {
  UNKNOWN = 0,
  MOVING = 1,
  STATIONARY = 2,
};

/**
 * Landmark types and mapping
 */
enum class VisualLandmarkType {
  RoadArrow,
  RoadText,
  TrafficSign,
  TrafficLight,
  MAX_LANDMARK_TYPE,
};

const std::map<VisualLandmarkType, std::string> kVisualLandmarkType2NameMap = {
    {VisualLandmarkType::RoadArrow, "RoadArrow"},
    {VisualLandmarkType::RoadText, "RoadText"},
    {VisualLandmarkType::TrafficSign, "TrafficSign"},
    {VisualLandmarkType::TrafficLight, "TrafficLight"},
};

const std::map<std::string, base::VisualLandmarkType>
    kVisualLandmarkName2TypeMap = {
        {"RoadArrow", VisualLandmarkType::RoadArrow},
        {"RoadText", VisualLandmarkType::RoadText},
        {"TrafficSign", VisualLandmarkType::TrafficSign},
        {"TrafficLight", VisualLandmarkType::TrafficLight},
};

/**
 * ObjectType mapping
 */
const std::map<ObjectType, std::string> kObjectType2NameMap = {
    {ObjectType::UNKNOWN, "UNKNOWN"},
    {ObjectType::UNKNOWN_MOVABLE, "UNKNOWN_MOVABLE"},
    {ObjectType::UNKNOWN_UNMOVABLE, "UNKNOWN_UNMOVABLE"},
    {ObjectType::PEDESTRIAN, "PEDESTRIAN"},
    {ObjectType::BICYCLE, "BICYCLE"},
    {ObjectType::VEHICLE, "VEHICLE"},
    {ObjectType::MAX_OBJECT_TYPE, "MAX_OBJECT_TYPE"}};

const std::map<std::string, ObjectType> kObjectName2TypeMap = {
    {"UNKNOWN", ObjectType::UNKNOWN},
    {"UNKNOWN_MOVABLE", ObjectType::UNKNOWN_MOVABLE},
    {"UNKNOWN_UNMOVABLE", ObjectType::UNKNOWN_UNMOVABLE},
    {"PEDESTRIAN", ObjectType::PEDESTRIAN},
    {"BICYCLE", ObjectType::BICYCLE},
    {"VEHICLE", ObjectType::VEHICLE},
    {"MAX_OBJECT_TYPE", ObjectType::MAX_OBJECT_TYPE}};

/**
 * VisualObjectType mapping
 */
const std::map<VisualObjectType, ObjectType> kVisualTypeMap = {
    {VisualObjectType::CAR, ObjectType::VEHICLE},
    {VisualObjectType::VAN, ObjectType::VEHICLE},
    {VisualObjectType::BUS, ObjectType::VEHICLE},
    {VisualObjectType::TRUCK, ObjectType::VEHICLE},
    {VisualObjectType::BICYCLE, ObjectType::BICYCLE},
    {VisualObjectType::TRICYCLE, ObjectType::BICYCLE},
    {VisualObjectType::PEDESTRIAN, ObjectType::PEDESTRIAN},
    {VisualObjectType::TRAFFICCONE, ObjectType::UNKNOWN_UNMOVABLE},
    {VisualObjectType::UNKNOWN_MOVABLE, ObjectType::UNKNOWN_MOVABLE},
    {VisualObjectType::UNKNOWN_UNMOVABLE, ObjectType::UNKNOWN_UNMOVABLE},
    {VisualObjectType::MAX_OBJECT_TYPE, ObjectType::MAX_OBJECT_TYPE},
};

const std::map<VisualObjectType, std::string> kVisualType2NameMap = {
    {VisualObjectType::CAR, "CAR"},
    {VisualObjectType::VAN, "VAN"},
    {VisualObjectType::BUS, "BUS"},
    {VisualObjectType::TRUCK, "TRUCK"},
    {VisualObjectType::BICYCLE, "BICYCLE"},
    {VisualObjectType::TRICYCLE, "TRICYCLE"},
    {VisualObjectType::PEDESTRIAN, "PEDESTRIAN"},
    {VisualObjectType::TRAFFICCONE, "TRAFFICCONE"},
    {VisualObjectType::UNKNOWN_MOVABLE, "UNKNOWN_MOVABLE"},
    {VisualObjectType::UNKNOWN_UNMOVABLE, "UNKNOWN_UNMOVABLE"},
    {VisualObjectType::MAX_OBJECT_TYPE, "MAX_OBJECT_TYPE"},
};

const std::map<std::string, base::VisualObjectType> kVisualName2TypeMap = {
    {"CAR", VisualObjectType::CAR},
    {"VAN", VisualObjectType::VAN},
    {"BUS", VisualObjectType::BUS},
    {"TRUCK", VisualObjectType::TRUCK},
    {"BICYCLE", VisualObjectType::BICYCLE},
    {"TRICYCLE", VisualObjectType::TRICYCLE},
    {"PEDESTRIAN", VisualObjectType::PEDESTRIAN},
    {"TRAFFICCONE", VisualObjectType::TRAFFICCONE},
    {"UNKNOWN_MOVABLE", VisualObjectType::UNKNOWN_MOVABLE},
    {"UNKNOWN_UNMOVABLE", VisualObjectType::UNKNOWN_UNMOVABLE},
    {"MAX_OBJECT_TYPE", VisualObjectType::MAX_OBJECT_TYPE},
};

/**
 * ObjectSubType mapping
 */
// const std::map<ObjectSubType, ObjectType> kSubType2TypeMap = {
//     {ObjectSubType::UNKNOWN, ObjectType::UNKNOWN},
//     {ObjectSubType::UNKNOWN_MOVABLE, ObjectType::UNKNOWN_MOVABLE},
//     {ObjectSubType::UNKNOWN_UNMOVABLE, ObjectType::UNKNOWN_UNMOVABLE},
//     {ObjectSubType::CAR, ObjectType::VEHICLE},
//     {ObjectSubType::VAN, ObjectType::VEHICLE},
//     {ObjectSubType::TRUCK, ObjectType::VEHICLE},
//     {ObjectSubType::BUS, ObjectType::VEHICLE},
//     {ObjectSubType::CYCLIST, ObjectType::BICYCLE},
//     {ObjectSubType::MOTORCYCLIST, ObjectType::BICYCLE},
//     {ObjectSubType::TRICYCLIST, ObjectType::BICYCLE},
//     {ObjectSubType::PEDESTRIAN, ObjectType::PEDESTRIAN},
//     {ObjectSubType::TRAFFICCONE, ObjectType::UNKNOWN_UNMOVABLE},
//     {ObjectSubType::MAX_OBJECT_TYPE, ObjectType::MAX_OBJECT_TYPE},
// };

// const std::map<ObjectSubType, std::string> kSubType2NameMap = {
//     {ObjectSubType::UNKNOWN, "UNKNOWN"},
//     {ObjectSubType::UNKNOWN_MOVABLE, "UNKNOWN_MOVABLE"},
//     {ObjectSubType::UNKNOWN_UNMOVABLE, "UNKNOWN_UNMOVABLE"},
//     {ObjectSubType::CAR, "CAR"},
//     {ObjectSubType::VAN, "VAN"},
//     {ObjectSubType::TRUCK, "TRUCK"},
//     {ObjectSubType::BUS, "BUS"},
//     {ObjectSubType::CYCLIST, "CYCLIST"},
//     {ObjectSubType::MOTORCYCLIST, "MOTORCYCLIST"},
//     {ObjectSubType::TRICYCLIST, "TRICYCLIST"},
//     {ObjectSubType::PEDESTRIAN, "PEDESTRIAN"},
//     {ObjectSubType::TRAFFICCONE, "TRAFFICCONE"},
//     {ObjectSubType::MAX_OBJECT_TYPE, "MAX_OBJECT_TYPE"},
// };

const std::map<ObjectSubType, ObjectType> kSubType2TypeMap = {
  {ObjectSubType::UNKNOWN, ObjectType::UNKNOWN},
  {ObjectSubType::UNKNOWN_MOVABLE, ObjectType::UNKNOWN_MOVABLE},
  {ObjectSubType::UNKNOWN_UNMOVABLE, ObjectType::UNKNOWN_UNMOVABLE},
  {ObjectSubType::PEDESTRIAN, ObjectType::PEDESTRIAN},
  {ObjectSubType::CAR, ObjectType::VEHICLE},
  {ObjectSubType::IGV_FULL, ObjectType::VEHICLE},
  {ObjectSubType::TRUCK, ObjectType::VEHICLE},
  {ObjectSubType::TRAILER_EMPTY, ObjectType::VEHICLE},
  {ObjectSubType::TRAILER_FULL, ObjectType::VEHICLE},
  {ObjectSubType::IGV_EMPTY, ObjectType::VEHICLE},
  {ObjectSubType::CRANE, ObjectType::STACKER},
  {ObjectSubType::OTHER_VEHICLE, ObjectType::VEHICLE},
  {ObjectSubType::CONE, ObjectType::CONE},
  {ObjectSubType::CONTAINER_FORKLIFT, ObjectType::FORKLIFT_STACKER},
  {ObjectSubType::FORKLIFT, ObjectType::VEHICLE},
  {ObjectSubType::LORRY, ObjectType::VEHICLE},
  {ObjectSubType::CONSTRUCTION_VEHICLE, ObjectType::VEHICLE},
  {ObjectSubType::WHEELCRANE, ObjectType::WHEELCRANE},
  {ObjectSubType::VAN, ObjectType::VEHICLE},
  {ObjectSubType::BUS, ObjectType::VEHICLE},
  {ObjectSubType::CYCLIST, ObjectType::BICYCLE},
  {ObjectSubType::MOTORCYCLIST, ObjectType::BICYCLE},
  {ObjectSubType::TRICYCLIST, ObjectType::BICYCLE},
  {ObjectSubType::TRAFFICCONE, ObjectType::CONE},
  {ObjectSubType::SMALLMOT, ObjectType::VEHICLE},
  {ObjectSubType::BIGMOT, ObjectType::VEHICLE},
  {ObjectSubType::NONMOT, ObjectType::BICYCLE},
  {ObjectSubType::MAX_OBJECT_TYPE, ObjectType::MAX_OBJECT_TYPE},
};

const std::map<ObjectSubType, std::string> kSubType2NameMap = {
  {ObjectSubType::UNKNOWN, "UNKNOWN"},
  {ObjectSubType::UNKNOWN_MOVABLE, "UNKNOWN_MOVABLE"},
  {ObjectSubType::UNKNOWN_UNMOVABLE, "UNKNOWN_UNMOVABLE"},
  {ObjectSubType::PEDESTRIAN, "PEDESTRIAN"},
  {ObjectSubType::CAR, "CAR"},
  {ObjectSubType::IGV_FULL, "IGV_FULL"},
  {ObjectSubType::TRUCK, "TRUCK"},
  {ObjectSubType::TRAILER_EMPTY, "TRAILER_EMPTY"},
  {ObjectSubType::TRAILER_FULL, "TRAILER_FULL"},
  {ObjectSubType::IGV_EMPTY, "IGV_EMPTY"},
  {ObjectSubType::CRANE, "CRANE"},
  {ObjectSubType::OTHER_VEHICLE, "OTHER_VEHICLE"},
  {ObjectSubType::CONE, "CONE"},
  {ObjectSubType::CONTAINER_FORKLIFT, "CONTAINER_FORKLIFT"},
  {ObjectSubType::FORKLIFT, "FORKLIFT"},
  {ObjectSubType::LORRY, "LORRY"},
  {ObjectSubType::CONSTRUCTION_VEHICLE, "CONSTRUCTION_VEHICLE"},
  {ObjectSubType::WHEELCRANE, "WHEELCRANE"},
  {ObjectSubType::VAN, "VAN"},
  {ObjectSubType::BUS, "BUS"},
  {ObjectSubType::CYCLIST, "CYCLIST"},
  {ObjectSubType::MOTORCYCLIST, "MOTORCYCLIST"},
  {ObjectSubType::TRICYCLIST, "TRICYCLIST"},
  {ObjectSubType::TRAFFICCONE, "TRAFFICCONE"},
  {ObjectSubType::MAX_OBJECT_TYPE, "MAX_OBJECT_TYPE"},
};


const std::map<std::string, ObjectSubType> kName2SubTypeMap = {
    {"UNKNOWN", ObjectSubType::UNKNOWN},
    {"UNKNOWN_MOVABLE", ObjectSubType::UNKNOWN_MOVABLE},
    {"UNKNOWN_UNMOVABLE", ObjectSubType::UNKNOWN_UNMOVABLE},
    {"CAR", ObjectSubType::CAR},
    {"VAN", ObjectSubType::VAN},
    {"TRUCK", ObjectSubType::TRUCK},
    {"BUS", ObjectSubType::BUS},
    {"CYCLIST", ObjectSubType::CYCLIST},
    {"MOTORCYCLIST", ObjectSubType::MOTORCYCLIST},
    {"TRICYCLIST", ObjectSubType::TRICYCLIST},
    {"PEDESTRIAN", ObjectSubType::PEDESTRIAN},
    {"TRAFFICCONE", ObjectSubType::TRAFFICCONE},
    {"MAX_OBJECT_TYPE", ObjectSubType::MAX_OBJECT_TYPE},
};

}  // namespace base
}  // namespace perception
}  // namespace century
