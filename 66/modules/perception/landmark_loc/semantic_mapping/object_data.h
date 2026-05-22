//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once
#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace semantic_mapping {

enum class ObjectMapCellMode : uint32_t {
  Init = 0,
  Line,
  Sign,
  Number,
  ModeSize
};

enum class ObjectType : uint32_t {
  Num0 = 0,
  Num1,
  Num2,
  Num3,
  Num4,
  Num5,
  Num6,
  Num7,
  Num8,
  Num9,
  Slotline,
  Boundaryline,
  Arrow,
  Bump,
  ManholeCover,
  CircleCover,
  Init,
  ObjectSize
};
enum class ObjectClass : uint32_t {
  Number = 0,
  Slotline,
  Boundaryline,
  Arrow,
  Bump,
  ManholeCover,
  CircleCover,
  Init,
  ObjectSize
};

// ObjectType -> ObjectMapCellMode
const std::unordered_map<ObjectType, ObjectClass> objectType_to_class = {
  // std::unordered_map<uint32_t, uint32_t> objectType_to_objectMode = {
  {ObjectType::Num0, ObjectClass::Number},
  {ObjectType::Num1, ObjectClass::Number},
  {ObjectType::Num2, ObjectClass::Number},
  {ObjectType::Num3, ObjectClass::Number},
  {ObjectType::Num4, ObjectClass::Number},
  {ObjectType::Num5, ObjectClass::Number},    
  {ObjectType::Num6, ObjectClass::Number},
  {ObjectType::Num7, ObjectClass::Number},
  {ObjectType::Num8, ObjectClass::Number},
  {ObjectType::Num9, ObjectClass::Number},
  {ObjectType::Slotline, ObjectClass::Slotline},
  {ObjectType::Boundaryline, ObjectClass::Boundaryline},
  {ObjectType::Arrow, ObjectClass::Arrow},
  {ObjectType::Bump, ObjectClass::Bump},
  {ObjectType::ManholeCover, ObjectClass::ManholeCover},
  {ObjectType::CircleCover, ObjectClass::CircleCover},
  {ObjectType::Init, ObjectClass::Init}
};


// ObjectType -> ObjectMapCellMode
const std::unordered_map<ObjectType, ObjectMapCellMode> objectType_to_objectMode = {
  // std::unordered_map<uint32_t, uint32_t> objectType_to_objectMode = {
  {ObjectType::Num0, ObjectMapCellMode::Number},
  {ObjectType::Num1, ObjectMapCellMode::Number},
  {ObjectType::Num2, ObjectMapCellMode::Number},
  {ObjectType::Num3, ObjectMapCellMode::Number},
  {ObjectType::Num4, ObjectMapCellMode::Number},
  {ObjectType::Num5, ObjectMapCellMode::Number},    
  {ObjectType::Num6, ObjectMapCellMode::Number},
  {ObjectType::Num7, ObjectMapCellMode::Number},
  {ObjectType::Num8, ObjectMapCellMode::Number},
  {ObjectType::Num9, ObjectMapCellMode::Number},
  {ObjectType::Slotline, ObjectMapCellMode::Line},
  {ObjectType::Boundaryline, ObjectMapCellMode::Line},
  {ObjectType::Arrow, ObjectMapCellMode::Sign},
  {ObjectType::Bump, ObjectMapCellMode::Sign},
  {ObjectType::ManholeCover, ObjectMapCellMode::Sign},
  {ObjectType::CircleCover, ObjectMapCellMode::Sign},
  {ObjectType::Init, ObjectMapCellMode::Init}
};

const std::unordered_map<ObjectMapCellMode, cv::Vec3b> objectType_to_bgr = {
  {ObjectMapCellMode::Init, cv::Vec3b(128, 128, 128)}, // gray
  {ObjectMapCellMode::Number, cv::Vec3b(255, 0, 0)}, // blue
  {ObjectMapCellMode::Line, cv::Vec3b(0, 255, 0)}, // green
  {ObjectMapCellMode::Sign, cv::Vec3b(0, 0, 255)} // red
};

}


