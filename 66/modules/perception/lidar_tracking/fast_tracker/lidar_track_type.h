#pragma once
#include <memory>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <fstream>
#include <Eigen/Geometry>
namespace Track{

namespace ros_interface {  
  enum ObjectClass : uint32_t{
    UNKNOWN = 0,
    CAR = 1,
    TRUCK = 2,
    PEDESTRIAN = 3,
    BICYCLE = 4,
  } ;

  typedef enum {
    UNKNOWN_STATE = 0,
    STATIC = 1,
    DYNAMIC = 2,
  } ObjectDynamicState;


  typedef struct{
    uint32_t    seq;
    double      stamp{0.0};
    std::string frame_id;
  } Header;

  typedef struct {
    Header header;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float v_x;
    float v_y;
    float v_z;
    float a_x;
    float a_y;
    float a_z;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
  } Object3DState;

  typedef struct {
    float length;
    float width;
    float height;
  } ObjectDimension;

  typedef struct {
    uint32_t    id;
    float       confidence_score;
    ObjectClass classification;

    ObjectDimension dimension;        
    Object3DState   state;
  } DetectObject3D;

  typedef struct {        
    Header header;
    std::vector<DetectObject3D> object;
  } DetectObjects3D;

  typedef struct {
    uint32_t            id;
    ObjectClass         classification;
    ObjectDynamicState  dynamic_state;

    ObjectDimension dimension;
    Object3DState   state;
    Object3DState   state_covariance;
  } TrackObject;

  typedef struct {
    Header header;
    std::vector<TrackObject> object;
  } TrackObjects;
  
} // namespace ros_interface
}