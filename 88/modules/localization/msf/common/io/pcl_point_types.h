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
 * @file pcl_point_types.h
 * @brief The pcl types.
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace century {
namespace localization {
namespace msf {
namespace robosense {

// struct PointXYZIRT {
//   float x;
//   float y;
//   float z;
//   unsigned char intensity;
//   unsigned char ring;
//   double timestamp;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
// } EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

struct PointXYZIRT {
  union EIGEN_ALIGN16 {
    float data[6];  // 16 bytes aligned float array

    struct {
      float x, y, z;                // 3D coordinates，meters
      unsigned char intensity;
      unsigned char ring;
      double timestamp;
    };
  };

//   ADD_GET_ARRAY3F_MAP
  PCL_ADD_EIGEN_MAPS_POINT4D
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZIT {
  float x;
  float y;
  float z;
  unsigned char intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

struct PointXYZIRTd {
  double x;
  double y;
  double z;
  unsigned char intensity;
  unsigned char ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

struct PointXYZITd {
  double x;
  double y;
  double z;
  unsigned char intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

}  // namespace robosense
}  // namespace msf
}  // namespace localization
}  // namespace century

POINT_CLOUD_REGISTER_POINT_STRUCT(
    century::localization::msf::robosense::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity,
                                            intensity)(double, timestamp,
                                                       timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    century::localization::msf::robosense::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
        std::uint8_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    century::localization::msf::robosense::PointXYZITd,
    (double, x, x)(double, y, y)(double, z, z)(std::uint8_t, intensity,
                                               intensity)(double, timestamp,
                                                          timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    century::localization::msf::robosense::PointXYZIRTd,
    (double, x, x)(double, y, y)(double, z, z)(
        std::uint8_t, intensity, intensity)(std::uint8_t, ring,
                                            ring)(double, timestamp, timestamp))

namespace pcl
{
  template <>
  struct PointRepresentation<century::localization::msf::robosense::PointXYZIRT>: public PointRepresentation<PointXYZ>
  {
    PointRepresentation()
    {
      // point type has 6 dimensions and 6 fields
      nr_dimensions_ = 6;
      trivial_ = true;
    }
  };
}                                            
namespace century {
namespace loc {
using PointXYZIRT = century::localization::msf::robosense::PointXYZIRT;
using PointCloudXYZIRT = pcl::PointCloud<PointXYZIRT>;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
// lidar trans
template <typename PointT>
class CloudFrameT {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  typedef typename pcl::PointCloud<PointT> CloudTypeT;
  typedef typename std::shared_ptr<CloudTypeT> CloudPtrT;

  // Constructor for CloudFrameT class
  CloudFrameT() : cloud_ptr_(new CloudTypeT()) {}

  double timestamp_{0.0};
  double measuretime_{0.0};
  CloudPtrT cloud_ptr_{};
};
typedef CloudFrameT<PointXYZIRT> CloudFrame;

}  // namespace loc
}  // namespace century
