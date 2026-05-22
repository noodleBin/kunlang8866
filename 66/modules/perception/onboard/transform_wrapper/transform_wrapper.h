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

#include <deque>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "gflags/gflags.h"

#include "modules/transform/proto/transform.pb.h"

#include "modules/transform/buffer.h"

namespace century {
namespace perception {
namespace onboard {

using century::common::Quaternion;
using century::transform::TransformStamped;

using century::transform::Buffer;

DECLARE_string(obs_sensor2novatel_tf2_frame_id);
DECLARE_string(obs_novatel2world_tf2_frame_id);
DECLARE_string(obs_novatel2world_tf2_child_frame_id);
DECLARE_double(obs_tf2_buff_size);

struct StampedTransform {
  double timestamp = 0.0;  // in second
  Eigen::Translation3d translation;
  Eigen::Quaterniond rotation;
};

class TransformCache {
 public:
  TransformCache() = default;
  ~TransformCache() = default;

  void AddTransform(const StampedTransform& transform);
  bool QueryTransform(double timestamp, StampedTransform* transform,
                      double max_duration = 0.0);

  inline void SetCacheDuration(double duration) { cache_duration_ = duration; }

 protected:
  // in ascending order of time
  std::deque<StampedTransform> transforms_;
  double cache_duration_ = 1.0;
};

namespace {
void ConvertMatrixToArray(const Eigen::Matrix4d& matrix,
                          std::vector<double>* array) {
  const double* pointer = matrix.data();
  for (int i = 0; i < matrix.size(); ++i) {
    array->push_back(pointer[i]);
  }
}

template <typename Point>
void ConstructTransformationMatrix(const Quaternion& quaternion,
                                   const Point& translation,
                                   Eigen::Matrix4d* matrix) {
  matrix->setConstant(0);
  Eigen::Quaterniond q;
  q.x() = quaternion.qx();
  q.y() = quaternion.qy();
  q.z() = quaternion.qz();
  q.w() = quaternion.qw();
  matrix->block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  (*matrix)(0, 3) = translation.x();
  (*matrix)(1, 3) = translation.y();
  (*matrix)(2, 3) = translation.z();
  (*matrix)(3, 3) = 1;
}
}  // namespace

class TransformWrapper {
 public:
  TransformWrapper() {}
  ~TransformWrapper() = default;

  void Init(const std::string& sensor2novatel_tf2_child_frame_id);
  void Init(const std::string& sensor2novatel_tf2_frame_id,
            const std::string& sensor2novatel_tf2_child_frame_id,
            const std::string& novatel2world_tf2_frame_id,
            const std::string& novatel2world_tf2_child_frame_id);

  // Attention: must initialize TransformWrapper first
  bool GetSensor2worldTrans(double timestamp,
                            Eigen::Affine3d* sensor2world_trans,
                            Eigen::Affine3d* novatel2world_trans = nullptr);

  bool GetExtrinsics(Eigen::Affine3d* trans);

  // Attention: can be called without initlization
  bool GetTrans(double timestamp, Eigen::Affine3d* trans,
                const std::string& frame_id, const std::string& child_frame_id);

  bool GetExtrinsicsBySensorId(const std::string& from_sensor_id,
                               const std::string& to_sensor_id,
                               Eigen::Affine3d* trans);

  bool QueryStaticTF(const std::string& frame_id,
                     const std::string& child_frame_id,
                     Eigen::Matrix4d* matrix) {
    TransformStamped transform;
    if (tf2_buffer_->GetLatestStaticTF(frame_id, child_frame_id, &transform)) {
      ConstructTransformationMatrix(transform.transform().rotation(),
                                    transform.transform().translation(),
                                    matrix);
      return true;
    }
    return false;
  }

 protected:
  bool QueryTrans(double timestamp, StampedTransform* trans,
                  const std::string& frame_id,
                  const std::string& child_frame_id);

 private:
  bool inited_ = false;

  Buffer* tf2_buffer_ = Buffer::Instance();

  std::string sensor2novatel_tf2_frame_id_;
  std::string sensor2novatel_tf2_child_frame_id_;
  std::string novatel2world_tf2_frame_id_;
  std::string novatel2world_tf2_child_frame_id_;

  std::unique_ptr<Eigen::Affine3d> sensor2novatel_extrinsics_{nullptr};

  TransformCache transform_cache_;
};

}  // namespace onboard
}  // namespace perception
}  // namespace century
