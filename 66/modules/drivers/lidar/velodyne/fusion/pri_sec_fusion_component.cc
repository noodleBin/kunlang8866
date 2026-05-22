/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/velodyne/fusion/pri_sec_fusion_component.h"

#include <memory>
#include <thread>
#ifdef __aarch64__
#include <arm_neon.h>
#endif

namespace century {
namespace drivers {
namespace velodyne {

using century::cyber::Time;

bool PriSecFusionComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AWARN << "Load config failed, config file" << ConfigFilePath();
    return false;
  }
  buffer_ptr_ = century::transform::Buffer::Instance();

  fusion_writer_ = node_->CreateWriter<PointCloud>(conf_.fusion_channel());

  for (const auto& channel : conf_.input_channel()) {
    auto reader = node_->CreateReader<PointCloud>(channel);
    readers_.emplace_back(reader);
  }

  try {
    point_cloud_pool_.reset(new CCObjectPool<PointCloud>(POOL_SIZE));
    point_cloud_pool_->ConstructAll();
    for (int i = 0; i < POOL_SIZE; i++) {
      auto point_cloud = point_cloud_pool_->GetObject();
      if (nullptr == point_cloud) {
        AERROR << "fail to getobject, i: " << i;
        return false;
      }
      point_cloud->mutable_point()->Reserve(500000);
    }
  } catch (const std::bad_alloc& e) {
    AERROR << e.what();
    return false;
  }

  return true;
}

bool PriSecFusionComponent::Proc(
    const std::shared_ptr<PointCloud>& point_cloud) {
  auto target = point_cloud_pool_->GetObject();
  if (nullptr == target) {
    target = std::make_shared<PointCloud>();
    target->mutable_point()->Reserve(500000);
  }
  target->CopyFrom(std::move(*point_cloud));
  auto fusion_readers = readers_;
  auto start_time = Time::Now().ToSecond();
  while ((Time::Now().ToSecond() - start_time) < conf_.wait_time_s() &&
         fusion_readers.size() > 0) {
    for (auto itr = fusion_readers.begin(); itr != fusion_readers.end();) {
      (*itr)->Observe();
      if (!(*itr)->Empty()) {
        auto source = (*itr)->GetLatestObserved();
        if (conf_.drop_expired_data() && IsExpired(target, source)) {
          ++itr;
        } else {
          Fusion(target, source);
          itr = fusion_readers.erase(itr);
        }
      } else {
        ++itr;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  fusion_writer_->Write(target);

  return true;
}

bool PriSecFusionComponent::IsExpired(
    const std::shared_ptr<PointCloud>& target,
    const std::shared_ptr<PointCloud>& source) {
  auto diff = target->measurement_time() - source->measurement_time();
  return diff * 1000 > conf_.max_interval_ms();
}

bool PriSecFusionComponent::QueryPoseAffine(const std::string& target_frame_id,
                                            const std::string& source_frame_id,
                                            Eigen::Affine3d* pose) {
  std::string err_string;
  if (!buffer_ptr_->canTransform(target_frame_id, source_frame_id,
                                 cyber::Time(0), 0.02f, &err_string)) {
    AERROR << "Can not find transform. "
           << "target_id:" << target_frame_id << " frame_id:" << source_frame_id
           << " Error info: " << err_string;
    return false;
  }
  century::transform::TransformStamped stamped_transform;
  try {
    stamped_transform = buffer_ptr_->lookupTransform(
        target_frame_id, source_frame_id, cyber::Time(0));
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  *pose =
      Eigen::Translation3d(stamped_transform.transform().translation().x(),
                           stamped_transform.transform().translation().y(),
                           stamped_transform.transform().translation().z()) *
      Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
                         stamped_transform.transform().rotation().qx(),
                         stamped_transform.transform().rotation().qy(),
                         stamped_transform.transform().rotation().qz());
  return true;
}

void PriSecFusionComponent::AppendPointCloud(
    std::shared_ptr<PointCloud> point_cloud,
    std::shared_ptr<PointCloud> point_cloud_add, const Eigen::Affine3d& pose) {
  if (std::isnan(pose(0, 0))) {
    for (auto& point : point_cloud_add->point()) {
      PointXYZIT* point_new = point_cloud->add_point();
      point_new->set_intensity(point.intensity());
      point_new->set_timestamp(point.timestamp());
      point_new->set_x(point.x());
      point_new->set_y(point.y());
      point_new->set_z(point.z());
    }
  } else {
    for (auto& point : point_cloud_add->point()) {
      if (std::isnan(point.x())) {
        PointXYZIT* point_new = point_cloud->add_point();
        point_new->set_intensity(point.intensity());
        point_new->set_timestamp(point.timestamp());
        point_new->set_x(point.x());
        point_new->set_y(point.y());
        point_new->set_z(point.z());
      } else {
        PointXYZIT* point_new = point_cloud->add_point();
        point_new->set_intensity(point.intensity());
        point_new->set_timestamp(point.timestamp());
        Eigen::Matrix<float, 3, 1> pt(point.x(), point.y(), point.z());
        point_new->set_x(static_cast<float>(
            pose(0, 0) * pt.coeffRef(0) + pose(0, 1) * pt.coeffRef(1) +
            pose(0, 2) * pt.coeffRef(2) + pose(0, 3)));
        point_new->set_y(static_cast<float>(
            pose(1, 0) * pt.coeffRef(0) + pose(1, 1) * pt.coeffRef(1) +
            pose(1, 2) * pt.coeffRef(2) + pose(1, 3)));
        point_new->set_z(static_cast<float>(
            pose(2, 0) * pt.coeffRef(0) + pose(2, 1) * pt.coeffRef(1) +
            pose(2, 2) * pt.coeffRef(2) + pose(2, 3)));
      }
    }
  }

  if (point_cloud->height() != 0) {
    int new_width = point_cloud->point_size() / point_cloud->height();
    point_cloud->set_width(new_width);
  } else {
    point_cloud->set_height(1);
    point_cloud->set_width(point_cloud->point_size());
  }
}

#ifdef __aarch64__ 
void PriSecFusionComponent::AppendPointCloudNeon(
  std::shared_ptr<PointCloud> point_cloud,
  std::shared_ptr<PointCloud> point_cloud_add,
  const Eigen::Affine3d& pose) {
  auto start_time = Time::Now().ToMicrosecond();  
  float m[16];
  for (int i = 0; i < 16; ++i) {
    m[i] = static_cast<float>(pose.matrix().data()[i]);
  }
  // 1. 直接获取列优先数据指针
  // 按行手动聚合，正确加载行向量
  // row0 = [m00, m01, m02, m03], row1 = [m10, m11, m12, m13], …
  float32x4_t row0 = { m[0],  m[4],  m[8],  m[12] };  // [m00, m01, m02, m03]
  float32x4_t row1 = { m[1],  m[5],  m[9],  m[13] };  // [m10, m11, m12, m13]
  float32x4_t row2 = { m[2],  m[6],  m[10], m[14] };  // [m20, m21, m22, m23]

  for (auto& point : point_cloud_add->point()) {
    PointXYZIT* pt_new = point_cloud->add_point();
    pt_new->set_intensity(point.intensity());
    pt_new->set_timestamp(point.timestamp());

    // 3. 直接用 initializer list 构造 [x, y, z, 1]
    const float32x4_t vec = {point.x(), point.y(), point.z(), 1.0f };

    // 4. 对每行做 vmul + vaddvq （水平累加）完成点积
    const float xf = vaddvq_f32( vmulq_f32(row0, vec) );
    const float yf = vaddvq_f32( vmulq_f32(row1, vec) );
    const float zf = vaddvq_f32( vmulq_f32(row2, vec) );

    pt_new->set_x(xf);
    pt_new->set_y(yf);
    pt_new->set_z(zf);
  }
  auto end_time = Time::Now().ToMicrosecond();
  AINFO << "neon fusion total takes: " << end_time - start_time;
  // 更新 width/height（与原逻辑一致）
  if (point_cloud->height() != 0) {
    int new_width = point_cloud->point_size() / point_cloud->height();
    point_cloud->set_width(new_width);
  } else {
    point_cloud->set_height(1);
    point_cloud->set_width(point_cloud->point_size());
  }
}
#endif

bool PriSecFusionComponent::Fusion(std::shared_ptr<PointCloud> target,
                                   std::shared_ptr<PointCloud> source) {
  Eigen::Affine3d pose;
  if (QueryPoseAffine(target->header().frame_id(), source->header().frame_id(),
                    &pose)) {
#ifdef __aarch64__                  
    AppendPointCloudNeon(target, source, pose);
#else
    AppendPointCloud(target, source, pose);                  
#endif
    return true;
  }
  return false;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace century
