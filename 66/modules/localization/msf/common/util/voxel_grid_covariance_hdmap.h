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

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <cmath>  // for std::isfinite
#include <limits>
#include <map>
#include <vector>

#include "pcl/common/common.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"

namespace century {
namespace localization {
namespace msf {

template <typename PointT>
class VoxelGridCovariance : public pcl::VoxelGrid<PointT> {
 public:
  enum LeafType { FEW, BAD, PLANE, LINE };

 protected:
  using pcl::VoxelGrid<PointT>::filter_name_;
  using pcl::VoxelGrid<PointT>::getClassName;
  using pcl::VoxelGrid<PointT>::input_;
  using pcl::VoxelGrid<PointT>::indices_;
  using pcl::VoxelGrid<PointT>::filter_limit_negative_;
  using pcl::VoxelGrid<PointT>::filter_limit_min_;
  using pcl::VoxelGrid<PointT>::filter_limit_max_;
  using pcl::VoxelGrid<PointT>::filter_field_name_;

  using pcl::VoxelGrid<PointT>::downsample_all_data_;
  using pcl::VoxelGrid<PointT>::leaf_layout_;
  using pcl::VoxelGrid<PointT>::save_leaf_layout_;
  using pcl::VoxelGrid<PointT>::leaf_size_;
  using pcl::VoxelGrid<PointT>::min_b_;
  using pcl::VoxelGrid<PointT>::max_b_;
  using pcl::VoxelGrid<PointT>::inverse_leaf_size_;
  using pcl::VoxelGrid<PointT>::div_b_;
  using pcl::VoxelGrid<PointT>::divb_mul_;

  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

 public:
  typedef boost::shared_ptr<pcl::VoxelGrid<PointT>> Ptr;
  typedef boost::shared_ptr<const pcl::VoxelGrid<PointT>> ConstPtr;

  struct Leaf {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Leaf()
        : nr_points_(0),
          mean_(Eigen::Vector3d::Zero()),
          centroid(),
          cov_(Eigen::Matrix3d::Identity()),
          icov_(Eigen::Matrix3d::Zero()),
          evecs_(Eigen::Matrix3d::Identity()),
          evals_(Eigen::Vector3d::Zero()) {}

    // Get the voxel covariance.
    Eigen::Matrix3d GetCov() const { return cov_; }

    // Get the inverse of the voxel covariance.
    Eigen::Matrix3d GetInverseCov() const { return icov_; }

    // Get the voxel centroid.
    Eigen::Vector3d GetMean() const { return mean_; }

    // Get the eigen vectors of the voxel covariance.
    Eigen::Matrix3d GetEvecs() const { return evecs_; }

    // Get the eigen values of the voxel covariance.
    Eigen::Vector3d GetEvals() const { return evals_; }

    // Get the number of points contained by this voxel.
    int GetPointCount() const { return nr_points_; }

    // Number of points contained by voxel.
    int nr_points_;

    // 3D voxel centroid.
    Eigen::Vector3d mean_;

    // voxel centroid.
    Eigen::VectorXf centroid;

    // Voxel covariance matrix.
    Eigen::Matrix3d cov_;

    // Inverse of voxel covariance matrix.
    Eigen::Matrix3d icov_;

    // Eigen vectors of voxel covariance matrix.
    Eigen::Matrix3d evecs_;

    // Eigen values of voxel covariance matrix.
    Eigen::Vector3d evals_;

    pcl::PointCloud<PointT> cloud_;
    LeafType type_;
  };
  // Pointer to VoxelGridCovariance leaf structure.
  typedef Leaf* LeafPtr;
  // Const pointer to VoxelGridCovariance leaf structure.
  typedef const Leaf* LeafConstPtr;

 public:
  VoxelGridCovariance()
      : searchable_(true),
        min_points_per_voxel_(6),
        min_covar_eigvalue_mult_(0.01),
        leaves_(),
        voxel_centroids_(),
        voxel_centroidsleaf_indices_(),
        kdtree_() {
    downsample_all_data_ = false;
    save_leaf_layout_ = false;
    leaf_size_.setZero();
    min_b_.setZero();
    max_b_.setZero();
    filter_name_ = "VoxelGridCovariance";
  }

  // Set the minimum number of points required for a cell.
  inline void SetMinPointPerVoxel(int min_points_per_voxel) {
    if (min_points_per_voxel > 2) {
      min_points_per_voxel_ = min_points_per_voxel;
    } else {
      PCL_WARN("%s, Covariance need 3 pts, set min_pt_per_vexel to 3",
               this->getClassName().c_str());
      min_points_per_voxel_ = 3;
    }
  }

  // Get the minimum number of points required for a cell to be used.
  inline int GetMinPointPerVoxel() { return min_points_per_voxel_; }

  // Set the minimum allowable ratio for eigenvalues
  inline void SetCovEigValueInflationRatio(double min_covar_eigvalue_mult) {
    min_covar_eigvalue_mult_ = min_covar_eigvalue_mult;
  }
  // Get the minimum allowable ratio
  inline double GetCovEigValueInflationRatio() {
    return min_covar_eigvalue_mult_;
  }

  // Filter cloud and initializes voxel structure.
  inline void Filter(PointCloudPtr output, bool searchable = false) {
    searchable_ = searchable;
    ApplyFilter(output);
    voxel_centroids_ = PointCloudPtr(new PointCloud(*output));
    if (searchable_ && voxel_centroids_->size() > 0) {
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }

  // Initializes voxel structure.
  inline void Filter(bool searchable = false) {
    searchable_ = searchable;
    voxel_centroids_ = PointCloudPtr(new PointCloud);
    ApplyFilter(voxel_centroids_);
    if (searchable_ && voxel_centroids_->size() > 0) {
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }

  // Get the voxel containing point p.
  inline LeafConstPtr GetLeaf(int index) {
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(index);
    if (leaf_iter != leaves_.end()) {
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return nullptr;
    }
  }

  // Get the voxel containing point p.
  inline LeafConstPtr GetLeaf(const PointT& p) {
    // Generate index associated with p
    int ijk0 = static_cast<int>(floor(p.x * inverse_leaf_size_[0]) - min_b_[0]);
    int ijk1 = static_cast<int>(floor(p.y * inverse_leaf_size_[1]) - min_b_[1]);
    int ijk2 = static_cast<int>(floor(p.z * inverse_leaf_size_[2]) - min_b_[2]);

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter != leaves_.end()) {
      // If such a leaf exists return the pointer to the leaf structure
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return nullptr;
    }
  }

  // Get the voxel containing point p.
  inline LeafConstPtr GetLeaf(const Eigen::Vector3f& p) {
    // Generate index associated with p
    int ijk0 =
        static_cast<int>(floor(p[0] * inverse_leaf_size_[0]) - min_b_[0]);
    int ijk1 =
        static_cast<int>(floor(p[1] * inverse_leaf_size_[1]) - min_b_[1]);
    int ijk2 =
        static_cast<int>(floor(p[2] * inverse_leaf_size_[2]) - min_b_[2]);

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter != leaves_.end()) {
      // If such a leaf exists return the pointer to the leaf structure
      LeafConstPtr ret(&(leaf_iter->second));
      return ret;
    } else {
      return nullptr;
    }
  }

  // Get the leaf structure map.
  inline std::map<size_t, Leaf>& GetLeaves() { return leaves_; }

 private:
  // Filter cloud and initializes voxel structure.
  void ApplyFilter(PointCloudPtr output) {
  }

  // Flag to determine if voxel structure is searchable. */
  bool searchable_;

  // Minimum points contained with in a voxel to allow it to be usable.
  int min_points_per_voxel_;

  // Minimum allowable ratio between eigenvalues.
  double min_covar_eigvalue_mult_;

  // Voxel structure containing all leaf nodes.
  std::map<size_t, Leaf> leaves_;

  /* Point cloud containing centroids of voxels
   * containing at least minimum number of points. */
  PointCloudPtr voxel_centroids_;

  /* Indices of leaf structurs associated with each point in
   * \ref _voxel_centroids (used for searching). */
  std::vector<int> voxel_centroidsleaf_indices_;

  // KdTree used for searching.
  pcl::KdTreeFLANN<PointT> kdtree_;
};

}  // namespace msf
}  // namespace localization
}  // namespace century
