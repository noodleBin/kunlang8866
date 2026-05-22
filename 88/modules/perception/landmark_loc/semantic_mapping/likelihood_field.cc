//
// Created by xiaxinrong on 2025/8/15.
//

#include "g2o_types.h"
#include "likelihood_filed.h"

#include <glog/logging.h>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/solvers/linear_solver_dense.h>

namespace semantic_mapping {

void LikelihoodField::SetTargetScan(PointCloudPtr scan) {
  target_ = scan;


  field_ = cv::Mat(map_param_.grid_size_[0], map_param_.grid_size_[1], CV_32F, 30.0);

  for (size_t i = 0; i < scan->points.size(); ++i) {

  double x =  scan->points[i].x / map_param_.resolution_ + map_param_.grid_center_[0];
  double y =  scan->points[i].y / map_param_.resolution_ + map_param_.grid_center_[1];

 
  for (auto& model_pt : model_) {
    int xx = int(x + model_pt.dx_);
    int yy = int(y + model_pt.dy_);
    if (xx >= 0 && xx < field_.cols && yy >= 0 && yy < field_.rows &&
    field_.at<float>(yy, xx) > model_pt.residual_) {
    field_.at<float>(yy, xx) = model_pt.residual_;
    }
  }
  }
}

void LikelihoodField::BuildModel() {
  const int range = 20;  
  for (int x = -range; x <= range; ++x) {
  for (int y = -range; y <= range; ++y) {
    model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
  }
  }
}

void LikelihoodField::SetSourceScan(PointCloudPtr scan) {
  source_ = scan;
}

cv::Mat LikelihoodField::GetFieldImage() {
  cv::Mat image(field_.rows, field_.cols, CV_8UC3);
  for (int x = 0; x < field_.cols; ++x) {
    for (int y = 0; y < field_.rows; ++y) {
      float r = field_.at<float>(y, x) * 255.0 / 30.0;
      image.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
    }
  }

  return image;
}

bool LikelihoodField::AlignG2O(SE2& init_pose) {
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;

  using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(
    new BlockSolverType(new LinearSolverType()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  auto* v = new VertexSE2();
  v->setId(0);
  v->setEstimate(init_pose);
  optimizer.addVertex(v);

  const double range_th = 15.0;  
  const double rk_delta = 0.8;

  has_outside_pts_ = false;

  for (size_t i = 0; i < source_->points.size(); ++i) {
    float r = hypot(source_->points[i].x, source_->points[i].y);
     
    if (r > range_th) {
      continue;
    }

    float angle = atan2(source_->points[i].y, source_->points[i].x);

    auto e = new EdgeSE2LikelihoodField(field_, r, angle, 1.0 / map_param_.resolution_);
    e->setVertex(0, v);

    if (e->IsOutSide()) {
      has_outside_pts_ = true;
      delete e;
      continue;
    }

    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    auto rk = new g2o::RobustKernelHuber;
    rk->setDelta(rk_delta);
    e->setRobustKernel(rk);
    optimizer.addEdge(e);
  }

  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  init_pose = v->estimate();
  return true;
  return true;
}

void LikelihoodField::SetFieldImageFromOccuMap(const cv::Mat& occu_map) {
  LOG(INFO) << "= SetFieldImageFromOccuMap =" << std::endl;
  const int boarder = 25;
  field_ = cv::Mat(map_param_.grid_size_[0], map_param_.grid_size_[1], CV_32F, 30.0);

  for (int x = boarder; x < occu_map.cols - boarder; ++x) {
    for (int y = boarder; y < occu_map.rows - boarder; ++y) {
      if (occu_map.at<uchar>(y, x) < 127) {
        for (auto& model_pt : model_) {
          int xx = int(x + model_pt.dx_);
          int yy = int(y + model_pt.dy_);
          if (xx >= 0 && xx < field_.cols && yy >= 0 && yy < field_.rows &&
            field_.at<float>(yy, xx) > model_pt.residual_) {
            field_.at<float>(yy, xx) = model_pt.residual_;
          }
        }
      }
    }
  }
}

}  // namespace semantic_mapping