//
// Created by xiaxinrong on 2025/8/15.
//

#pragma once

#include <fstream>
#include <map>
#include <memory>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/solvers/linear_solver_dense.h>
#include "common/eigen_types.h"
#include "likelihood_filed.h"
#include "submap.h"
#include "g2o_types.h"


namespace semantic_mapping {

class LoopClosing {
   public:

  struct LoopConstraints {
    LoopConstraints(size_t id1, size_t id2, const SE2& T12) : id_submap1_(id1), id_submap2_(id2), T12_(T12) {}
    size_t id_submap1_ = 0;
    size_t id_submap2_ = 0;
    SE2 T12_;  //  delat pose
    bool valid_ = true;
  };

  LoopClosing() { 
    debug_fout_.open("/century/data/log/loops.txt"); 
  }

  /// add new submap which is on building
  void AddNewSubmap(std::shared_ptr<Submap> submap) {
    submaps_.emplace(submap->id(), submap);
    current_submap_id_ = submap->id();
  }

  /// add one finished submap and will be called following AddNewSubmap invoked
  void AddFinishedSubmap(std::shared_ptr<Submap> submap) {
    // auto mr_field = std::make_shared<MRLikelihoodField>(4, submap->object_occu_map().resolution());
    auto lr_field = std::make_shared<LikelihoodField>();
    lr_field->set_pose(submap->pose());
    // mr_field->SetFieldImageFromOccuMap(submap->occu_map().GetOccupancyGrid());
    lr_field->SetFieldImageFromOccuMap(submap->object_occu_map().GetOccupancyGrid());
    submap_to_field_.emplace(submap, lr_field);
  }

  /// do loop detect for new frame ，and update it's pose and relative submap's pose
  void AddNewFrame(std::shared_ptr<Frame> frame) {
    LOG(WARNING) << "= LoopClosing::AddNewFrame =" << std::endl;
    current_frame_ = frame;
    if (!DetectLoopCandidates()) {
      return;
    }

    MatchInHistorySubmaps();

    if (has_new_loops_) {
      Optimize();
    }
  }

  /// get loop detection result among submap
  std::map<std::pair<size_t, size_t>, LoopConstraints> GetLoops() const { return loop_constraints_; }

  bool HasNewLoops() const { return has_new_loops_; }

   private:

  bool DetectLoopCandidates() {
    LOG(WARNING) << "= LoopClosing::DetectLoopCandidates =" << std::endl;
    // current frame need some gap with history submap
    has_new_loops_ = false;
    if (current_submap_id_ < submap_gap_) {
      return false;
    }

    current_candidates_.clear();

    LOG(WARNING) << "= LoopClosing::DetectLoopCandidates-02 =" << std::endl;
    for (auto& sp : submaps_) {
      if ((current_submap_id_ - sp.first) <= submap_gap_) {
        // not check nearby submap
        continue;
      }

      auto hist_iter = loop_constraints_.find(std::pair<size_t, size_t>(sp.first, current_submap_id_));
      if (hist_iter != loop_constraints_.end() && hist_iter->second.valid_) {
        continue;
      }

      Vec2d center = sp.second->pose().translation();
      Vec2d frame_pos = current_frame_->pose_map_SE2().translation();
      double dis = (center - frame_pos).norm();
      if (dis < candidate_distance_th_) {
      
        LOG(INFO) << "taking " << current_frame_->keyframe_id() << " with " << sp.first
              << ", last submap id: " << current_submap_id_;
        current_candidates_.emplace_back(sp.first);
      }
    }

    LOG(WARNING) << "= LoopClosing::DetectLoopCandidates-04 =" << std::endl;
    return !current_candidates_.empty();
  }

 
  void MatchInHistorySubmaps(){
    LOG(WARNING) << "= LoopClosing::MatchInHistorySubmaps =" << std::endl;

    LOG(WARNING) << "current_candidates_: " << current_candidates_.size() << std::endl;
    for (const size_t& can : current_candidates_) {
      LOG(WARNING) << "can: " << can << std::endl;
      auto mr = submap_to_field_.at(submaps_[can]);
      mr->SetSourceScan(current_frame_->scan());
     //   LOG(WARNING) << "current_frame_->scan(): " << current_frame_->scan()->ranges.size() << std::endl;

      auto submap = submaps_[can];
      SE2 pose_in_target_submap = submap->pose().inverse() * current_frame_->pose_map_SE2();  // T_S1_C

      LOG(WARNING) << "AlignG2O" << std::endl;
      if (mr->AlignG2O(pose_in_target_submap)) {
        LOG(WARNING) << "AlignG2O-true" << std::endl;
        // set constraints from current submap to target submap
        // T_S1_S2 = T_S1_C * T_C_W * T_W_S2
        SE2 T_this_cur =
          pose_in_target_submap * current_frame_->pose_map_SE2().inverse() * submaps_[current_submap_id_]->pose();
        loop_constraints_.emplace(std::pair<size_t, size_t>(can, current_submap_id_),
                      LoopConstraints(can, current_submap_id_, T_this_cur));
        LOG(INFO) << "adding loop from submap " << can << " to " << current_submap_id_;

      
        // auto occu_image = submap->occu_map().GetOccupancyGridBlackWhite();
        auto occu_image = submap->object_occu_map().GetOccupancyGrid();
         
        cv::putText(occu_image, "loop submap " + std::to_string(submap->id()), cv::Point2f(20, 20),
              cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
        cv::putText(occu_image, "keyframes " + std::to_string(submap->num_keyframes()), cv::Point2f(20, 50),
              cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
        cv::imshow("loop closure", occu_image);

        has_new_loops_ = true;
      } else {
        LOG(WARNING) << "AlignG2O-false" << std::endl;
      }

      debug_fout_ << current_frame_->id() << " " << can << " " << submaps_[can]->pose().translation().x() << " "
            << submaps_[can]->pose().translation().y() << " " << submaps_[can]->pose().so2().log()
            << std::endl;
    }

    current_candidates_.clear();
  }

  /// pose graph optimization
  // called by: AddNewFrame[has_new_loops_=true]
  void Optimize() {
    LOG(WARNING) << "= LoopClosing::Optimize =" << std::endl;
    // pose graph optimization
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;

    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
      new BlockSolverType(new LinearSolverType()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    for (auto& sp : submaps_) {
      auto* v = new VertexSE2();
      v->setId(sp.first);
      v->setEstimate(sp.second->pose());
      if (sp.first == 0) {
        v->setFixed(true);
      }
      optimizer.addVertex(v);
    }

    /// constrain among submap
    for (int i = 0; i < current_submap_id_; ++i) {
      auto first_submap = submaps_[i];
      auto next_submap = submaps_[i + 1];

      EdgeSE2* e = new EdgeSE2();
      e->setVertex(0, optimizer.vertex(i));
      e->setVertex(1, optimizer.vertex(i + 1));
      e->setMeasurement(first_submap->pose_odom().inverse() * next_submap->pose_odom());
      e->setInformation(Mat3d::Identity() * 1e4);

      optimizer.addEdge(e);
    }

    /// constrain among loop closure
    std::map<std::pair<size_t, size_t>, EdgeSE2*> loop_edges;
    for (auto& c : loop_constraints_) {
      if (!c.second.valid_) {
        continue;
      }

      auto first_submap = submaps_[c.first.first];
      auto second_submap = submaps_[c.first.second];

      EdgeSE2* e = new EdgeSE2();
      e->setVertex(0, optimizer.vertex(first_submap->id()));
      e->setVertex(1, optimizer.vertex(second_submap->id()));
      e->setMeasurement(c.second.T12_);
      e->setInformation(Mat3d::Identity());

      auto rk = new g2o::RobustKernelCauchy;
      rk->setDelta(loop_rk_delta_);
      e->setRobustKernel(rk);

      optimizer.addEdge(e);
      loop_edges.emplace(c.first, e);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // validate the loop constraints
    int inliers = 0;
    for (auto& ep : loop_edges) {
      if (ep.second->chi2() < loop_rk_delta_) {
        LOG(INFO) << "loop from " << ep.first.first << " to " << ep.first.second
              << " is correct, chi2: " << ep.second->chi2();
        ep.second->setRobustKernel(nullptr);
        loop_constraints_.at(ep.first).valid_ = true;
        inliers++;
      } else {
        ep.second->setLevel(1);
        LOG(INFO) << "loop from " << ep.first.first << " to " << ep.first.second
              << " is invalid, chi2: " << ep.second->chi2();
        loop_constraints_.at(ep.first).valid_ = false;
      }
    }

    optimizer.optimize(5);

    for (auto& sp : submaps_) {
      VertexSE2* v = (VertexSE2*)optimizer.vertex(sp.first);
      sp.second->set_pose(v->estimate());

  
      sp.second->UpdateFramePoseWorld();
    }

    LOG(INFO) << "loop inliers: " << inliers << "/" << loop_constraints_.size();

    // remove error match
    for (auto iter = loop_constraints_.begin(); iter != loop_constraints_.end();) {
      if (!iter->second.valid_) {
        iter = loop_constraints_.erase(iter);
      } else {
        iter++;
      }
    }
  }

  std::shared_ptr<Frame> current_frame_ = nullptr;
  size_t current_submap_id_ = 0;  

  std::map<size_t, std::shared_ptr<Submap>> submaps_; 

 
  std::map<std::shared_ptr<Submap>, std::shared_ptr<LikelihoodField>> submap_to_field_;

  std::vector<size_t> current_candidates_;           
  std::map<std::pair<size_t, size_t>, LoopConstraints> loop_constraints_;  // submap id pair to loop constraints
  bool has_new_loops_ = false;

  std::ofstream debug_fout_;

  inline static constexpr float candidate_distance_th_ = 15.0;  // candidate frame distcnce to submap center
  inline static constexpr int submap_gap_ = 1;          
  inline static constexpr double loop_rk_delta_ = 1.0; 
};

}  // namespace semantic_mapping

