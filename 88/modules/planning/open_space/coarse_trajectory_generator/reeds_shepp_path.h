/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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
 * @file
 */

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <omp.h>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"

namespace century {
namespace planning {

struct ReedSheppPath {
  std::vector<double> segs_lengths;
  std::vector<char> segs_types;
  double total_length = 0.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  // true for driving forward and false for driving backward
  std::vector<bool> gear;
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

class ReedShepp {
 public:
  ReedShepp(const common::VehicleParam& vehicle_param,
            const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~ReedShepp() = default;
  // Pick the shortest path from all possible combination of movement primitives
  // by Reed Shepp
  bool ShortestRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::shared_ptr<ReedSheppPath> optimal_path);
  // Pick the bestest path from all possible combination of movement primitives
  // by Reed Shepp
  bool BestestRSP(const std::shared_ptr<Node3d> start_node,
                  const std::shared_ptr<Node3d> end_node,
                  std::shared_ptr<ReedSheppPath> optimal_path);
  void SetKappaContraintConfig(const double traj_kappa_contraint_ratio);
  void ResetDefaultKappaContraint();
  // Get ReedSheppPath Label
  std::string GetReedSheppPathLabel(const ReedSheppPath& path);

 protected:
  // Generate all possible combination of movement primitives by Reed Shepp and
  // interpolate them
  bool GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    std::vector<ReedSheppPath>* all_possible_paths);
  // Generate all possible combination of movement primitives by Reed Shepp and
  // interpolate them
  bool GenerateBestRSPs(const std::shared_ptr<Node3d> start_node,
                        const std::shared_ptr<Node3d> end_node,
                        std::vector<ReedSheppPath>* all_possible_paths);

  // Set the general profile of the movement primitives
  bool GenerateRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::vector<ReedSheppPath>* all_possible_paths);
  // Set the best general profile of the movement primitives
  bool GenerateBestRSP(const std::shared_ptr<Node3d> start_node,
                       const std::shared_ptr<Node3d> end_node,
                       std::vector<ReedSheppPath>* all_possible_paths);
  // Set the general profile of the movement primitives, parallel implementation
  bool GenerateRSPPar(const std::shared_ptr<Node3d> start_node,
                      const std::shared_ptr<Node3d> end_node,
                      std::vector<ReedSheppPath>* all_possible_paths);
  // Get the shortest path index from all possible combination paths
  size_t GetShortestRSPIndex(
      const std::vector<ReedSheppPath>& all_possible_paths);

  // Get the bestest rescue path index from all possible combination paths
  bool GetBestRescueRSPIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      size_t* best_path_index);
  // Find the ideal rescue path index from all possible combination paths
  bool FindIdealRescueTypeIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      size_t* best_path_index);
  // Find the cost rescue path index from all possible combination paths
  bool FindCostRescueTypeIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      size_t* best_path_index);
  // Find Max Forward Segs
  void FindMaxForwardSegsIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      std::vector<size_t>* path_forward_segs);
  // Find Max Forward Lengths
  void FindMaxForwardLengthsIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      const std::vector<size_t>& max_forward_path_indexs,
      std::vector<double>* path_forward_lengths);

  // Get the bestest uturn path index from all possible combination paths
  bool GetBestUTurnRSPIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      size_t* best_path_index);
  // Find the ideal uturn path index from all possible combination paths
  bool FindIdealUTurnTypeIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      size_t* best_path_index);
  // Find the cost uturn path index from all possible combination paths
  bool FindCostUTurnTypeIndex(
      const std::vector<ReedSheppPath>& all_possible_paths,
      size_t* best_path_index);

  // Set local exact configurations profile of each movement primitive
  bool GenerateLocalConfigurations(const std::shared_ptr<Node3d> start_node,
                                   const std::shared_ptr<Node3d> end_node,
                                   ReedSheppPath* shortest_path);
  // Interpolation usde in GenetateLocalConfiguration
  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double>* px, std::vector<double>* py,
                     std::vector<double>* pphi, std::vector<bool>* pgear);
  // motion primitives combination setup function
  bool SetRSP(const int size, const double* lengths, const char* types,
              std::vector<ReedSheppPath>* all_possible_paths);
  // setRSP parallel version
  bool SetRSPPar(const int size, const double* lengths,
                 const std::string& types,
                 std::vector<ReedSheppPath>* all_possible_paths, const int idx);
  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC_LSL(const double x, const double y, const double phi,
               std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC_LSR(const double x, const double y, const double phi,
               std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC_Forward(const double x, const double y, const double phi,
                   std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC_Backward(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC_LRLRn(const double x, const double y, const double phi,
                  std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC_LRLRp(const double x, const double y, const double phi,
                  std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC_Forward(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC_Forward_LRSL(const double x, const double y, const double phi,
                         std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC_Forward_LRSR(const double x, const double y, const double phi,
                         std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC_Backward(const double x, const double y, const double phi,
                     std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC_Backward_LRSL(const double x, const double y, const double phi,
                          std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC_Backward_LRSR(const double x, const double y, const double phi,
                          std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath>* all_possible_paths);
  // different options for different combination of motion primitives
  void LSL(const double x, const double y, const double phi, RSPParam* param);
  void LSR(const double x, const double y, const double phi, RSPParam* param);
  void LRL(const double x, const double y, const double phi, RSPParam* param);
  void SLS(const double x, const double y, const double phi, RSPParam* param);
  void LRLRn(const double x, const double y, const double phi, RSPParam* param);
  void LRLRp(const double x, const double y, const double phi, RSPParam* param);
  void LRSR(const double x, const double y, const double phi, RSPParam* param);
  void LRSL(const double x, const double y, const double phi, RSPParam* param);
  void LRSLR(const double x, const double y, const double phi, RSPParam* param);
  std::pair<double, double> calc_tau_omega(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

 protected:
  common::VehicleParam vehicle_param_;
  PlannerOpenSpaceConfig planner_open_space_config_;
  double max_kappa_;
};
}  // namespace planning
}  // namespace century
