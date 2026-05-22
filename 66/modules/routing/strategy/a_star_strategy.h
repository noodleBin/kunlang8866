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

#pragma once

#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/routing/strategy/strategy.h"

namespace century {
namespace routing {

struct SearchNode {
  const TopoNode* topo_node = nullptr;
  double f = std::numeric_limits<double>::max();

  SearchNode() = default;
  explicit SearchNode(const TopoNode* node)
      : topo_node(node), f(std::numeric_limits<double>::max()) {}
  SearchNode(const SearchNode& search_node) = default;

  bool operator<(const SearchNode& node) const {
    // in order to let the top of priority queue is the smallest one!
    return f > node.f;
  }

  bool operator==(const SearchNode& node) const {
    return topo_node == node.topo_node;
  }
};

class AStarStrategy : public Strategy {
 public:
  explicit AStarStrategy(bool enable_change);
  ~AStarStrategy() = default;

  virtual bool Search(const TopoGraph* graph, const SubTopoGraph* sub_graph,
                      const TopoNode* src_node, const TopoNode* dest_node,
                      std::vector<NodeWithRange>* const result_nodes,bool is_need_lanechange_first);

 private:
  void Clear();
  void InitializeSearch(const TopoNode* src_node, const TopoNode* dest_node,
                        std::priority_queue<SearchNode>* open_set_detail);
  void ProcessNode(SearchNode* current_node,
                   std::priority_queue<SearchNode>* open_set_detail);
  void UpdateNode(const TopoNode* from_node, const TopoNode* to_node,
                  const TopoEdge* edge, double tentative_g_score,
                  const TopoNode* dest_node,
                  std::priority_queue<SearchNode>* open_set_detail);
  double HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node);
  double GetResidualS(const TopoNode* node);
  double GetResidualS(const TopoEdge* edge, const TopoNode* to_node);
  double GetCostToNeighbor(const TopoEdge* edge);

 private:
  bool change_lane_enabled_;
  std::unordered_set<const TopoNode*> open_set_;
  std::unordered_set<const TopoNode*> closed_set_;
  std::unordered_map<const TopoNode*, const TopoNode*> came_from_;
  std::unordered_map<const TopoNode*, double> g_score_;
  std::unordered_map<const TopoNode*, double> enter_s_;
  bool is_need_lanechange_first_ = false;
};

}  // namespace routing
}  // namespace century
