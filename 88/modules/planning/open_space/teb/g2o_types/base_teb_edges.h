/******************************************************************************
 * Copyright 2023 The Century Authors. All Rights Reserved.
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

#include <cmath>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>

#include "modules/planning/open_space/teb/utils/teb_config.h"

namespace century {
namespace planning {

/**
 * @class BaseTebUnaryEdge
 * @brief Base edge connecting a single vertex in the TEB optimization problem
 *
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with
 * additional information for the dedicated TEB problem (e.g. config). The
 * destructor erases the edge in all attached vertices in order to allow keeping
 * the vertices valid in subsequent g2o optimization calls. Memory of edges
 * should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebMultiEdge, BaseTebBinaryEdge, g2o::BaseBinaryEdge,
 * g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi> {
 public:
  using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
  using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

  /**
   * @brief Construct edge.
   */
  BaseTebUnaryEdge() { _vertices[0] = NULL; }

  /**
   * @brief Destruct edge.
   *
   * We need to erase vertices manually, since we want to keep them even if
   * TebOptimalPlanner::clearGraph() is called. This is necessary since the
   * vertices are managed by the Timed_Elastic_Band class.
   */
  virtual ~BaseTebUnaryEdge() {
    if (_vertices[0]) {
      _vertices[0]->edges().erase(this);
    }
  }

  /**
   * @brief Compute and return error / cost value.
   *
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain
   * the current cost.
   * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
   */
  ErrorVector& GetError() {
    computeError();
    return _error;
  }

  /**
   * @brief Read values from input stream
   */
  bool read(std::istream& is) override {
    // TODO(all) generic Read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */
  bool write(std::ostream& os) const override {
    // TODO(all) generic Write
    return os.good();
  }

  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */
  void SetTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

 protected:
  using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
  using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

  const TebConfig* cfg_;  //!< Store TebConfig class for parameters

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class BaseTebBinaryEdge
 * @brief Base edge connecting two vertices in the TEB optimization problem
 *
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with
 * additional information for the dedicated TEB problem (e.g. config). The
 * destructor erases the edge in all attached vertices in order to allow keeping
 * the vertices valid in subsequent g2o optimization calls. Memory of edges
 * should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebMultiEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge,
 * g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj> {
 public:
  using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

  /**
   * @brief Construct edge.
   */
  BaseTebBinaryEdge() { _vertices[0] = _vertices[1] = NULL; }

  /**
   * @brief Destruct edge.
   *
   * We need to erase vertices manually, since we want to keep them even if
   * TebOptimalPlanner::clearGraph() is called. This is necessary since the
   * vertices are managed by the Timed_Elastic_Band class.
   */
  virtual ~BaseTebBinaryEdge() {
    if (_vertices[0]) {
      _vertices[0]->edges().erase(this);
    }
    if (_vertices[1]) {
      _vertices[1]->edges().erase(this);
    }
  }

  /**
   * @brief Compute and return error / cost value.
   *
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain
   * the current cost.
   * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
   */
  ErrorVector& GetError() {
    computeError();
    return _error;
  }

  /**
   * @brief Read values from input stream
   */
  bool read(std::istream& is) override {
    // TODO(all) generic Read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */
  bool write(std::ostream& os) const override {
    // TODO(all) generic Write
    return os.good();
  }

  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */
  void SetTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

 protected:
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

  const TebConfig* cfg_;  //!< Store TebConfig class for parameters

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @class BaseTebMultiEdge
 * @brief Base edge connecting two vertices in the TEB optimization problem
 *
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with
 * additional information for the dedicated TEB problem (e.g. config). The
 * destructor erases the edge in all attached vertices in order to allow keeping
 * the vertices valid in subsequent g2o optimization calls. Memory of edges
 * should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebBinaryEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge,
 * g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */
template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E> {
 public:
  using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
  using g2o::BaseMultiEdge<D, E>::computeError;

  /**
   * @brief Construct edge.
   */
  BaseTebMultiEdge() {
    for (std::size_t i = 0; i < _vertices.size(); ++i) {
      _vertices[i] = NULL;
    }
  }

  /**
   * @brief Destruct edge.
   *
   * We need to erase vertices manually, since we want to keep them even if
   * TebOptimalPlanner::clearGraph() is called. This is necessary since the
   * vertices are managed by the Timed_Elastic_Band class.
   */
  virtual ~BaseTebMultiEdge() {
    for (std::size_t i = 0; i < _vertices.size(); ++i) {
      if (_vertices[i]) {
        _vertices[i]->edges().erase(this);
      }
    }
  }

  // Overwrites resize() from the parent class
  virtual void resize(size_t size) {
    g2o::BaseMultiEdge<D, E>::resize(size);

    for (std::size_t i = 0; i < _vertices.size(); ++i) {
      _vertices[i] = NULL;
    }
  }

  /**
   * @brief Compute and return error / cost value.
   *
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain
   * the current cost.
   * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
   */
  ErrorVector& GetError() {
    computeError();
    return _error;
  }

  /**
   * @brief Read values from input stream
   */
  bool read(std::istream& is) override {
    // TODO(all) generic Read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */
  bool write(std::ostream& os) const override {
    // TODO(all) generic Write
    return os.good();
  }

  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */
  void SetTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

 protected:
  using g2o::BaseMultiEdge<D, E>::_error;
  using g2o::BaseMultiEdge<D, E>::_vertices;

  const TebConfig* cfg_;  //!< Store TebConfig class for parameters

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning
}  // namespace century
