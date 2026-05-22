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

#include <boost/shared_ptr.hpp>

namespace century {
namespace planning {

/**
 * @class EquivalenceClass
 * @brief Abstract class that defines an interface for computing and comparing
 * equivalence classes
 *
 * Equivalence relations are utilized in order to test if two trajectories are
 * belonging to the same equivalence class w.r.t. the current obstacle
 * configurations. A common equivalence relation is the concept of homotopy
 * classes. All trajectories belonging to the same homotopy class can
 * CONTINUOUSLY be deformed into each other without intersecting any obstacle.
 * Hence they likely share the same local minimum after invoking (local)
 * trajectory optimization. A weaker equivalence relation is defined by the
 * concept of homology classes (e.g. refer to HSignature).
 *
 * Each EquivalenceClass object (or subclass) stores a candidate value which
 * might be compared to another EquivalenceClass object.
 *
 * @remarks Currently, the computeEquivalenceClass method is not available in
 * the generic interface EquivalenceClass. Call the "compute"-methods directly
 * on the subclass.
 */
class EquivalenceClass {
 public:
  /**
   * @brief Default constructor
   */
  EquivalenceClass() {}

  /**
   * @brief virtual destructor
   */
  virtual ~EquivalenceClass() {}

  /**
   * @brief Check if two candidate classes are equivalent
   * @param other The other equivalence class to test with
   */
  virtual bool IsEqual(const EquivalenceClass &other) const = 0;

  /**
   * @brief Check if the equivalence value is detected correctly
   * @return Returns false, if the equivalence class detection failed, e.g. if
   * nan- or inf values occur.
   */
  virtual bool IsValid() const = 0;

  /**
   * @brief Check if the trajectory is non-looping around an obstacle
   * @return Returns false, if the trajectory loops around an obstacle
   */
  virtual bool IsReasonable() const = 0;
};

using EquivalenceClassPtr = boost::shared_ptr<EquivalenceClass>;

}  // namespace planning
}  // namespace century
