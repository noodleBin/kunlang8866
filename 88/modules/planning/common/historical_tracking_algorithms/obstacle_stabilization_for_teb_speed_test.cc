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

/**
 * @file obstacle_stabilization_for_teb_speed_test.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/obstacle_stabilization_for_teb_speed.h"

#include <vector>

#include "gtest/gtest.h"

namespace century {
namespace planning {
namespace {
constexpr double kEpsilon = 1e-8;
}  // namespace

TEST(ObstacleStabilizationForTEBSpeedTest, SingleObstacle) {
  // intial input data
  std::vector<std::tuple<std::string, double, double>> obstacles_info{
      {"123", 0.8, 0.8}, {"123", 0.9, 0.8}, {"123", 1.0, 0.8},
      {"123", 1.1, 1.1}, {"123", 1.2, 1.2}, {"123", 1.3, 1.3},
      {"123", 1.1, 1.1}, {"123", 1.2, 1.2}, {"123", 1.5, 1.5},
      {"123", 0.7, 0.7}, {"123", 0.8, 0.7}, {"123", 0.7, 0.7},
      {"123", 2.0, 0.7}, {"123", 1.8, 0.7}, {"123", 1.5, 0.7},
      {"123", 1.3, 0.7}, {"123", 1.2, 0.7}, {"123", 1.1, 1.1},
      {"123", 1.3, 1.3}, {"123", 0.9, 0.9}, {"123", 1.2, 0.9}};
  ObstacleStabilizationForTEBSpeed object(50UL);
  std::vector<double> stable_lat_distances;
  uint32_t seq_num = 1UL;
  for (auto& item : obstacles_info) {
    object.SetSequenceNum(seq_num);
    double stable_lat_dis = object.GetObstacleDistance(
        std::get<0>(item), std::get<1>(item), true, 0.0);
    EXPECT_FLOAT_EQ(stable_lat_dis, std::get<2>(item))
        << "unexpected seqence number is " << seq_num << ", \nitem value is ["
        << std::get<0>(item) << ", " << std::get<1>(item) << ", "
        << std::get<2>(item) << "]";
    ++seq_num;
  }
}

TEST(ObstacleStabilizationForTEBSpeedTest, MultiObstacle) {
  // intial input data
  std::vector<std::tuple<std::string, double, double>> obstacles_info{
      {"123", 0.8, 0.8}, {"456", 2.0, 2.0}, {"789", 1.0, 1.0},
      {"123", 1.0, 0.8}, {"456", 1.8, 1.8}, {"789", 1.3, 1.0},
      {"123", 1.1, 0.8}, {"456", 1.6, 1.6}, {"789", 1.5, 1.0},
      {"123", 1.2, 1.2}, {"456", 1.8, 1.6}, {"789", 0.7, 0.7},
      {"123", 1.3, 1.3}, {"456", 1.4, 1.4}, {"789", 1.5, 0.7},
      {"123", 1.1, 1.1}, {"456", 1.2, 1.2}, {"789", 1.1, 0.7},
      {"123", 1.3, 1.3}, {"456", 1.5, 1.2}, {"789", 1.2, 0.7}};
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  ObstacleStabilizationForTEBSpeed object(50UL);
  std::vector<double> stable_lat_distances;
  uint32_t seq_num = 1UL;
  for (auto& item : obstacles_info) {
    object.SetSequenceNum(seq_num);
    double stable_lat_dis = object.GetObstacleDistance(
        std::get<0>(item), std::get<1>(item), true, 0.0);
    EXPECT_FLOAT_EQ(stable_lat_dis, std::get<2>(item))
        << "unexpected seqence number is " << seq_num << ", \nitem value is ["
        << std::get<0>(item) << ", " << std::get<1>(item) << ", "
        << std::get<2>(item) << "]";
    if (std::get<0>(item) == "789") {
      ++seq_num;
    }
  }
}

}  // namespace planning
}  // namespace century
