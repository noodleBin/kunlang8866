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
 * @file hysteresis_interval_test.cc
 **/

#include "modules/planning/common/historical_tracking_algorithms/hysteresis_interval.h"

#include <array>

#include "gtest/gtest.h"

#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "cyber/common/file.h"
namespace century {
namespace planning {
namespace {
constexpr double kEpsilon = 1e-8;
}  // namespace
namespace {
testing::AssertionResult DoubleLessNearPredFormat(const char* expr1,
                                                  const char* expr2,
                                                  const char* abs_error_expr,
                                                  double val1, double val2,
                                                  double abs_error) {
  const double diff = val1 - val2;
  testing::Message msg;
  if (std::abs(diff) <= abs_error && diff < 0) {
    return testing::AssertionSuccess();
  } else if (std::abs(diff) <= abs_error) {
    msg << "The " << expr1 << " is greater than " << expr2 << ", where\n"
        << expr1 << " evaluates to " << val1 << ",\n"
        << expr2 << " evaluates to " << val2 << ", and\n"
        << abs_error_expr << " evaluates to " << abs_error << ".";
  } else {
    msg << "The difference between " << expr1 << " and " << expr2 << " is "
        << diff << ", which exceeds " << abs_error_expr << ", where\n"
        << expr1 << " evaluates to " << val1 << ",\n"
        << expr2 << " evaluates to " << val2 << ", and\n"
        << abs_error_expr << " evaluates to " << abs_error << ".";
  }
  return testing::AssertionFailure(msg);
}

testing::AssertionResult DoubleGreatNearPredFormat(const char* expr1,
                                                   const char* expr2,
                                                   const char* abs_error_expr,
                                                   double val1, double val2,
                                                   double abs_error) {
  const double diff = val1 - val2;
  testing::Message msg;
  if (std::abs(diff) <= abs_error && diff > 0) {
    return testing::AssertionSuccess();
  } else if (std::abs(diff) <= abs_error) {
    msg << "The " << expr1 << " is less than " << expr2 << ", where\n"
        << expr1 << " evaluates to " << val1 << ",\n"
        << expr2 << " evaluates to " << val2 << ", and\n"
        << abs_error_expr << " evaluates to " << abs_error << ".";
  } else {
    msg << "The difference between " << expr1 << " and " << expr2 << " is "
        << diff << ", which exceeds " << abs_error_expr << ", where\n"
        << expr1 << " evaluates to " << val1 << ",\n"
        << expr2 << " evaluates to " << val2 << ", and\n"
        << abs_error_expr << " evaluates to " << abs_error << ".";
  }
  return testing::AssertionFailure(msg);
}

#define EXPECT_LT_NEAR(val1, val2, abs_error) \
  EXPECT_PRED_FORMAT3(DoubleLessNearPredFormat, val1, val2, abs_error)

#define EXPECT_GT_NEAR(val1, val2, abs_error) \
  EXPECT_PRED_FORMAT3(DoubleGreatNearPredFormat, val1, val2, abs_error)
}  // namespace

class HysteresisIntervalTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    // initial obstacles
    prediction::PredictionObstacles prediction_obstacles_2162,
        prediction_obstacles_2163, prediction_obstacles_2164,
        prediction_obstacles_multi;
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "/century/modules/planning/testdata/common/sample_prediction_2162.pb.txt",
        &prediction_obstacles_2162));
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "/century/modules/planning/testdata/common/sample_prediction_2163.pb.txt",
        &prediction_obstacles_2163));
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "/century/modules/planning/testdata/common/sample_prediction_2164.pb.txt",
        &prediction_obstacles_2164));
    ASSERT_TRUE(cyber::common::GetProtoFromFile(
        "/century/modules/planning/testdata/common/"
        "sample_prediction_multi_obstacles.pb.txt",
        &prediction_obstacles_multi));

    auto obstacles_2162_uniptr =
        Obstacle::CreateObstacles(prediction_obstacles_2162);
    auto obstacles_2163_uniptr =
        Obstacle::CreateObstacles(prediction_obstacles_2163);
    auto obstacles_2164_uniptr =
        Obstacle::CreateObstacles(prediction_obstacles_2164);
    auto obstacles_multi_uniptr =
        Obstacle::CreateObstacles(prediction_obstacles_multi);

    for (auto& uptr : obstacles_2162_uniptr) {
      std::shared_ptr<Obstacle> sptr(std::move(uptr));
      obstacles_2162.push_back(std::move(sptr));
    }
    for (auto& uptr : obstacles_2163_uniptr) {
      std::shared_ptr<Obstacle> sptr(std::move(uptr));
      obstacles_2163.push_back(std::move(sptr));
    }
    for (auto& uptr : obstacles_2164_uniptr) {
      std::shared_ptr<Obstacle> sptr(std::move(uptr));
      obstacles_2164.push_back(std::move(sptr));
    }
    for (auto& uptr : obstacles_multi_uniptr) {
      std::shared_ptr<Obstacle> sptr(std::move(uptr));
      obstacles_multi.push_back(std::move(sptr));
    }

    ASSERT_EQ(21, obstacles_2162.size());
    ASSERT_EQ(9, obstacles_2163.size());
    ASSERT_EQ(7, obstacles_2164.size());
    ASSERT_EQ(30, obstacles_multi.size());
  }

  virtual void TearDown() {}

 protected:
  std::list<std::shared_ptr<Obstacle>> obstacles_2162;
  std::list<std::shared_ptr<Obstacle>> obstacles_2163;
  std::list<std::shared_ptr<Obstacle>> obstacles_2164;
  std::list<std::shared_ptr<Obstacle>> obstacles_multi;
  std::shared_ptr<HysteresisInterval> object = nullptr;
};

TEST_F(HysteresisIntervalTest, TraceSingleObstacle) {
  // intial hysteresis interval
  std::array<std::shared_ptr<Obstacle>, 21> obstacles;
  auto it_2162 = obstacles_2162.begin();
  for (auto& item : obstacles) {
    item = *it_2162++;
  }
  ASSERT_TRUE(it_2162 == obstacles_2162.end());
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 2162: 0.8->1.8->0.8 (step 0.1)
  // |Index | 0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  | 9  | 10 |
  // |Real V|0.8 |0.9 |1.0 |1.1 |1.2 |1.3 |1.4 |1.5 |1.6 |1.7 |1.8 |
  // |Hy V  |0.8 |0.9 |1.0 |1.1 |1.2 |1.3-|1.3-|1.3-|1.6 |1.7 |1.8 |
  // ---------------------------------------------------------------
  // |Index | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 |
  // |Real V|1.7 |1.6 |1.5 |1.4 |1.3 |1.2 |1.1 |1.0 |0.9 |0.8 |
  // |Hy V  |1.7 |1.6 |1.5 |1.4 |1.3+|1.3+|1.3+|1.0 |0.9 |0.8 |
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  object = std::make_shared<HysteresisInterval>(1.3, 0.6);  // [1.0, 1.6]
  size_t pos = 0;
  for (const auto& item : obstacles) {
    HysteresisInterval::SetSequenceNum(pos);
    // position count base 0
    // 5:1.3-, 6: 1.3-, 7:1.3-, 15:1.3+, 16:1.3+, 17:1.3+
    if (5 == pos || 6 == pos || 7 == pos) {
      EXPECT_LT_NEAR(object->HyValue(*item, item->speed()), 1.3, kEpsilon)
          << "unexpected position is " << pos;
    } else if (15 == pos || 16 == pos || 17 == pos) {
      EXPECT_GT_NEAR(object->HyValue(*item, item->speed()), 1.3, kEpsilon)
          << "unexpected position is " << pos;
    } else {
      EXPECT_FLOAT_EQ(object->HyValue(*item, item->speed()), item->speed())
          << "unexpected position is " << pos;
    }
    ++pos;
  }
  object = nullptr;
}

TEST_F(HysteresisIntervalTest, TraceMultiObstacle) {
  // intial hysteresis interval
  std::array<std::shared_ptr<Obstacle>, 37> obstacles;
  auto it_2162 = obstacles_2162.begin();
  auto it_2163 = obstacles_2163.begin();
  auto it_2164 = obstacles_2164.begin();
  std::array<int, 37> positions{1, 1, 1, 2, 3, 1, 1, 1, 2, 3, 1, 1, 1,
                                2, 3, 1, 1, 1, 2, 3, 1, 1, 1, 2, 3, 1,
                                1, 1, 2, 2, 3, 1, 1, 1, 2, 2, 3};
  auto it_p = positions.begin();
  for (auto& item : obstacles) {
    if (1 == *it_p) {
      item = *it_2162++;
    } else if (2 == *it_p) {
      item = *it_2163++;
    } else if (3 == *it_p) {
      item = *it_2164++;
    } else {
      FAIL() << "the obstacles initialization failed at position: " << *it_p;
    }
    ++it_p;
  }
  ASSERT_TRUE(it_2162 == obstacles_2162.end() &&
              it_2163 == obstacles_2163.end() &&
              it_2164 == obstacles_2164.end());
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 2162: 0.8->1.8->0.8 (step 0.1)
  // |Index | 0  | 1  | 2  | 3  | 4  | 5   | 6   | 7  | 8  | 9  | 10 |
  // |Real V|0.8 |0.9 |1.0 |1.1 |1.2 |1.3  |1.4  |1.5 |1.6 |1.7 |1.8 |
  // |Hy V  |0.8 |0.9 |1.0 |1.1 |1.2 |1.25-|1.25-|1.5 |1.6 |1.7 |1.8 |
  // ---------------------------------------------------------------
  // |Index | 11 | 12 | 13 | 14 | 15 | 16  | 17  | 18 | 19 | 20 |
  // |Real V|1.7 |1.6 |1.5 |1.4 |1.3 |1.2  |1.1  |1.0 |0.9 |0.8 |
  // |Hy V  |1.7 |1.6 |1.5 |1.4 |1.3 |1.25+|1.25+|1.0 |0.9 |0.8 |
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 2163: 1.3->0.8->1.6 (1.3->1.0:step 0.1, 1.0->0.8->1.6:step 0.2)
  // |Index | 0  | 1   | 2   | 3  | 4  | 5  | 6  | 7   | 8  |
  // |Real V|1.3 |1.2  |1.1  |1.0 |0.8 |1.0 |1.2 |1.4  |1.6 |
  // |Hy V  |1.3 |1.25+|1.25+|1.0 |0.8 |1.0 |1.2 |1.25-|1.6 |
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 2164: 1.8->1.2->1.8 (step 0.2)
  // |Index | 0  | 1  | 2  | 3   | 4   | 5  | 6  |
  // |Real V|1.8 |1.6 |1.4 |1.2  |1.4  |1.6 |1.8 |
  // |Hy V  |1.8 |1.6 |1.4 |1.25+|1.25+|1.6 |1.8 |
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  object = std::make_shared<HysteresisInterval>(1.25, 0.5);  // [1.0, 1.5]
  std::array<int, 37> seq_nums{1,  2,  3,  3,  3,  4,  5,  6,  6,  6,
                               7,  8,  9,  9,  9,  10, 11, 12, 12, 12,
                               13, 14, 15, 15, 15, 16, 17, 18, 18, 19,
                               19, 20, 21, 22, 22, 23, 23};
  size_t pos = 0;
  for (const auto& item : obstacles) {
    HysteresisInterval::SetSequenceNum(seq_nums[pos]);
    // position count base 0
    // 2162: 7- 1.25-, 10-1.25-, 26-1.25+, 27-1.25+
    // 2163: 8- 1.25+, 13-1.25+, 34-1.25-
    // 2164: 19-1.25+
    if (7 == pos || 10 == pos || 34 == pos) {
      EXPECT_LT_NEAR(object->HyValue(*item, item->speed()), 1.25, kEpsilon)
          << "unexpected position is " << pos;
    } else if (26 == pos || 27 == pos || 8 == pos || 13 == pos || 19 == pos) {
      EXPECT_GT_NEAR(object->HyValue(*item, item->speed()), 1.25, kEpsilon)
          << "unexpected position is " << pos;
    } else {
      EXPECT_FLOAT_EQ(object->HyValue(*item, item->speed()), item->speed())
          << "unexpected position is " << pos;
    }
    ++pos;
  }
  object = nullptr;
}

TEST_F(HysteresisIntervalTest, AutoRemoveObstacle1) {
  // intial hysteresis interval
  std::array<std::shared_ptr<Obstacle>, 30> obstacles;
  auto it_multi = obstacles_multi.begin();
  for (auto& item : obstacles) {
    item = *it_multi++;
  }
  ASSERT_TRUE(it_multi == obstacles_multi.end());
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // |ID    |1001|1002|1003|1004|1005|1006|1007|1008|1009|1010|
  // |Pos X |2.1 |2.2 |2.3 |2.4 |2.5 |2.6 |2.7 |2.8 |2.9 |3.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ----------------------------------------------------------
  // |ID    |1011|1012|1013|1014|1015|1016|1017|1018|1019|1020|
  // |Pos X |3.1 |3.2 |3.3 |3.4 |3.5 |3.6 |3.7 |3.8 |3.9 |4.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ----------------------------------------------------------
  // |ID    |1021|1022|1023|1024|1025|1026|1027|1028|1029|1030|
  // |Pos X |4.1 |4.2 |4.3 |4.4 |4.5 |4.6 |4.7 |4.8 |4.9 |5.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  object = std::make_shared<HysteresisInterval>(1.25, 0.5);  // [1.0, 1.5]
  object->SetCapacity(20UL);
  HysteresisInterval::SetSequenceNum(1);
  for (const auto& item : obstacles) {
    object->HyValue(*item, item->speed());
  }
  EXPECT_EQ(object->GetCapacity(), 20UL) << "unexpected Capacity.";
  EXPECT_EQ(object->GetElementsNum(), 20UL) << "unexpected Elements Number.";
  object = nullptr;
}

TEST_F(HysteresisIntervalTest, AutoRemoveObstacle2) {
  // intial hysteresis interval
  std::array<std::shared_ptr<Obstacle>, 30> obstacles;
  auto it_multi = obstacles_multi.begin();
  for (auto& item : obstacles) {
    item = *it_multi++;
  }
  ASSERT_TRUE(it_multi == obstacles_multi.end());
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // |ID    |1001|1002|1003|1004|1005|1006|1007|1008|1009|1010|
  // |Pos X |2.1 |2.2 |2.3 |2.4 |2.5 |2.6 |2.7 |2.8 |2.9 |3.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ----------------------------------------------------------
  // |ID    |1011|1012|1013|1014|1015|1016|1017|1018|1019|1020|
  // |Pos X |3.1 |3.2 |3.3 |3.4 |3.5 |3.6 |3.7 |3.8 |3.9 |4.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ----------------------------------------------------------
  // |ID    |1021|1022|1023|1024|1025|1026|1027|1028|1029|1030|
  // |Pos X |4.1 |4.2 |4.3 |4.4 |4.5 |4.6 |4.7 |4.8 |4.9 |5.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  object = std::make_shared<HysteresisInterval>(1.25, 0.5);  // [1.0, 1.5]
  std::array<int, 30> seq_nums{1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3,
                               4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6};
  object->SetCapacity(20UL);
  size_t pos = 0;
  for (const auto& item : obstacles) {
    HysteresisInterval::SetSequenceNum(seq_nums[pos]);
    object->HyValue(*item, item->speed());
    if (pos < 15) {
      EXPECT_EQ(object->GetElementsNum(), pos + 1UL)
          << "unexpected Elements Number at position:" << pos;
    } else if (pos < 20) {
      EXPECT_EQ(object->GetElementsNum(), pos - 4UL)
          << "unexpected Elements Number at position:" << pos;
    } else if (pos < 25) {
      EXPECT_EQ(object->GetElementsNum(), pos - 9UL)
          << "unexpected Elements Number at position:" << pos;
    } else {
      EXPECT_EQ(object->GetElementsNum(), pos - 14UL)
          << "unexpected Elements Number at position:" << pos;
    }
    ++pos;
  }
  EXPECT_EQ(object->GetCapacity(), 20UL) << "unexpected Capacity.";
  object = nullptr;
}

TEST_F(HysteresisIntervalTest, AutoRemoveObstacle3) {
  // intial hysteresis interval
  std::array<std::shared_ptr<Obstacle>, 30> obstacles;
  auto it_multi = obstacles_multi.begin();
  for (auto& item : obstacles) {
    item = *it_multi++;
  }
  ASSERT_TRUE(it_multi == obstacles_multi.end());
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // |ID    |1001|1002|1003|1004|1005|1006|1007|1008|1009|1010|
  // |Pos X |2.1 |2.2 |2.3 |2.4 |2.5 |2.6 |2.7 |2.8 |2.9 |3.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ----------------------------------------------------------
  // |ID    |1011|1012|1013|1014|1015|1016|1017|1018|1019|1020|
  // |Pos X |3.1 |3.2 |3.3 |3.4 |3.5 |3.6 |3.7 |3.8 |3.9 |4.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ----------------------------------------------------------
  // |ID    |1021|1022|1023|1024|1025|1026|1027|1028|1029|1030|
  // |Pos X |4.1 |4.2 |4.3 |4.4 |4.5 |4.6 |4.7 |4.8 |4.9 |5.0 |
  // |Real V|0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |0.8 |
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  object = std::make_shared<HysteresisInterval>(1.25, 0.5);  // [1.0, 1.5]
  std::array<int, 30> seq_nums{1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                               11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                               21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
  object->SetCapacity(20UL);
  size_t pos = 0;
  for (const auto& item : obstacles) {
    HysteresisInterval::SetSequenceNum(seq_nums[pos]);
    object->HyValue(*item, item->speed());
    if (pos < 10) {
      EXPECT_EQ(object->GetElementsNum(), pos + 1UL)
          << "unexpected Elements Number at position:" << pos;
    } else if (pos < 15) {
      EXPECT_EQ(object->GetElementsNum(), pos - 4UL)
          << "unexpected Elements Number at position:" << pos;
    } else if (pos < 20) {
      EXPECT_EQ(object->GetElementsNum(), pos - 9UL)
          << "unexpected Elements Number at position:" << pos;
    } else if (pos < 25) {
      EXPECT_EQ(object->GetElementsNum(), pos - 14UL)
          << "unexpected Elements Number at position:" << pos;
    } else {
      EXPECT_EQ(object->GetElementsNum(), pos - 19UL)
          << "unexpected Elements Number at position:" << pos;
    }
    ++pos;
  }
  EXPECT_EQ(object->GetCapacity(), 20UL) << "unexpected Capacity.";
  object = nullptr;
}

}  // namespace planning
}  // namespace century
