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

#include "gtest/gtest.h"

#include "cyber/threadlib/threadlib.h"

namespace century {
namespace cyber {

TEST(Threadlib, get_thread_attr_test) {
  ThreadlibConfData::Instance()->SetProcessGroup("threadlib_example");
  ThreadlibConfData::Instance()->ParseThreadlibConf();
  Thread th1(
      "increment",
      [](int num) {
        while (++num < 10) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      },
      0);
  auto thread_attr1 = th1.GetThreadAttribute();
  EXPECT_EQ(thread_attr1.cpuset, "1");
  EXPECT_EQ(thread_attr1.policy, "SCHED_FIFO");
  EXPECT_EQ(thread_attr1.prio, 19);
  th1.Join();

  Thread th2("unknown", []() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  });
  auto thread_attr2 = th2.GetThreadAttribute();
  EXPECT_EQ(thread_attr2.cpuset, "");
  EXPECT_EQ(thread_attr2.policy, "SCHED_OTHER");
  EXPECT_EQ(thread_attr2.prio, 0);
  th2.Join();
}

TEST(Threadlib, swap_test) {
  auto th1_name = "th1";
  Thread th1(
      th1_name,
      [](int num) {
        while (++num < 10) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      },
      0);

  auto th2_name = "th2";
  Thread th2(
      th2_name,
      [](int num) {
        while (++num < 10) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      },
      0);

  auto th1_orig_id = th1.GetId();
  auto th2_orig_id = th2.GetId();
  auto th1_orig_name = th1.GetName();
  auto th2_orig_name = th2.GetName();

  th1.Swap(th2);

  EXPECT_EQ(th1.GetId(), th2_orig_id);
  EXPECT_EQ(th2.GetId(), th1_orig_id);
  EXPECT_EQ(th1.GetName(), th2_orig_name);
  EXPECT_EQ(th2.GetName(), th1_orig_name);

  th1.Join();
  th2.Join();
}

TEST(Threadlib, move_test) {
  auto th1_name = "th1";
  Thread th1(
      th1_name,
      [](int num) {
        while (++num < 10) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      },
      0);
  auto th1_orig_id = th1.GetId();
  auto th2 = std::move(th1);

  EXPECT_EQ(th2.GetId(), th1_orig_id);
  EXPECT_EQ(th2.GetName(), th1_name);

  auto th3(std::move(th2));
  EXPECT_EQ(th3.GetId(), th1_orig_id);
  EXPECT_EQ(th3.GetName(), th1_name);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(th1.Joinable(), false);
  EXPECT_EQ(th1.GetName(), "");
  EXPECT_EQ(th2.Joinable(), false);
  EXPECT_EQ(th2.GetName(), "");

  Thread th4;
  th4 = std::move(th3);
  EXPECT_EQ(th1.Joinable(), false);
  EXPECT_EQ(th1.GetName(), "");
  EXPECT_EQ(th2.Joinable(), false);
  EXPECT_EQ(th2.GetName(), "");
  EXPECT_EQ(th3.Joinable(), false);
  EXPECT_EQ(th3.GetName(), "");
  EXPECT_EQ(th4.GetId(), th1_orig_id);
  EXPECT_EQ(th4.GetName(), th1_name);
  th4.Join();
}

}  // namespace cyber
}  // namespace century
