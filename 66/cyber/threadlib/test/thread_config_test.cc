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

TEST(Threadlib, config_parse_test) {
  ThreadlibConfData::Instance()->SetProcessGroup("threadlib_example");
  ThreadlibConfData::Instance()->ParseThreadlibConf();
  auto increment_thread_conf =
      ThreadlibConfData::Instance()->GetUserThreadConfig().at("increment");
  EXPECT_EQ(increment_thread_conf.name(), "increment");
  EXPECT_EQ(increment_thread_conf.cpuset(), "1");
  EXPECT_EQ(increment_thread_conf.policy(), "SCHED_FIFO");
  EXPECT_EQ(increment_thread_conf.prio(), 19);
}

TEST(Threadlib, thread_attribute_test) {
  ThreadlibConfData::Instance()->SetProcessGroup("threadlib_example");
  ThreadlibConfData::Instance()->ParseThreadlibConf();
  Thread th(
      "increment",
      [](int num) {
        while (num < 100) {
          ++num;
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      },
      0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  cpu_set_t cpuset_real, cpuset_expect;
  CPU_ZERO(&cpuset_real);
  CPU_ZERO(&cpuset_expect);
  CPU_SET(1, &cpuset_expect);
  pthread_getaffinity_np(th.NativeHandle(), sizeof(cpuset_real), &cpuset_real);
  EXPECT_NE(CPU_EQUAL(&cpuset_real, &cpuset_expect), 0);

  int policy;
  struct sched_param sp;
  pthread_getschedparam(th.NativeHandle(), &policy, &sp);
  EXPECT_EQ(policy, SCHED_FIFO);
  EXPECT_EQ(sp.sched_priority, 19);
  th.Join();
}

TEST(Threadlib, invalid_conf_test) {
  ThreadlibConfData::Instance()->SetProcessGroup("empty");
  ThreadlibConfData::Instance()->ParseThreadlibConf();
  thread th(
      "increment",
      [](int num) {
        while (num < 100) {
          ++num;
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      },
      0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  int policy;
  struct sched_param sp;
  pthread_getschedparam(th.NativeHandle(), &policy, &sp);
  EXPECT_EQ(policy, SCHED_OTHER);
  EXPECT_EQ(sp.sched_priority, 0);
  th.Join();
}

}  // namespace cyber
}  // namespace century
