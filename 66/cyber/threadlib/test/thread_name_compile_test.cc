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

TEST(Threadlib, name_compile_test) {
  // const char []
  const char name1[] = "test1";
  century::cyber::Thread th1(name1, []() {});
  th1.Join();
  EXPECT_EQ(th1.GetName(), name1);

  // char*
  char* name2 = "test2";
  century::cyber::Thread th2(name2, []() {});
  th2.Join();
  EXPECT_EQ(th2.GetName(), name2);

  // const char*
  const char* name3 = "test3";
  century::cyber::Thread th3(name3, []() {});
  th3.Join();
  EXPECT_EQ(th3.GetName(), name3);

  // std::string
  std::string name4 = "test4";
  century::cyber::Thread th4(name4, []() {});
  th4.Join();
  EXPECT_EQ(th4.GetName(), name4);

  // std::string&&
  std::string name5 = "test5";
  century::cyber::Thread th5(std::move(name5), []() {});
  th5.Join();
  EXPECT_EQ(th5.GetName(), "test5");

  // const std::string&
  const std::string& name6 = "test6";
  century::cyber::Thread th6(name6, []() {});
  th6.Join();
  EXPECT_EQ(th6.GetName(), name6);

  // int
  century::cyber::Thread th7(1, []() {});
  th7.Join();
  EXPECT_EQ(th7.GetName(), "");

  // float
  century::cyber::Thread th8(1.0f, []() {});
  th8.Join();
  EXPECT_EQ(th8.GetName(), "");

  // double
  century::cyber::Thread th9(1.0, []() {});
  th9.Join();
  EXPECT_EQ(th9.GetName(), "");
}

}  // namespace cyber
}  // namespace century
