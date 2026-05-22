/******************************************************************************
 * Copyright 2020 The Century Authors. All Rights Reserved.
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

#include "modules/v2x/v2x_proxy/app/v2x_proxy.h"

#include <iostream>
#include <memory>

#include <google/protobuf/text_format.h>

#include "gtest/gtest.h"

#include "modules/v2x/v2x_proxy/app/utils.h"

TEST(V2xProxy, V2xProxy) {
  ::century::cyber::Init("TestCase");
  auto hdmap = std::make_shared<::century::hdmap::HDMap>();
  hdmap->LoadMapFromFile(century::hdmap::BaseMapFile());
  ::century::v2x::V2xProxy v2x_proxy(hdmap);
  EXPECT_TRUE(v2x_proxy.InitFlag());
  sleep(1);
  v2x_proxy.stop();
  ::century::cyber::Clear();
}
