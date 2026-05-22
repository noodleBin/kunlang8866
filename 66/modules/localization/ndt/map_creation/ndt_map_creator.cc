/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
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

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/random.hpp>

#include "absl/strings/str_cat.h"

#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/common/util/extract_ground_plane.h"
#include "modules/localization/msf/common/util/system_utility.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map.h"
#include "modules/localization/msf/local_pyramid_map/ndt_map/ndt_map_pool.h"

using ::century::common::EigenAffine3dVec;
using ::century::common::EigenVector3dVec;

int main(int argc, char** argv) {
  return 0;
}
