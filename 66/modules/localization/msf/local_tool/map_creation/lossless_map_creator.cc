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

#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/common/util/extract_ground_plane.h"
#include "modules/localization/msf/common/util/file_utility.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map.h"
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_pool.h"

const unsigned int CAR_SENSOR_LASER_NUMBER = 64;

using century::localization::msf::FeatureXYPlane;
using century::localization::msf::pyramid_map::MapNodeIndex;
using century::localization::msf::pyramid_map::PyramidMap;
using century::localization::msf::pyramid_map::PyramidMapConfig;
using century::localization::msf::pyramid_map::PyramidMapMatrix;
using century::localization::msf::pyramid_map::PyramidMapNode;
using century::localization::msf::pyramid_map::PyramidMapNodePool;
typedef century::localization::msf::FeatureXYPlane::PointT PclPointT;
typedef century::localization::msf::FeatureXYPlane::PointCloudT PclPointCloudT;
typedef century::localization::msf::FeatureXYPlane::PointCloudPtrT
    PclPointCloudPtrT;

bool ParseCommandLine(int argc, char* argv[],
                      boost::program_options::variables_map* vm) {
  boost::program_options::options_description desc("Allowd options");
  desc.add_options()("help", "product help message")(
      "use_plane_inliers_only",
      boost::program_options::value<bool>()->required(),
      "use plane inliers only")
      // ("use_plane_fitting_ransac",
      // boost::program_options::value<bool>()->required(),
      //  "use plane fitting ransac")
      ("pcd_folders",
       boost::program_options::value<std::vector<std::string>>()
           ->multitoken()
           ->composing()
           ->required(),
       "pcd folders(repeated)")(
          "pose_files",
          boost::program_options::value<std::vector<std::string>>()
              ->multitoken()
              ->composing()
              ->required(),
          "pose files(repeated)")(
          "map_folder",
          boost::program_options::value<std::string>()->required(),
          "map folder")(
          "zone_id", boost::program_options::value<int>()->required(),
          "zone id")("coordinate_type",
                     boost::program_options::value<std::string>()->required(),
                     "coordinate type: UTM or LTM")(
          "map_resolution_type",
          boost::program_options::value<std::string>()->required(),
          "map resolution type: single or multi")(
          "resolution",
          boost::program_options::value<float>()->default_value(0.125),
          "optional: resolution for single resolution generation, default: "
          "0.125");
  try {
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, desc), *vm);
    if (vm->count("help")) {
      AERROR << desc;
      return false;
    }
    boost::program_options::notify(*vm);
  } catch (std::exception& e) {
    AERROR << "Error: " << e.what() << " " << desc;
    return false;
  } catch (...) {
    AERROR << "Unknown error!";
    return false;
  }
  return true;
}

void VarianceOnline(double* mean, double* var, unsigned int* N, double x) {
  ++(*N);
  double value = (x - (*mean)) / (*N);
  double v1 = x - (*mean);
  (*mean) += value;
  double v2 = x - (*mean);
  (*var) = (((*N) - 1) * (*var) + v1 * v2) / (*N);
}

using ::century::common::EigenAffine3dVec;
using ::century::common::EigenVector3dVec;

int main(int argc, char** argv) {
  return 0;
}
