#include "modules/perception/lidar/lib/detector/center_point_detection/centerpoint_detector.h"
#include <cuda_runtime_api.h>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/detector/dnn_common/common/check.hpp"
#include "modules/perception/base/singleton.h"


namespace century {
namespace perception {
namespace lidar {

using base::Object;
using base::PointD;
using base::PointF;

CenterPointDetector::CenterPointDetector() {
  checkCudaErrors(cudaStreamCreate(&stream_));
}

CenterPointDetector::~CenterPointDetector() {
  checkRuntime(cudaStreamDestroy(stream_));
}


base::ObjectSubType CenterPointDetector::GetObjectSubType(const int label) {
  switch (label) {
        case 0: return base::ObjectSubType::PEDESTRIAN;
        case 1: return base::ObjectSubType::CAR;
        case 2: return base::ObjectSubType::IGV_FULL;
        case 3: return base::ObjectSubType::TRUCK;
        case 4: return base::ObjectSubType::TRAILER_EMPTY;
        case 5: return base::ObjectSubType::TRAILER_FULL;
        case 6: return base::ObjectSubType::IGV_EMPTY;
        case 7: return base::ObjectSubType::CRANE;
        case 8: return base::ObjectSubType::OTHER_VEHICLE;
        case 9: return base::ObjectSubType::CONE;
        case 10: return base::ObjectSubType::CONTAINER_FORKLIFT;
        case 11: return base::ObjectSubType::FORKLIFT;
        case 12: return base::ObjectSubType::LORRY;  //WheelCrane
        case 13: return base::ObjectSubType::CONSTRUCTION_VEHICLE;
        case 14: return base::ObjectSubType::WHEELCRANE;
        default: return base::ObjectSubType::CAR;
  }
}


void CenterPointDetector::GetObjects(
    const std::vector<Bndbox>& bboxes,
    LidarFrame* frame) noexcept {
  int num_objects = bboxes.size();
  std::vector<std::shared_ptr<base::Object>> objects(num_objects);
  for (size_t i = 0; i < num_objects; i++) {
    std::shared_ptr<base::Object> object(new base::Object);

    object->id = i;

    float yaw = -bboxes[i].rt - M_PI / 2;
    // read params of bounding box
    float x = bboxes[i].x;
    float y = bboxes[i].y;
    float z = bboxes[i].z;
    float dx = bboxes[i].w;
    float dy = bboxes[i].l;
    float dz = bboxes[i].h;
     object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;

    object->size[0] = dx;
    object->size[1] = dy;
    object->size[2] = dz;

    object->center[0] = x;
    object->center[1] = y;
    object->center[2] = z;
    object->confidence = bboxes[i].score;

    auto roll = 0.f;
    auto pitch = 0.f;
    Eigen::Quaternionf quater =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();
    for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
      for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
        for (float vz : std::vector<float>{0, dz}) {
          Eigen::Vector3f v3f(vx, vy, vz);
          v3f = affine3f * v3f;
          base::PointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->lidar_supplement.cloud.push_back(point);
          //  base::PointD ptd{point.x , point.y , point.z};
          //  object->polygon.push_back(ptd);
          base::PointD ptd;
          ptd.x = point.x;
          ptd.y = point.y;
          ptd.z = point.z;
          object->lidar_supplement.cloud_world.push_back(ptd);
        }
      }
    }

    // classification
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());

    object->sub_type = GetObjectSubType(bboxes[i].id);

    // Mapping real label to type
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
    objects[i] = object;
  }

  {
    auto* mutex = base::Singleton<std::mutex>::GetInstance();
    std::lock_guard<std::mutex> lock(*mutex);
    frame->segmented_objects.insert(frame->segmented_objects.end(),
                                     objects.begin(), objects.end());
  }


  // for (size_t i = 0; i < num_objects; i++) {
  //   frame->segmented_objects.push_back(objects[i]);
  // }
  return;
}



bool CenterPointDetector::Init(const LidarDetectorInitOptions& options) {
  std::string config_file;
  if(options.cfg_file.empty()) {
    auto* config_manager = lib::ConfigManager::Instance();
    const lib::ModelConfig* model_config = nullptr;
    ACHECK(
        config_manager->GetModelConfig("PointCloudPreprocessor", &model_config));
    const std::string work_root = config_manager->work_root();

    config_file = cyber::common::GetAbsolutePath(work_root, "conf/perception/lidar");
  } else {
    config_file = options.cfg_file;
  }
  config_file =
      cyber::common::GetAbsolutePath(config_file, "centerpoint_detection.pb.txt");
  
  ACHECK(
      cyber::common::GetProtoFromFile(config_file, &centerpoint_detection_config_))
       << ", config_file: " << config_file;

  centerpoint_ptr.reset(new CenterPointVoxel(centerpoint_detection_config_));
  max_points_ = centerpoint_detection_config_.max_points();
  feature_number_ = centerpoint_detection_config_.feature_num();
  input_raw_.resize(max_points_ * feature_number_);
  checkCudaErrors(cudaMalloc((void **)&d_points_, max_points_ * feature_number_ * sizeof(float)));
  return true;
}

int loadData2(const char *file, void **data, unsigned int *length)
{
    std::fstream dataFile(file, std::ifstream::in);

    if (!dataFile.is_open()) {
        std::cout << "Can't open files: "<< file<<std::endl;
        return -1;
    }

    unsigned int len = 0;
    dataFile.seekg (0, dataFile.end);
    len = dataFile.tellg();
    dataFile.seekg (0, dataFile.beg);

    char *buffer = new char[len];
    if (buffer==NULL) {
        std::cout << "Can't malloc buffer."<<std::endl;
        dataFile.close();
        exit(EXIT_FAILURE);
    }

    dataFile.read(buffer, len);
    dataFile.close();

    *data = (void*)buffer;
    *length = len;
    return 0;  
}

bool CenterPointDetector::Detect(const LidarDetectorOptions& options,
                                  LidarFrame* frame) {

  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }

  auto points_size = frame->raw_cloud->size();
  original_cloud_ = frame->raw_cloud;

  if (points_size <= 0) {
    return false;
  }
  // std::string file_name = "test" + std::to_string(count) + ".bin";
  // std::ofstream bin_file(file_name, std::ios::out | std::ios::binary);
  // if (!bin_file.is_open()) {
  //   std::cerr << "Failed to open file for writing: " << file_name << std::endl;
  // }

  int use_ind = 0;
  for (auto ptx = 0u; ptx < points_size; ++ptx) {
    auto& point = frame->raw_cloud->at(ptx);
    if (!std::isnan(point.x) && !std::isnan(point.y) && 
        !std::isnan(point.z) && !std::isnan(point.intensity)) {
        input_raw_[use_ind * feature_number_ + 0] = point.x;
        input_raw_[use_ind * feature_number_ + 1] = point.y;
        input_raw_[use_ind * feature_number_ + 2] = point.z;
        input_raw_[use_ind * feature_number_ + 3] = point.intensity;
        use_ind ++;
        // bin_file.write(
        // reinterpret_cast<const char*>(&(point.x)),
        // sizeof(float));
        // bin_file.write(
        //     reinterpret_cast<const char*>(&(point.y)),
        //     sizeof(float));
        // bin_file.write(
        //     reinterpret_cast<const char*>(&(point.z)),
        //     sizeof(float));
        // bin_file.write(reinterpret_cast<const char*>(
        //                   &(point.intensity)),
        //               sizeof(float));

    }

  }
  // bin_file.close();

  AINFO << "start centerpoint inference points_size : " << points_size;
    
 

  // unsigned int length = 0;
  // void *pc_data = NULL;
  // loadData2("/century/data/kl_test/pointcloud.bin" , &pc_data, &length);
  // size_t points_num = length / (4 * sizeof(float)) ;
  // std::cout << "points_num : " << points_num << std::endl;

  checkCudaErrors(cudaMemset(d_points_, 0, max_points_ * feature_number_ * sizeof(float)));
  checkCudaErrors(cudaMemcpy(d_points_, input_raw_.data(), use_ind * feature_number_ * sizeof(float), cudaMemcpyHostToDevice));
  
  int result = centerpoint_ptr->doinfer((float*)d_points_,
                                          use_ind, stream_);
  if (result != 0) {
    AERROR << "centerpoint inference failed : " << use_ind;
  }

  // file_name = "test" + std::to_string(count) + ".txt";
  // std::ofstream ofs;
  // ofs.open(file_name, std::ios::out);
  // ofs.setf(std::ios::fixed, std::ios::floatfield);
  // ofs.precision(5);
  // if (ofs.is_open()) {
  //     for (const auto box : centerpoint_ptr->getbboxes()) {
  //       ofs << box.x << " ";
  //       ofs << box.y << " ";
  //       ofs << box.z << " ";
  //       ofs << box.w << " ";
  //       ofs << box.l << " ";
  //       ofs << box.h << " ";
  //     //   ofs << box.vx << " ";
  //     //   ofs << box.vy << " ";
  //       ofs << -box.rt - M_PI / 2  << " ";
  //       ofs << box.id << " ";
  //       ofs << box.score << " ";
  //       ofs << "\n";
  //     }
  // }
  // ofs.close();
  GetObjects(centerpoint_ptr->getbboxes(), frame);
  std::cout  << "pointpillar inference success : " << (centerpoint_ptr->getbboxes().size()) << std::endl;
  // count ++;
  return true;
}

PERCEPTION_REGISTER_LIDARDETECTOR(CenterPointDetector);
}
}
}