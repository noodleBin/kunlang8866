#include <queue>
#include <mutex>
#include <condition_variable>
#include <list>
#include <memory>
#include <iostream>
#include "modules/perception/lidar_tracking/lidar_tracking_component.h"
#include "modules/localization/proto/localization.pb.h"
#include "cyber/common/file.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <array>
#include <sys/stat.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"
#include <fstream>

using namespace century::perception;
using century::localization::LocalizationEstimate;
using LocalizationReaderPtr =
    std::shared_ptr<century::cyber::Reader<LocalizationEstimate>>;
using LocalizationPtr = std::shared_ptr<LocalizationEstimate>;
using namespace century;
using namespace century::perception::lidar;

const std::array<std::string, 6> camera_names = {
    "FrontLeft", "FrontMiddle", "FrontRight",
    "RearLeft", "RearMiddle", "RearRight"
};

const std::array<std::string, 4> lidar_names = {
  "FrontLeft",  "RearRight", "HelisFrontLeft", "HelisRearRight"
};
const std::string data_folder = "/century/data/log/";
const int start_pos = 6; 
void ShowImages(
  const std::shared_ptr<drivers::CompressedImage>& compressed_image,
  const std::string& label) noexcept {
  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                            compressed_image->data().end());
  cv::Mat mat_image = cv::imdecode(compressed_raw_data, cv::IMREAD_COLOR);
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(6) << compressed_image->header().timestamp_sec();;
  std::string str_ts = timestamp_stream.str().substr(start_pos);
  const std::string folder_path = data_folder + label + "/";
  cv::imwrite(folder_path + str_ts + ".jpg", mat_image);
  return;
}
void OnFrontLeftReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(0));  
}

void OnFrontMidReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(1)); 
}

void OnFrontRightReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(2));
}

void OnRearLeftReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(3));
}

void OnRearMidReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(4));
}

void OnRearRightReceiveImage(
    const std::shared_ptr<drivers::CompressedImage>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(5));
}

void OnLidarReceive(const std::shared_ptr<drivers::PointXYZIRTCloud>& raw_cloud, const std::string & lidar_name, const Eigen::Affine3d& transform) {
  const float kPointInfThreshold = 1e2;
  
  base::PointXYZIRTF point;
  PointCloudTypePtr input_cloud = base::PointXYZIRTFCloudPool::Instance().Get();
  for (int i = 0; i < raw_cloud->point_size(); ++i) {
    const century::drivers::PointXYZIRT& pt = raw_cloud->point(i);
   
    if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
      continue;
    }
    if (fabs(pt.x()) > kPointInfThreshold ||
        fabs(pt.y()) > kPointInfThreshold ||
        fabs(pt.z()) > kPointInfThreshold) {
      continue;
    }
    
    Eigen::Vector3d vec3d_lidar(pt.x(), pt.y(), pt.z());
    Eigen::Vector3d vec3d_imu = transform * vec3d_lidar;
    
    point.x = vec3d_imu(0);
    point.y = vec3d_imu(1);
    point.z = vec3d_imu(2);
    point.intensity = static_cast<float>(pt.intensity());
    point.ring = pt.ring();
    input_cloud->push_back(
        point, static_cast<double>(pt.timestamp()) * 1e-9,
        std::numeric_limits<float>::max(), i, 0);
  }

  input_cloud->resize(input_cloud->size());

  

  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(6) << raw_cloud->header().timestamp_sec();
  std::string str_ts = timestamp_stream.str().substr(6);
  std::string folder_name = data_folder + "/"+ lidar_name +"_pcl/" + str_ts;
 // std::replace(folder_name.begin(), folder_name.end(), '.', '_');
 
  bool dump_txt = false;
  std::ofstream* file_name_stream = nullptr;
  if(dump_txt) {
    const std::string file_name = folder_name +".txt";
    file_name_stream = new std::ofstream(
      file_name, std::ios::trunc);
  }
  
  folder_name = folder_name + ".pcd";
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto& item : *input_cloud) {
    if(item.z < 0.2) {
     // continue;
    }
    cloud->points.emplace_back(item.x, item.y, item.z, item.intensity);
    if(file_name_stream) {
      *file_name_stream << item.x <<","<< item.y <<","<< item.z <<","<< item.intensity << std::endl;
    }
   
  }
  if(file_name_stream) {
    file_name_stream->close();
  }
  
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (pcl::io::savePCDFileASCII(folder_name, *cloud) != 0) {
    AERROR << "Failed to save " << folder_name << std::endl;
  }
  return;
}



void OnReceiveLocCallback(const LocalizationPtr& localization_msg) {
  static std::ofstream of(data_folder +"/localization.txt", std::ios::app);
  std::string ts_str = std::to_string(localization_msg->header().timestamp_sec());
  of << std::fixed << std::setprecision(6)
     << ts_str.substr(start_pos) << " " 
     << localization_msg->pose().position().x() << " " 
     << localization_msg->pose().position().y() << " " 
     << localization_msg->pose().position().z() << " "
     << localization_msg->pose().orientation().qx() << " "
     << localization_msg->pose().orientation().qy() << " "
     << localization_msg->pose().orientation().qz() << " "
     << localization_msg->pose().orientation().qw() << std::endl;
}

bool EnsureDirectory(const std::string& path) {

  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    if (info.st_mode & S_IFDIR) {
     // AINFO << "Folder already exists: " << path << std::endl;
      return true;
    } else {
      AERROR << "Path exists but is not a directory: " << path << std::endl;
      return false;
    }
  } else {
    if (mkdir(path.c_str(), 0755) == 0) {
      AINFO << "Folder created successfully: " << path << std::endl;
      return true;
    } else {
      AERROR << "Failed to create folder: " << path << std::endl;
      return false;
    }
  }
}

Eigen::Matrix3d QuaternionToRotationMatrix(const double x, const double y, const double z, const double w) {

  Eigen::Matrix3d rotationMatrix;
  rotationMatrix(0, 0) = 1 - 2 * y * y - 2 * z * z;
  rotationMatrix(0, 1) = 2 * x * y - 2 * w * z;
  rotationMatrix(0, 2) = 2 * x * z + 2 * w * y;

  rotationMatrix(1, 0) = 2 * x * y + 2 * w * z;
  rotationMatrix(1, 1) = 1 - 2 * x * x - 2 * z * z;
  rotationMatrix(1, 2) = 2 * y * z - 2 * w * x;

  rotationMatrix(2, 0) = 2 * x * z - 2 * w * y;
  rotationMatrix(2, 1) = 2 * y * z + 2 * w * x;
  rotationMatrix(2, 2) = 1 - 2 * x * x - 2 * y * y;

  return rotationMatrix;
}

std::unordered_map<std::string, Eigen::Affine3d> FetchLidarExtrinsic(const std::string extrinsic_file = 
    "/century/modules/perception/lidar_tracking/tools/lidar_extrinsic.txt") {
  
  std::unordered_map<std::string, Eigen::Affine3d> transforms = {
    {lidar_names.at(0), Eigen::Affine3d::Identity()},
    {lidar_names.at(1), Eigen::Affine3d::Identity()},
    {lidar_names.at(2), Eigen::Affine3d::Identity()},
    {lidar_names.at(3), Eigen::Affine3d::Identity()}
  };
  std::ifstream inputFile(extrinsic_file);
  if (!inputFile.is_open()) {
    std::cerr << "Error opening file: " << extrinsic_file << std::endl;
    exit(0);
  }

  std::string line;
  int n=0;
  while (getline(inputFile, line)) {

    std::string data[7];
    int i = 0;
    std::stringstream line_stream(line);
    while (std::getline(line_stream, data[i++], ',')) { 

    } 
    transforms.at(lidar_names.at(n)).translation() = Eigen::Vector3d(stod(data[0]),stod(data[1]), stod(data[2]));
    transforms.at(lidar_names.at(n)).rotate(QuaternionToRotationMatrix(stod(data[3]),stod(data[4]), stod(data[5]), stod(data[6])));
    std::cout<< std::fixed << std::setprecision(6)
             << data[0]<<" " << data[1]<<" " << data[2]<<" " << data[3]<<" " << data[4]<<" " << data[5]<<" " << data[6]<<std::endl;
  
    if(n++ == lidar_names.size()-1) {
      break;
    }
  }
  return transforms;
}

int main(int argc, char** argv) {
  century::cyber::Init(argv[0]);
  auto node = century::cyber::CreateNode(argv[0]);
  const std::string localization_channel_name =  "/century/loc/pose";
  LocalizationReaderPtr localization_reader_ = node->CreateReader<LocalizationEstimate>( localization_channel_name,
                    [](const LocalizationPtr& msg) { OnReceiveLocCallback(msg); });
  for(const auto& camera_name : camera_names) {
    const std::string folder_path = data_folder + camera_name + "/";
    if (!EnsureDirectory(folder_path)) {
      AERROR << "Failed to ensure directory: " << folder_path;
      return -1;
    }
  }
  for(const auto& lidar_name : lidar_names) {
    const std::string folder_path = data_folder + lidar_name + "_pcl/";
    if (!EnsureDirectory(folder_path)) {
      AERROR << "Failed to ensure directory: " << folder_path;
      return -1;
    }
  }

  auto transforms = FetchLidarExtrinsic();

  auto front_left_camera_reader =
      node->CreateReader<drivers::CompressedImage>("/century/sensor/camera/left_front/image/compressed",
        [](const std::shared_ptr<drivers::CompressedImage>&  msg) { OnFrontLeftReceiveImage(msg); }  );

  auto front_mid_camera_reader =
      node->CreateReader<drivers::CompressedImage>("/century/sensor/camera/front/image/compressed",
        [](const std::shared_ptr<drivers::CompressedImage>&  msg) { OnFrontMidReceiveImage(msg); }  );

  auto front_right_camera_reader =
      node->CreateReader<drivers::CompressedImage>("/century/sensor/camera/right_front/image/compressed",
        [](const std::shared_ptr<drivers::CompressedImage>&  msg) { OnFrontRightReceiveImage(msg); }  );

  auto rear_left_camera_reader =
      node->CreateReader<drivers::CompressedImage>("/century/sensor/camera/left_rear/image/compressed",
        [](const std::shared_ptr<drivers::CompressedImage>&  msg) { OnRearLeftReceiveImage(msg); }  );

  auto rear_mid_camera_reader = 
      node->CreateReader<drivers::CompressedImage>("/century/sensor/camera/rear/image/compressed", 
        [](const std::shared_ptr<drivers::CompressedImage>&  msg) { OnRearMidReceiveImage(msg); }  );

  auto rear_right_camera_reader =
      node->CreateReader<drivers::CompressedImage>("/century/sensor/camera/right_rear/image/compressed",
        [](const std::shared_ptr<drivers::CompressedImage>&  msg) { OnRearRightReceiveImage(msg); } );
  
  auto lidar_bp_front_left_reader = 
      node->CreateReader<drivers::PointXYZIRTCloud>("/lidar/bp/front_left",
        [&](const std::shared_ptr<drivers::PointXYZIRTCloud>& msg) { OnLidarReceive(msg, lidar_names.at(0), 
                                                                      transforms.at(lidar_names.at(0))); });

  auto lidar_bp_rear_right_reader = 
      node->CreateReader<drivers::PointXYZIRTCloud>("/lidar/bp/rear_right",
        [&](const std::shared_ptr<drivers::PointXYZIRTCloud>& msg) { OnLidarReceive(msg, lidar_names.at(1), 
                                                                    transforms.at(lidar_names.at(1))); });   
  auto lidar_helis_front_left_reader = 
      node->CreateReader<drivers::PointXYZIRTCloud>("/lidar/helios/front_left",
        [&](const std::shared_ptr<drivers::PointXYZIRTCloud>& msg) { OnLidarReceive(msg, lidar_names.at(2), 
                                                                      transforms.at(lidar_names.at(0))); });

  auto lidar_helis_rear_right_reader = 
      node->CreateReader<drivers::PointXYZIRTCloud>("/lidar/helios/rear_right",
        [&](const std::shared_ptr<drivers::PointXYZIRTCloud>& msg) { OnLidarReceive(msg, lidar_names.at(3), 
                                                                    transforms.at(lidar_names.at(1))); });   

  century::cyber::WaitForShutdown();
}