#include <queue>
#include <mutex>
#include <condition_variable>
#include <list>
#include <memory>
#include <iostream>
//#include "modules/perception/lidar_tracking/lidar_tracking_component.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/lidar/proto/config.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "cyber/common/file.h"
#include "cyber/cyber.h"
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
#include <unordered_map>
#define RAW_IMAGE

using namespace century::perception;
using century::localization::LocalizationEstimate;
using LocalizationReaderPtr =
    std::shared_ptr<century::cyber::Reader<LocalizationEstimate>>;
using LocalizationPtr = std::shared_ptr<LocalizationEstimate>;
using namespace century;
using namespace century::perception::lidar;
#ifdef RAW_IMAGE
using ImageFormat = drivers::Image;
#define LEFT_FRONT_IMAGE_TOPIC "/century/sensor/camera/left_front/image"
#define FRONT_IMAGE_TOPIC "/century/sensor/camera/front/image"
#define RIGHT_FRONT_IMAGE_TOPIC "/century/sensor/camera/right_front/image"
#define LEFT_REAR_IMAGE_TOPIC "/century/sensor/camera/left_rear/image"
#define REAR_IMAGE_TOPIC "/century/sensor/camera/rear/image"
#define RIGHT_REAR_IMAGE_TOPIC "/century/sensor/camera/right_rear/image"
#else
using ImageFormat = drivers::CompressedImage;
#define LEFT_FRONT_IMAGE_TOPIC "/century/sensor/camera/left_front/image/compressed"
#define FRONT_IMAGE_TOPIC "/century/sensor/camera/front/image/compressed"
#define RIGHT_FRONT_IMAGE_TOPIC "/century/sensor/camera/right_front/image/compressed"
#define LEFT_REAR_IMAGE_TOPIC "/century/sensor/camera/left_rear/image/compressed"
#define REAR_IMAGE_TOPIC "/century/sensor/camera/rear/image/compressed"
#define RIGHT_REAR_IMAGE_TOPIC "/century/sensor/camera/right_rear/image/compressed"
#endif


namespace {
#pragma pack(push, 1)
struct PointPacked {
private:
    float x_;
    float y_;
    float z_;
    uint16_t intensity_;
    uint16_t ring_;
    double timestamp_;

public:
    // Default constructor
    PointPacked() = default;

    // Parameterized constructor
    PointPacked(float x, float y, float z,
                uint16_t intensity, uint16_t ring,
                double timestamp)
        : x_(x), y_(y), z_(z),
          intensity_(intensity), ring_(ring),
          timestamp_(timestamp) {}

    // ---- Accessors (Getters) ----
    inline float x() const noexcept { return x_; }
    inline float y() const noexcept { return y_; }
    inline float z() const noexcept { return z_; }
    inline uint16_t intensity() const noexcept { return intensity_; }
    inline uint16_t ring() const noexcept { return ring_; }
    inline double timestamp() const noexcept { return timestamp_; }

    // ---- Modifiers (Setters) ----
    inline void x(float val) noexcept { x_ = val; }
    inline void y(float val) noexcept { y_ = val; }
    inline void z(float val) noexcept { z_ = val; }
    inline void intensity(uint16_t val) noexcept { intensity_ = val; }
    inline void ring(uint16_t val) noexcept { ring_ = val; }
    inline void timestamp(double val) noexcept { timestamp_ = val; }
};
#pragma pack(pop)

const std::array<std::string, 6> camera_names = {
    "FrontLeft", "FrontMiddle", "FrontRight",
    "RearLeft", "RearMiddle", "RearRight"
};

const std::array<std::string, 4> lidar_names = {
  "HelisFrontLeft", "HelisRearRight","FrontLeft",  "RearRight",
};
const std::string data_folder = "/century/data/log/";
const int start_pos = 2; 
std::atomic<bool> need_record_ts{true};
const std::unordered_map<std::string, int > camera_type_mapping ={
  {"FrontLeft", 0},
  {"FrontMiddle",1}, 
  {"FrontRight",2},
};
const std::unordered_map<std::string, int > lidar_type_mapping ={
  {"HelisFrontLeft",3},
  {"HelisRearRight",4},
  {"FrontLeft", 5}, 
  {"RearRight", 6}
};
const uint32_t type_localization = 7;
std::unordered_map<uint64_t, std::array<double, type_localization + 1>> ts_recoeder;
std::string localization_file;
}

void ShowImages(const std::shared_ptr<ImageFormat>& compressed_image,
                const std::string& label) noexcept {
  if((need_record_ts) && (camera_type_mapping.count(label) > 0)) {
    auto type = camera_type_mapping.at(label);
    uint64_t ts = static_cast<uint64_t>(std::round(compressed_image->header().timestamp_sec() * 10));

    ts_recoeder[ts][type] = std::round(std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch()
          ).count() / 100.0) / 10.0;
  }
  #ifdef RAW_IMAGE
  uint32_t width = compressed_image->width();
  uint32_t height = compressed_image->height();
  const std::string& encoding = compressed_image->encoding();
  const auto& data = compressed_image->data();

  if (0 == width || 0 == height || data.empty()) {
    AERROR << "empty image";
    return;
  }

  cv::Mat raw_image;
  if ("rgb8" == encoding) {
    raw_image = cv::Mat(
        height, width, CV_8UC3,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_RGB2BGR);
  } else if ("bgr8" == encoding) {
    raw_image = cv::Mat(
        height, width, CV_8UC3,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
  } else if ("rgba8" == encoding) {
    raw_image = cv::Mat(
        height, width, CV_8UC4,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_RGBA2BGR);
  } else if ("bgra8" == encoding) {
    raw_image = cv::Mat(
        height, width, CV_8UC4,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_BGRA2BGR);
  } else if ("mono8" == encoding) {
    raw_image = cv::Mat(
        height, width, CV_8UC1,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_GRAY2BGR);
  } else if ("yuyv" == encoding || "YUYV" == encoding ||
            "yuy2" == encoding || "YUY2" == encoding) {
    raw_image = cv::Mat(
        height, width, CV_8UC2,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(raw_image, raw_image, cv::COLOR_YUV2BGR_YUYV);
  } else if ("yuv420" == encoding || "i420" == encoding ||
            "I420" == encoding) {
    // YUV420 (I420) format: Y plane + U plane + V plane
    size_t y_size = width * height;
    size_t uv_size = (width / 2) * (height / 2);
    if (data.size() >= y_size + 2 * uv_size) {
      cv::Mat y_plane(
          height, width, CV_8UC1,
          const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
      cv::Mat u_plane(height / 2, width / 2, CV_8UC1,
                      const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(
                          data.data() + y_size)));
      cv::Mat v_plane(height / 2, width / 2, CV_8UC1,
                      const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(
                          data.data() + y_size + uv_size)));

      cv::Mat u_resized, v_resized;
      cv::resize(u_plane, u_resized, cv::Size(width, height), 0, 0,
                cv::INTER_LINEAR);
      cv::resize(v_plane, v_resized, cv::Size(width, height), 0, 0,
                cv::INTER_LINEAR);

      std::vector<cv::Mat> yuv_planes = {y_plane, u_resized, v_resized};
      cv::Mat yuv_image;
      cv::merge(yuv_planes, yuv_image);
      cv::cvtColor(yuv_image, raw_image, cv::COLOR_YUV2BGR);
    } else {
      AERROR << "Invalid YUV420 data size: " << data.size()
            << " expected: " << (y_size + 2 * uv_size);
      return;
    }
  } else {
    AERROR << "Unsupported image encoding: " << encoding << " for camera ";
    return;
  }

  if (raw_image.empty()) {
    AERROR << "Failed to create cv::Mat from raw image for camera ";
    return;
  }

  #else
  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                            compressed_image->data().end());
  cv::Mat raw_image = cv::imdecode(compressed_raw_data, cv::IMREAD_COLOR);
  #endif
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(1) << compressed_image->header().timestamp_sec();;
  std::string str_ts = timestamp_stream.str().substr(start_pos);
  const std::string folder_path = data_folder + label + "/";
  cv::imwrite(folder_path + str_ts + ".jpg", raw_image);
  return;
}



void OnFrontLeftReceiveImage(
    const std::shared_ptr<ImageFormat>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(0));  
}

void OnFrontMidReceiveImage(
    const std::shared_ptr<ImageFormat>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(1)); 
}

void OnFrontRightReceiveImage(
    const std::shared_ptr<ImageFormat>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(2));
}

void OnRearLeftReceiveImage(
    const std::shared_ptr<ImageFormat>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(3));
}

void OnRearMidReceiveImage(
    const std::shared_ptr<ImageFormat>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(4));
}

void OnRearRightReceiveImage(
    const std::shared_ptr<ImageFormat>& compressed_image) {
  ShowImages(compressed_image, camera_names.at(5));
}

void OnLidarReceive(const std::shared_ptr<drivers::PointCloudPacked>& raw_cloud, const std::string & lidar_name, const Eigen::Affine3d& transform) {
  if(need_record_ts) {
    auto type = lidar_type_mapping.at(lidar_name);
    uint64_t ts = static_cast<uint64_t>(std::round(raw_cloud->header().timestamp_sec() * 10));
  //  std::cout << "type: "<< type <<" ts: "<< ts;
    ts_recoeder[ts][type] = std::round(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                std::chrono::system_clock::now().time_since_epoch()
                                            ).count() / 100.0) / 10.0;
  }
  
  const float kPointInfThreshold = 1e2;
  std::vector<PointPacked> ppd( raw_cloud->point_size());
  const std::string& raw_data = raw_cloud->data();
  std::memcpy(ppd.data(), raw_data.data(), raw_data.size());
  base::PointXYZIRTF point;
  PointCloudTypePtr input_cloud = base::PointXYZIRTFCloudPool::Instance().Get();
  for (int i = 0; i < raw_cloud->point_size(); ++i) {
    const auto& pt = ppd[i];
   
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

    if(vec3d_imu(2) > 0.2) {
      continue;
    }
    
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
  timestamp_stream << std::fixed << std::setprecision(1) << raw_cloud->header().timestamp_sec();
  std::string str_ts = timestamp_stream.str().substr(start_pos);
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
  if(need_record_ts) {
    uint64_t ts = static_cast<uint64_t>(std::round(localization_msg->header().timestamp_sec() * 10));
    ts_recoeder[ts][type_localization] = std::round(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()
      ).count() / 100.0) / 10.0;
  }
  static std::ofstream of(data_folder + "/"+ localization_file +"localization.txt", std::ios::app);
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
    "/century/modules/perception/landmark_loc/tools/lidar_extrinsic.txt") {
  
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
  if(2 == argc) {
    localization_file = argv[1];
  }

  auto node = century::cyber::CreateNode(argv[0]);
  const std::string localization_channel_name =  "/century/loc/pose";
  
  LocalizationReaderPtr localization_reader_ = node->CreateReader<LocalizationEstimate>( localization_channel_name,
                    [](const LocalizationPtr& msg) { OnReceiveLocCallback(msg); });
  
  if(!localization_file.empty()) {
    century::cyber::WaitForShutdown();
    return 0;
  }
    

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
    node->CreateReader<ImageFormat>(LEFT_FRONT_IMAGE_TOPIC,
      [](const std::shared_ptr<ImageFormat>&  msg) { OnFrontLeftReceiveImage(msg); }  );

  auto front_mid_camera_reader =
    node->CreateReader<ImageFormat>(FRONT_IMAGE_TOPIC,
      [](const std::shared_ptr<ImageFormat>&  msg) { OnFrontMidReceiveImage(msg); }  );

  auto front_right_camera_reader =
    node->CreateReader<ImageFormat>(RIGHT_FRONT_IMAGE_TOPIC,
      [](const std::shared_ptr<ImageFormat>&  msg) { OnFrontRightReceiveImage(msg); }  );
  #if 0
  auto rear_left_camera_reader =
      node->CreateReader<ImageFormat>(LEFT_REAR_IMAGE_TOPIC,
        [](const std::shared_ptr<ImageFormat>&  msg) { OnRearLeftReceiveImage(msg); }  );

  auto rear_mid_camera_reader = 
        node->CreateReader<ImageFormat>(REAR_IMAGE_TOPIC, 
          [](const std::shared_ptr<ImageFormat>&  msg) { OnRearMidReceiveImage(msg); }  );

  auto rear_right_camera_reader =
        node->CreateReader<ImageFormat>(RIGHT_REAR_IMAGE_TOPIC,
          [](const std::shared_ptr<ImageFormat>&  msg) { OnRearRightReceiveImage(msg); } );
  #endif

 
  auto lidar_helios_front_left_reader = 
    node->CreateReader<drivers::PointCloudPacked>("/lidar/helios/front_left",
    [&](const std::shared_ptr<drivers::PointCloudPacked>& msg) { OnLidarReceive(msg, lidar_names.at(0), 
                                                                    transforms.at(lidar_names.at(0))); });
  auto lidar_helios_rear_right_reader = 
    node->CreateReader<drivers::PointCloudPacked>("/lidar/helios/rear_right",
    [&](const std::shared_ptr<drivers::PointCloudPacked>& msg) { OnLidarReceive(msg, lidar_names.at(1), 
                                                              transforms.at(lidar_names.at(1))); });   

  auto lidar_bp_front_left_reader = 
    node->CreateReader<drivers::PointCloudPacked>("/lidar/bp/front_left",
    [&](const std::shared_ptr<drivers::PointCloudPacked>& msg) { OnLidarReceive(msg, lidar_names.at(2), 
                                                                  transforms.at(lidar_names.at(2))); });
  auto lidar_bp_rear_right_reader = 
    node->CreateReader<drivers::PointCloudPacked>("/lidar/bp/rear_right",
    [&](const std::shared_ptr<drivers::PointCloudPacked>& msg) { OnLidarReceive(msg, lidar_names.at(3), 
                                                                    transforms.at(lidar_names.at(3))); });   

  if(need_record_ts) {
    std::cout <<"start to dump ts sync record"<<std::endl;
    sleep(30);
    need_record_ts = false;
    sleep(2);
    std::cout <<"start to write timer file"<<std::endl;
    const std::string file_name = "/century/data/timer.txt";
    std::ofstream file_name_stream(file_name, std::ios::trunc);
    for (const auto& kv : ts_recoeder) {
      const auto& ts = kv.second;
      for (size_t i = 0; i < ts.size(); ++i) {
        file_name_stream << std::fixed << std::setprecision(1) << kv.first << " "<< i <<" "<< ts[i] << std::endl;
      }
    }
    for (const auto& kv : ts_recoeder) {
      const auto& ts = kv.second;
      auto [min_it, max_it] = std::minmax_element(ts.begin(), ts.end());
      double diff = *max_it - *min_it;
      file_name_stream << "ts: "<< std::fixed << std::setprecision(1) << kv.first <<" min: "<< *min_it <<" max: "<< *max_it <<" diff: "<< diff <<"\n";
    }
    file_name_stream.close();
    std::cout <<"timer file write done"<<std::endl;
  }
  century::cyber::WaitForShutdown();
}
