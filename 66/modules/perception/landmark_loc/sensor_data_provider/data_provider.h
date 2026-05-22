// Created by xiaxinrong on 2025/8/15.
#pragma once
#include "cyber/common/file.h"
#include "cyber/cyber.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/lidar/proto/config.pb.h"
#include "modules/perception/base/point_cloud.h"
#include <mutex>
#include "modules/localization/proto/localization.pb.h"
#include "common/datatypes.h"
#include "modules/perception/landmark_loc/segmentation/src/image_segmentation.h"
#include "modules/localization/proto/msf_reset.pb.h"
#include <list>
#include <array>
#include <atomic>
#include <memory>
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/perception/landmark_loc/imu_fusion/async_delayed_measurement_eskf.h"
namespace landmark_loc {
using namespace semantic_mapping;
using namespace century;
using namespace century::perception;
using namespace century::localization;
typedef pcl::PointXYZL PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
//#define IMAGE_COMPRESSED_USE

#ifdef IMAGE_COMPRESSED_USE
using ImageFormat = drivers::CompressedImage;
#else
using ImageFormat = drivers::Image;
#endif

using PointCloudTypePtr = base::PointXYZIRTFCloudPtr;
using IndicesPtr = pcl::IndicesPtr;

using PointCloudTypePtr = base::PointXYZIRTFCloudPtr;
using PointXYZIRTFCloud = base::PointXYZIRTFCloud;

using LocalizationReaderPtr =
    std::shared_ptr<century::cyber::Reader<LocalizationEstimate>>;
using LocalizationPtr = std::shared_ptr<LocalizationEstimate>;
using LocalizationWriterPtr =
    std::shared_ptr<century::cyber::Writer<LocalizationEstimate>>;
using OdomResetWriterPtr =
    std::shared_ptr<century::cyber::Writer<MsfReset>>;
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

class DataProvider {
 public:
  DataProvider() = default;
  virtual ~DataProvider() = default;
  bool Init(const std::string& process_name,
            const std::unordered_set<CameraType> camera_types_register,
            const std::unordered_set<LidarType> lidar_types_register,
            const double& seg_thres, const bool speed_up, const bool need_recorder,
            const bool enable_odom_receive,
            const bool using_imu_chassis);
  semantic_mapping::CameraParmeter ProposeCameraParmeter() const;
  bool FetchBundleData(double& ts, 
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                       std::unordered_map<CameraType, cv::Mat>& images, 
                       Eigen::Matrix4d& closest_pose, Eigen::Matrix4d& closest_dr);
  bool PublishOdomReset();
  
  private:
  bool GetClosestPose(Eigen::Matrix4d& object_synced_state, double& ts, const std::map<double, Eigen::Matrix4d>& receive_pose);
  std::array<std::shared_ptr<cv::Mat>, CamTypeSize> GetClosestImages(const double timestamp);
  bool CalculateTargetTs(const std::set<double>& lidar_ts, const std::set<double>& cam_ts, const std::map<double, Eigen::Matrix4d>& pose_ts,
                           double& loc_ts, double& sensor_ts); 
  void OnReceiveImage(
    const std::shared_ptr<ImageFormat>& compressed_image, const CameraType& cam_type) ;
  void OnReceiveLidar(
      const std::shared_ptr<drivers::PointCloudPacked>& lidar_data, const LidarType& lidar_type) ;
  void OnReceiveLocCallback(const LocalizationPtr& localization_msg);
  void OnReceiveDRCallback(const LocalizationPtr& localization_msg);
  void OnReceiveChassisCallback(const std::shared_ptr<canbus::Chassis>& chassis_msg);
  void OnReceiveImuCallback(const std::shared_ptr<localization::CorrectedImu>& imu_msg);
  bool RegisterAllReceivers();
  bool ReadImuExtrinsic();
  bool ReadIntrinsic(const CameraType cam_type);
  bool ReadExtrinsic(const CameraType cam_type);
  bool ReadLidarExtrinsic();
  void SendOutPose(const LocalizationPtr& odom_msg, const double req_ts);
  std::unique_ptr<century::cyber::Node> node_ = nullptr;
  LocalizationWriterPtr pose_publisher_ = nullptr;
  OdomResetWriterPtr odom_reset_publisher_ = nullptr;
  #if 0
  century::perception::onboard::TransformWrapper lidar2vehicle_trans_;
  century::perception::onboard::TransformWrapper camera2vehicle_trans_;
  #endif
  std::array<Eigen::Affine3d, LidarTypeSize> lidar2vehicle_matrix_;
  std::unordered_set<CameraType>  camera_types_register_;
  std::unordered_set<LidarType>  lidar_types_register_;
  Eigen::Quaterniond camera_q_[CamTypeSize];
  Eigen::Vector3d camera_t_[CamTypeSize];
  cv::Mat camera_intrinsic_[CamTypeSize];
  cv::Mat camera_distort_[CamTypeSize];
  std::mutex pcl_mutex_;
  std::map<double, std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr, LidarTypeSize>> receive_pcl_;

  std::mutex camera_mutex_;
  std::map<double, std::array<std::shared_ptr<cv::Mat>, CamTypeSize>> receive_images_;

  std::mutex pose_mutex_;
  std::map<double, Eigen::Matrix4d> receive_pose_;

  std::mutex dr_mutex_;
  std::map<double, Eigen::Matrix4d> receive_dr_;

  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::CorrectedImu>> corrected_imu_listener_ = nullptr;


  std::array<std::shared_ptr<cyber::Reader<ImageFormat>>, CamTypeSize> cam_reader_;
  std::array<std::shared_ptr<cyber::Reader<drivers::PointCloudPacked>>, LidarTypeSize> lidar_reader_;
  LocalizationReaderPtr loc_reader_;
  LocalizationReaderPtr dr_reader_;  // ts align with rtk loc
  //config_file_path: "/century/modules/perception/production/conf/perception/lidar/helios_rear_right_conf.pb.txt"
  const std::string chassis_topic = "/century/canbus/chassis";
  const std::string imu_topic = "/century/sensor/gnss/corrected_imu";
  const std::string lidar_config_path = "/century/modules/perception/production/conf/perception/lidar/";
  const std::array<std::string , LidarTypeSize> lidar_name = {"helios_front_left_conf", "helios_rear_right_conf",
                                                              "bp_front_left" ,"bp_rear_right" };
 // const std::string cam_config_path = "/century/modules/perception/production/conf/perception/camera/";
 // const std::string intrinsic_file_path = "/century/modules/perception/production/conf/perception/camera/";
  const std::string cam_config_path = "/century/data/data/parameter/";
  const std::string intrinsic_file_path = "/century/data/data/parameter/";
  const std::array<std::string , CamTypeSize> cam_name = {"FrontLeft", "FrontMiddle", "FrontRight",
                                                          "RearRight", "RearMiddle", "RearLeft" };
  const std::string intrinsic_postfix = "_param.xml";
  const std::string extrinsic_postfix = "_transform.pb.txt";
  const std::array<std::string, CamTypeSize> cam_topic = {
    #ifdef IMAGE_COMPRESSED_USE
      "/century/sensor/camera/left_front/image/compressed",
      "/century/sensor/camera/front/image/compressed",
      "/century/sensor/camera/right_front/image/compressed",
      "/century/sensor/camera/right_rear/image/compressed",
      "/century/sensor/camera/rear/image/compressed",
      "/century/sensor/camera/left_rear/image/compressed"
    #else
      "/century/sensor/camera/left_front/image",
      "/century/sensor/camera/front/image",
      "/century/sensor/camera/right_front/image",
      "/century/sensor/camera/right_rear/image",
      "/century/sensor/camera/rear/image",
      "/century/sensor/camera/left_rear/image"
    #endif
  };
  const std::array<std::string, LidarTypeSize> lidar_topic = {
   "/lidar/helios/front_left",
   "/lidar/helios/rear_right",
   "/lidar/bp/front_left",
   "/lidar/bp/rear_right",
  };
  const std::string odom_reset_topic = "/century/loc/msf_reset";
  const std::string loccalization_publish_topic =  "/century/loc/amcl_pose";
  const std::string localization_topic =  "/century/loc/pose";
  const std::string received_dr_topic =  "/century/loc/fusion_pose";
  const std::string labelsPath = "/century/modules/perception/landmark_loc/segmentation/onnx/models/best.names";       // Path to class labels
  const std::string modelPath  = "/century/modules/perception/landmark_loc/segmentation/onnx/models/best.onnx";     // Path to YOLO11 model                                       
  std::array<std::unique_ptr<ImageSegmentation>, CamTypeSize> segmentors_;
  double last_ts_ = 0.0;
  const double ground_height_thres_ = 0.2;
  const uint32_t sensor_data_buff_size_ = 20;
  const uint32_t pose_data_buff_size_ = 200;
  bool need_recorder_ = false;
  bool enable_odom_receive_ = false;
  bool using_imu_chassis_ = false;
  std::unique_ptr<delayed_measurement_eskf::AsyncDelayedMeasurementEskf> eskf_;
  // IMU-to-vehicle extrinsic (2D yaw only, from ins_calib.yaml tf_vehicle_imu)
  double imu_to_vehicle_yaw_rad_ = 0.0;
  double imu_to_vehicle_cos_ = 1.0;
  double imu_to_vehicle_sin_ = 0.0;
  static constexpr float kZeroSpeedThreshold = 1e-3f;
  std::atomic<double> latest_dr_ts_{0.0};
  std::atomic<double> latest_gt_ts_{0.0};
};
}
