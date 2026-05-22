// Created by xiaxinrong on 2025/8/15.
#include "modules/perception/landmark_loc/amcl_worker/recorder/amcl_recorder.h"
#include "modules/perception/landmark_loc/sensor_data_provider/data_provider.h"
#include "modules/perception/onboard/proto/lidar_component_config.pb.h"
#include "cyber/common/log.h"
#include "tinyxml2.h"
#include "common/pose_utils.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/base/object_pool_types.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include "modules/perception/landmark_loc/publisher/amcl_pose_publisher.h"
#include "cyber/time/clock.h"
#include "yaml-cpp/yaml.h"
namespace landmark_loc {
namespace {
  const uint32_t start_pos = 4;
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
}
bool DataProvider::ReadIntrinsic(const CameraType cam_type) {
  tinyxml2::XMLDocument doc;
  const std::string xmlFilePath = (cam_config_path + cam_name.at(cam_type) + intrinsic_postfix);
  if (doc.LoadFile(xmlFilePath.c_str())) {
    LOG(ERROR) << "FATAL ERROR: Failed to open intrinsic xml file "
              << xmlFilePath << std::endl;
    LOG(ERROR) << "the file is not found or it's not a valid xml file"
              << std::endl;
    return false;
  }
  LOG(INFO) << "ReadIntrinsic: " << xmlFilePath << std::endl;
  tinyxml2::XMLElement *param_element = doc.FirstChildElement("param");
  if (!param_element) {
    LOG(ERROR) << "cannot find tag 'param' in xml file " << xmlFilePath
              << std::endl;
    return false;
  }

  std::map<std::string, float> values = {
      {"fx", 0.f},           {"fy", 0.f},         {"cx", 0.f}, {"cy", 0.f},
      {"k1", 0.f},           {"k2", 0.f},         {"k3", 0.f}, {"k4", 0.f},
      {"image_height", 0.f}, {"image_width", 0.f}};

  for (auto &p : values) {
    auto tag_name = p.first;
    tinyxml2::XMLElement *textNode =
        param_element->FirstChildElement(tag_name.c_str());
    if (!textNode) {
      LOG(ERROR) << "cannot find tag " << tag_name << " in xml file "
                << xmlFilePath << std::endl;
      return false;
    }
    LOG(INFO) << "ReadIntrinsic: " << tag_name << " = " << textNode->GetText();
    p.second = atof(textNode->GetText());
  }
  camera_intrinsic_[cam_type] = (cv::Mat_<double>(3, 3) << values.at("fx"), 0.0, values.at("cx"), 0.0, values.at("fy"), values.at("cy"), 0.0, 0.0, 1.0);
  camera_distort_[cam_type] = (cv::Mat_<double>(4, 1) << values.at("k1"), values.at("k2"), values.at("k3"), values.at("k4"));

  return true;
}

void DataProvider::OnReceiveDRCallback(const LocalizationPtr& dr_msg) {
  
  if(!dr_msg) {
    return;
  }
  //LOG(INFO) << "Receive localization message" << std::endl;
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(3) << dr_msg->header().timestamp_sec();
  double ts = std::stod(timestamp_stream.str().substr(start_pos).c_str());
  latest_dr_ts_.store(ts, std::memory_order_relaxed);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  auto t = Eigen::Vector3d(dr_msg->pose().position().x(),
                           dr_msg->pose().position().y(),
                           dr_msg->pose().position().z());
  auto q = Eigen::Quaterniond(dr_msg->pose().orientation().qw(),
                              dr_msg->pose().orientation().qx(),
                              dr_msg->pose().orientation().qy(),
                              dr_msg->pose().orientation().qz());
  pose = semantic_mapping::PoseUtils::toEigenMat(t,q);
  {
    std::lock_guard<std::mutex> lock(dr_mutex_);
    receive_dr_[ts] = pose;
    if(receive_dr_.size() > pose_data_buff_size_) {
      for (auto it = receive_dr_.begin(); it != receive_dr_.end() && receive_dr_.size() > pose_data_buff_size_/2; ) {
        it = receive_dr_.erase(it); 
      }
    }
  }

  if(enable_odom_receive_) {
    SendOutPose(dr_msg, ts);
  }
  
}

void DataProvider::OnReceiveLocCallback(const LocalizationPtr& localization_msg) {
  if(!localization_msg) {
    return;
  }
  LOG_FIRST_N(INFO, 3) << "[LocCB] using_imu_chassis=" << using_imu_chassis_
                       << " eskf=" << (eskf_ ? "yes" : "null")
                       << " ts=" << std::fixed << localization_msg->header().timestamp_sec();
  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(3) << localization_msg->header().timestamp_sec();
  double ts = std::stod(timestamp_stream.str().substr(start_pos).c_str());
  latest_gt_ts_.store(ts, std::memory_order_relaxed);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  auto t = Eigen::Vector3d(localization_msg->pose().position().x(),
                            localization_msg->pose().position().y(),
                            localization_msg->pose().position().z());
  auto q = Eigen::Quaterniond(localization_msg->pose().orientation().qw(),
                               localization_msg->pose().orientation().qx(),
                               localization_msg->pose().orientation().qy(),
                               localization_msg->pose().orientation().qz());
  pose = semantic_mapping::PoseUtils::toEigenMat(t,q);
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    receive_pose_[ts] = pose;
    if(receive_pose_.size() > pose_data_buff_size_) {
      for (auto it = receive_pose_.begin(); it != receive_pose_.end() && receive_pose_.size() > pose_data_buff_size_/2; ) {
        it = receive_pose_.erase(it); 
      }
    }
  }
  if(need_recorder_){
    auto dr_r = q.normalized().toRotationMatrix();
    Eigen::Vector3d dr_position(t(0), t(1), std::atan2(dr_r(1,0), dr_r(0,0)));
    AmclRecorder::GetInstance()->AddResult(dr_position, ts, 1);
  }

  if (using_imu_chassis_ && eskf_) {
    const double raw_ts = localization_msg->header().timestamp_sec();
    auto rot = q.normalized().toRotationMatrix();
    const double yaw = std::atan2(rot(1, 0), rot(0, 0));
    if (!eskf_->initialized()) {
      delayed_measurement_eskf::DelayedMeasurementEskf::StateVec init_state;
      init_state << t(0), t(1), 0.0, 0.0, yaw;
      eskf_->Initialize(raw_ts, init_state);
    } else {
      delayed_measurement_eskf::DelayedMeasurementEskf::MeasVec meas;
      meas << t(0), t(1), yaw;
      delayed_measurement_eskf::DelayedMeasurementEskf::MeasVec meas_cov;
      meas_cov << 1e-2, 1e-2, 1e-3;
      eskf_->AddInstantMeasurement(meas, meas_cov);
    }
    // Publish the latest ESKF fused state
    const auto estimate = eskf_->latest_estimate();
    const double eskf_yaw = estimate.state(4);
    auto eskf_pose = std::make_shared<LocalizationEstimate>();
    eskf_pose->mutable_header()->set_timestamp_sec(estimate.timestamp_sec);
    eskf_pose->mutable_pose()->mutable_position()->set_x(estimate.state(0));
    eskf_pose->mutable_pose()->mutable_position()->set_y(estimate.state(1));
    eskf_pose->mutable_pose()->mutable_position()->set_z(t(2));
    Eigen::Quaterniond eskf_q(Eigen::AngleAxisd(eskf_yaw, Eigen::Vector3d::UnitZ()));
    eskf_pose->mutable_pose()->mutable_orientation()->set_qw(eskf_q.w());
    eskf_pose->mutable_pose()->mutable_orientation()->set_qx(eskf_q.x());
    eskf_pose->mutable_pose()->mutable_orientation()->set_qy(eskf_q.y());
    eskf_pose->mutable_pose()->mutable_orientation()->set_qz(eskf_q.z());
    eskf_pose->mutable_pose()->mutable_linear_velocity()->set_x(estimate.state(2));
    eskf_pose->mutable_pose()->mutable_linear_velocity()->set_y(estimate.state(3));
    eskf_pose->mutable_pose()->mutable_linear_velocity()->set_z(0.0);
    eskf_pose->set_measurement_time(estimate.timestamp_sec);
    if(pose_publisher_ ) {
      if(need_recorder_) {
        Eigen::Vector3d amcl_position;
        amcl_position(0) = eskf_pose->pose().position().x();
        amcl_position(1) = eskf_pose->pose().position().y();
        amcl_position(2) = eskf_yaw;
        AmclRecorder::GetInstance()->AddResult(amcl_position, ts, 0);
      }
      ADEBUG << "[ESKF] Publish state=[" << std::fixed << std::setprecision(3)
             << estimate.state.transpose() << "] t=" << estimate.timestamp_sec;
      pose_publisher_->Write(eskf_pose);
    }
    return;
  }

  if(!enable_odom_receive_) {
    SendOutPose(localization_msg, ts);
  }
}

void DataProvider::OnReceiveImage( const std::shared_ptr<ImageFormat>& compressed_image, 
                                   const CameraType& cam_type) {
  if(!compressed_image) {
    return;
  }
  std::shared_ptr<cv::Mat> img_ptr =nullptr;
  #ifdef IMAGE_COMPRESSED_USE
  std::vector<uint8_t> compressed_raw_data(compressed_image->data().begin(),
                                            compressed_image->data().end());
  img_ptr = std::make_shared<cv::Mat>(
                              cv::imdecode(compressed_raw_data, cv::IMREAD_COLOR));
  #else
  uint32_t width = compressed_image->width();
  uint32_t height = compressed_image->height();
  const std::string& encoding = compressed_image->encoding();
  const auto& data = compressed_image->data();

  if (0 == width || 0 == height || data.empty()) {
    AERROR << "empty image";
    return;
  }

  if ("rgb8" == encoding) {
    img_ptr = std::make_shared<cv::Mat>(
        height, width, CV_8UC3,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(*img_ptr, *img_ptr, cv::COLOR_RGB2BGR);
  } else if ("bgr8" == encoding) {
    img_ptr = std::make_shared<cv::Mat>(
        height, width, CV_8UC3,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
  } else if ("rgba8" == encoding) {
    img_ptr = std::make_shared<cv::Mat>(
        height, width, CV_8UC4,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(*img_ptr, *img_ptr, cv::COLOR_RGBA2BGR);
  } else if ("bgra8" == encoding) {
    img_ptr = std::make_shared<cv::Mat>(
        height, width, CV_8UC4,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(*img_ptr, *img_ptr, cv::COLOR_BGRA2BGR);
  } else if ("mono8" == encoding) {
    img_ptr = std::make_shared<cv::Mat>(
        height, width, CV_8UC1,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(*img_ptr, *img_ptr, cv::COLOR_GRAY2BGR);
  } else if ("yuyv" == encoding || "YUYV" == encoding ||
            "yuy2" == encoding || "YUY2" == encoding) {
    img_ptr = std::make_shared<cv::Mat>(
        height, width, CV_8UC2,
        const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())));
    cv::cvtColor(*img_ptr, *img_ptr, cv::COLOR_YUV2BGR_YUYV);
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
      img_ptr = std::make_shared<cv::Mat>();
      cv::cvtColor(yuv_image, *img_ptr, cv::COLOR_YUV2BGR);
    } else {
      AERROR << "Invalid YUV420 data size: " << data.size()
            << " expected: " << (y_size + 2 * uv_size);
      return;
    }
  } else {
    AERROR << "Unsupported image encoding: " << encoding << " for camera ";
    return;
  }

  if (img_ptr->empty()) {
    AERROR << "Failed to create cv::Mat from raw image for camera ";
    return;
  }

  #endif

  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(1) << compressed_image->header().timestamp_sec();
  double ts = std::stod(timestamp_stream.str().substr(start_pos));
  //LOG(INFO) << "Receive camera message "<<timestamp_stream.str().substr(6)<<" " << cam_type << std::endl;
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if(receive_images_.count(ts) == 0) {
      receive_images_[ts] = std::array<std::shared_ptr<cv::Mat>, CamTypeSize>{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    }
    receive_images_.at(ts).at(cam_type) = img_ptr;
    if(receive_images_.size() > sensor_data_buff_size_) {
      for (auto it = receive_images_.begin(); (receive_images_.size() > sensor_data_buff_size_/2) && (it != receive_images_.end()); ) {
        it = receive_images_.erase(it); 
      }
    }
  }
  LOG_EVERY_N(INFO,50) <<"image size: " << receive_images_.size() << std::endl;

}

void DataProvider::OnReceiveLidar( const std::shared_ptr<drivers::PointCloudPacked>& lidar_data, 
                     const LidarType& lidar_type) {
  const float kPointInfThreshold = 1e2;
  if(lidar_data->point_size() == 0) {
    return;
  } 
  std::vector<PointPacked> ppd( lidar_data->point_size());
  const std::string& raw_data = lidar_data->data();
  std::memcpy(ppd.data(), raw_data.data(), raw_data.size());
  
  pcl::PointXYZI point;

  std::ostringstream timestamp_stream;
  timestamp_stream << std::fixed << std::setprecision(1) << lidar_data->header().timestamp_sec();
  double ts = std::stod(timestamp_stream.str().substr(start_pos).c_str());
  //LOG(INFO) << "Receive lidar message "<<timestamp_stream.str().substr(6)<<" " << lidar_type <<" SIZE "<< lidar_data->point_size();
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < lidar_data->point_size(); ++i) {
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
    Eigen::Vector3d vec3d_imu = lidar2vehicle_matrix_.at(static_cast<uint32_t>(lidar_type)) * vec3d_lidar;

    if(vec3d_imu(2) > ground_height_thres_) {
      continue;
    }
    
    point.x = vec3d_imu(0);
    point.y = vec3d_imu(1);
    point.z = vec3d_imu(2);
    point.intensity = static_cast<float>(pt.intensity());
    input_cloud->push_back(point);
  }

  input_cloud->resize(input_cloud->size());
  {
    std::lock_guard<std::mutex> lock(pcl_mutex_);
    if(receive_pcl_.count(ts) == 0) {
      receive_pcl_[ts] = std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr, LidarTypeSize>{nullptr, nullptr, nullptr, nullptr};
    }
    receive_pcl_.at(ts).at(lidar_type) = input_cloud;
    if(receive_pcl_.size() > sensor_data_buff_size_) {
      for (auto it = receive_pcl_.begin(); (receive_pcl_.size() > sensor_data_buff_size_/2) && (it != receive_pcl_.end());) {
        it = receive_pcl_.erase(it);  
      }
    }
  }
 // LOG(INFO) << "lidar message SIZE "<< receive_pcl_.at(ts).at(lidar_type)->points.size();
  LOG_EVERY_N(INFO,50) <<"pcl size: " << receive_pcl_.size() << std::endl;
}

semantic_mapping::CameraParmeter DataProvider::ProposeCameraParmeter() const {
  semantic_mapping::CameraParmeter out;
  std::copy(camera_q_, camera_q_ + CamTypeSize, out.camera_q);
  std::copy(camera_t_, camera_t_ + CamTypeSize, out.camera_t);
  std::copy(camera_intrinsic_, camera_intrinsic_ + CamTypeSize, out.camera_intrinsic);
  std::copy(camera_distort_, camera_distort_ + CamTypeSize, out.camera_distort);
  return out;
}

void DataProvider::OnReceiveChassisCallback(const std::shared_ptr<canbus::Chassis>& chassis_msg) {
  if (!using_imu_chassis_ || !eskf_ || !eskf_->initialized()) {
    return;
  }
  LOG_FIRST_N(INFO, 3) << "[ChassisCB] wheels=["
                        << chassis_msg->wheel_speed_0() << ","
                        << chassis_msg->wheel_speed_1() << ","
                        << chassis_msg->wheel_speed_2() << ","
                        << chassis_msg->wheel_speed_3() << "]";
  const bool all_wheels_zero =
      std::abs(chassis_msg->wheel_speed_0()) < kZeroSpeedThreshold &&
      std::abs(chassis_msg->wheel_speed_1()) < kZeroSpeedThreshold &&
      std::abs(chassis_msg->wheel_speed_2()) < kZeroSpeedThreshold &&
      std::abs(chassis_msg->wheel_speed_3()) < kZeroSpeedThreshold;
  if (all_wheels_zero) {
    delayed_measurement_eskf::DelayedMeasurementEskf::ZuptVec zupt_cov;
    zupt_cov << 1e-4, 1e-4;
    eskf_->AddZeroVelocityUpdate(zupt_cov);
  }
}

void DataProvider::OnReceiveImuCallback(const std::shared_ptr<localization::CorrectedImu>& imu_msg) {
  if (!using_imu_chassis_ || !eskf_) {
    return;
  }
  if (!imu_msg->has_header() || !imu_msg->has_imu()) {
    return;
  }
  const double timestamp_sec = imu_msg->header().timestamp_sec();
  LOG_FIRST_N(INFO, 5) << "[ImuCB] ts=" << std::fixed << timestamp_sec
                        << " initialized=" << eskf_->initialized();
  const auto& imu = imu_msg->imu();
  // CorrectedImu stores acceleration/angular_velocity in imu().linear_acceleration() / angular_velocity()
  // (the _vrf fields are NOT populated). Apply same (y,-x,z) swap as MSF to get to intermediate frame.
  const double ax_imu = imu.linear_acceleration().y();
  const double ay_imu = -imu.linear_acceleration().x();
  const double wz = imu.angular_velocity().z();
  // Apply R_veh_imu (tf_vehicle_imu yaw) to get vehicle frame (Right/Forward/Up)
  const double ax_veh = imu_to_vehicle_cos_ * ax_imu - imu_to_vehicle_sin_ * ay_imu;
  const double ay_veh = imu_to_vehicle_sin_ * ax_imu + imu_to_vehicle_cos_ * ay_imu;
  // After MSF (y,-x,z) swap, data is already in FLU (Forward/Left/Up), same as ESKF body frame
  delayed_measurement_eskf::DelayedMeasurementEskf::ControlVec control;
  control(0) = ax_veh;    // forward
  control(1) = ay_veh;    // left
  control(2) = wz;
  eskf_->AddControl(timestamp_sec, control);
}

bool  DataProvider::RegisterAllReceivers() {
  if(!node_) {
    return false;
  }

  loc_reader_ = node_->CreateReader<LocalizationEstimate>(localization_topic,
    [this](const LocalizationPtr& msg) { OnReceiveLocCallback(msg); });

  if(using_imu_chassis_) {
    chassis_listener_ = node_->CreateReader<canbus::Chassis>( chassis_topic,
      [this](const std::shared_ptr<canbus::Chassis>& msg) { OnReceiveChassisCallback(msg); });

    corrected_imu_listener_ = node_->CreateReader<localization::CorrectedImu>(imu_topic,
      [this](const std::shared_ptr<localization::CorrectedImu>& msg) { OnReceiveImuCallback(msg); });
    return true;
  }

  

  dr_reader_ = node_->CreateReader<LocalizationEstimate>(received_dr_topic,
    [this](const LocalizationPtr& msg) { OnReceiveDRCallback(msg); });


  for(CameraType i = CamFrontLeft; i<CameraType::CamTypeSize; ) {
    if(camera_types_register_.count(i)==0) {
      i = static_cast<CameraType>(static_cast<uint32_t>(i)+1);
      continue;
    } 
    cam_reader_.at(i) = node_->CreateReader<ImageFormat>(cam_topic.at(i),
      [i, this](const std::shared_ptr<ImageFormat>& msg) { OnReceiveImage(msg, i); }  );
    i = static_cast<CameraType>(static_cast<uint32_t>(i)+1);
  }
    
  for(LidarType i = HeliosFrontLeft; i<LidarType::LidarTypeSize;) {
    if(lidar_types_register_.count(i)==0) {
      i = static_cast<LidarType>(static_cast<uint32_t>(i)+1);
      continue;
    } 
    lidar_reader_.at(i) = node_->CreateReader<drivers::PointCloudPacked>(lidar_topic.at(i),
      [i, this](const std::shared_ptr<drivers::PointCloudPacked>& msg) { OnReceiveLidar(msg, i); });
    i = static_cast<LidarType>(static_cast<uint32_t>(i)+1);
  }

  return true;
}


bool DataProvider::ReadExtrinsic(const CameraType cam_type) {
  std::vector<float> res;
  const std::string filename = cam_config_path + cam_name.at(cam_type) + extrinsic_postfix;
  std::ifstream inputFile(filename);
  if (!inputFile.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    exit(0);
  }
  LOG(INFO) << filename << std::endl;
  std::string line;
  std::vector<std::string> lines(11);
  for (size_t i = 0; i < lines.size(); i++)
    std::getline(inputFile, lines[i]); //translation:
  inputFile.close();
  for (int i: {1, 2, 3, 6, 7, 8, 9}) {
    auto line = lines[i];
    std::istringstream iss(line);
    std::string token;
    float value;
    iss >> token >> value;
    res.push_back(value);
    LOG(INFO) << value << std::endl;
  }
  camera_q_[cam_type] = Eigen::Quaterniond(res[6], res[3], res[4], res[5]);;
  camera_t_[cam_type] =  Eigen::Vector3d(res[0], res[1], res[2]);

  return true;

}


bool DataProvider::ReadImuExtrinsic() {
  const std::string ins_calib_file = "/century/vehicle_config/calib/ins_calib.yaml";
  YAML::Node config;
  try {
    config = YAML::LoadFile(ins_calib_file);
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to load IMU extrinsic from " << ins_calib_file
               << ": " << e.what();
    return false;
  }
  if (!config["tf_vehicle_imu"]) {
    LOG(ERROR) << "No tf_vehicle_imu in " << ins_calib_file;
    return false;
  }
  const double yaw_deg =
      config["tf_vehicle_imu"]["rotation"]["yaw"].as<double>();
  imu_to_vehicle_yaw_rad_ = yaw_deg * M_PI / 180.0;
  imu_to_vehicle_cos_ = std::cos(imu_to_vehicle_yaw_rad_);
  imu_to_vehicle_sin_ = std::sin(imu_to_vehicle_yaw_rad_);
  LOG(INFO) << "IMU extrinsic yaw: " << yaw_deg << " deg ("
            << imu_to_vehicle_yaw_rad_ << " rad)";
  return true;
}

bool DataProvider::ReadLidarExtrinsic() {
  const std::string extrinsic_file = cam_config_path  + "/lidar_extrinsic.txt";
  std::ifstream inputFile(extrinsic_file);
  if (!inputFile.is_open()) {
    std::cerr << "Error opening file: " << extrinsic_file << std::endl;
    exit(0);
  }

  std::string line;
  int n=0;
  while (getline(inputFile, line)) {
    lidar2vehicle_matrix_.at(n) = Eigen::Affine3d::Identity();
    std::string data[7];
    int i = 0;
    std::stringstream line_stream(line);
    while (std::getline(line_stream, data[i++], ',')) { 

    } 
    lidar2vehicle_matrix_.at(n).translation() = Eigen::Vector3d(stod(data[0]),stod(data[1]), stod(data[2]));
    lidar2vehicle_matrix_.at(n).rotate(QuaternionToRotationMatrix(stod(data[3]),stod(data[4]), stod(data[5]), stod(data[6])));
    LOG(INFO)<< std::fixed << std::setprecision(6)
             << data[0]<<" " << data[1]<<" " << data[2]<<" " << data[3]<<" " << data[4]<<" " << data[5]<<" " << data[6]<<std::endl;
  
    if(n++ == lidar_name.size()-1) {
      break;
    }
  }

  return true;
}

bool DataProvider::Init(const std::string& process_name,
                        const std::unordered_set<semantic_mapping::CameraType> camera_types_register,
                        const std::unordered_set<semantic_mapping::LidarType> lidar_types_register,
                        const double& seg_thres, const bool speed_up, const bool need_recorder,
                        const bool enable_odom_receive,
                        const bool using_imu_chassis) {
  enable_odom_receive_ = enable_odom_receive;
  using_imu_chassis_ = using_imu_chassis;
  if (using_imu_chassis_) {
    eskf_ = std::make_unique<delayed_measurement_eskf::AsyncDelayedMeasurementEskf>();
    if (!ReadImuExtrinsic()) {
      LOG(WARNING) << "ReadImuExtrinsic failed, using identity rotation for IMU.";
    }
  }
  need_recorder_ = need_recorder;
  camera_types_register_ = camera_types_register;
  if(camera_types_register_.empty()) {
    return false;
  }
  lidar_types_register_ = lidar_types_register;
  if(lidar_types_register_.empty()) {
    return false;
  }
  LOG(INFO) << "DataProvider Init with process name: " << process_name << std::endl;
  if(!century::cyber::Init(process_name.c_str())) {
    LOG(INFO) << "Cyber Init failed!";
    return false;
  }
  node_ = century::cyber::CreateNode(process_name.c_str());
  if(!node_) {
    LOG(INFO) << "CybeR Creare node failed!";
    return false;
  }
  LOG(INFO) << "DataProvider Init successfully, using_imu_chassis=" << using_imu_chassis_
            << " eskf=" << (eskf_ ? "created" : "null");
 
#if 0
  for(LidarType i=HeliosFrontLeft; i<LidarType::LidarTypeSize; ) {
    century::perception::onboard::LidarDetectionComponentConfig comp_config;
    if (!cyber::common::GetProtoFromFile(lidar_config_path + lidar_name.at(i) + ".pb.txt", &comp_config)) {
      LOG(ERROR) << "Get config failed for "<< lidar_name.at(i);
      return false;
    }
    const auto sensor_name = comp_config.sensor_name();
    Eigen::Matrix4d lidar2imu_matrix;
    lidar2vehicle_trans_.QueryStaticTF("imu", sensor_name, &lidar2imu_matrix);
    Eigen::Matrix4d imu2vehicle_matrix;
    lidar2vehicle_trans_.QueryStaticTF("vehicle", "imu", &imu2vehicle_matrix);
    lidar2vehicle_matrix_.at(i) = Eigen::Affine3d(imu2vehicle_matrix * lidar2imu_matrix);
    i = static_cast<LidarType>(static_cast<uint32_t>(i)+1);
  }

  for(CameraType i=CamFrontLeft; i<CameraType::CamTypeSize; ) {
    century::perception::onboard::LidarDetectionComponentConfig comp_config;
    if (!cyber::common::GetProtoFromFile(cam_config_path + cam_name.at(i) + ".pb.txt", &comp_config)) {
      LOG(ERROR) << "Get config failed for "<< lidar_name.at(i);
      return false;
    }
    const auto sensor_name = comp_config.sensor_name();
    Eigen::Matrix4d camera2local_matrix;
    camera2vehicle_trans_.QueryStaticTF("camera", sensor_name, &camera2local_matrix);
    semantic_mapping::PoseUtils::convertEigenMat(camera2local_matrix, camera_t_[i], camera_q_[i]);
    if(!ReadIntrinsic(i)) {
      LOG(ERROR) << "Read intrinsic failed for "<< cam_name.at(i);
      return false;
    }
    i = static_cast<CameraType>(static_cast<uint32_t>(i)+1);
  }
#else
  if(!ReadLidarExtrinsic()) {
    LOG(ERROR) << "Read lidar extrinsic failed!";
    return false;
  }
  for(CameraType i=CamFrontLeft; i<CameraType::CamTypeSize; ) {
    if(!ReadExtrinsic(i)) {
      LOG(ERROR) << "Read extrinsic failed for "<< cam_name.at(i);
      return false;
    }
    if(!ReadIntrinsic(i)) {
      LOG(ERROR) << "Read intrinsic failed for "<< cam_name.at(i);
      return false;
    }
    i = static_cast<CameraType>(static_cast<uint32_t>(i)+1);
  }
#endif

 
  for (const auto& cam_type : camera_types_register_) {
    segmentors_.at(cam_type) =
        std::make_unique<ImageSegmentation>(modelPath, labelsPath, seg_thres,
                                            speed_up);
  }
  RegisterAllReceivers();
  pose_publisher_ = node_->CreateWriter<LocalizationEstimate>(loccalization_publish_topic);
  odom_reset_publisher_ = node_->CreateWriter<::century::localization::MsfReset>(odom_reset_topic);

  return true;
}
bool DataProvider::FetchBundleData(double& ts, 
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                                   std::unordered_map<CameraType, cv::Mat>& images, 
                                   Eigen::Matrix4d& closest_pose, Eigen::Matrix4d& closest_dr){
  auto start = std::chrono::steady_clock::now();
  std::map<double, std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr, LidarTypeSize>> receive_pcl;
  {
    std::lock_guard<std::mutex> lock(pcl_mutex_);
    receive_pcl = receive_pcl_;
  }

  std::map<double, std::array<std::shared_ptr<cv::Mat>, CamTypeSize>> receive_images;
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    receive_images = receive_images_;
  }

  for (auto it = receive_images.begin(); it != receive_images.end() && it->first < last_ts_; ) {
    it = receive_images.erase(it); 
  }

  std::map<double, Eigen::Matrix4d> receive_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    receive_pose = receive_pose_;
  }

  std::map<double, Eigen::Matrix4d> receive_dr;
  {
    std::lock_guard<std::mutex> lock(dr_mutex_);
    receive_dr = receive_dr_;
  }

  std::set<double> cam_ts;
  for(const auto& iter : receive_images) {
    bool all_ready = true;
    for(const auto& cam_type : camera_types_register_) {
      if(!iter.second.at(cam_type)) {
        all_ready = false;
        break;
      }
    }
    if(all_ready) {
      cam_ts.insert(iter.first);
    }
  }
  
  std::set<double> lidar_ts;
  for(const auto& iter : receive_pcl) {
    bool all_ready = true;
    for(const auto& lidar_type : lidar_types_register_) {
      if(!iter.second.at(lidar_type)) {
        all_ready = false;
        break;
      }

      if(iter.second.at(lidar_type)->points.empty()) {
        all_ready = false;
        break;
      }
    }
    if(all_ready) {
      lidar_ts.insert(iter.first);
    }
  }

  double  sensor_ts = 0.0;
  double  loc_ts = 0.0;
   if(!CalculateTargetTs(lidar_ts, cam_ts, receive_pose, loc_ts, sensor_ts)) {
    return false;
   }
  
  
  double timestamp = std::round(sensor_ts * 10.0) / 10.0;;
 
  pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for(const auto& iter : receive_pcl.at(timestamp)) {
    if((!iter) || (iter->points.empty())) {
      continue;
    }
    if(result_cloud->points.empty()) {
      result_cloud = iter;
    } else {
      // Concatenate the point clouds
      result_cloud->insert(result_cloud->end(), iter->begin(), iter->end());
      result_cloud->width = result_cloud->points.size();
      result_cloud->height = 1;
    }
  }
  
  cloud = result_cloud;
  if(cloud->points.empty()) {
    LOG(ERROR) << "No lidar data received for ts: " << std::fixed << std::setprecision(1) << timestamp;
    return false;
  }

 
  for (const auto& cam_type : camera_types_register_) {
    const auto& image_ptr = receive_images.at(timestamp).at(cam_type);
    if (image_ptr == nullptr || segmentors_.at(cam_type) == nullptr) {
      continue;
    }
    auto seg_start = std::chrono::steady_clock::now();
    cv::Mat segmented;
    segmentors_.at(cam_type)->DoSegment(*image_ptr, segmented);
    auto seg_end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(seg_end - seg_start);
    LOG(INFO) << cam_type << " segmentation time: " << duration.count() << " ms";
    images[cam_type] = std::move(segmented);
  }
 
 
  closest_pose = receive_pose.at(loc_ts);
  if(enable_odom_receive_) {
    closest_dr = Eigen::Matrix4d::Identity();
    if(!GetClosestPose(closest_dr, loc_ts, receive_dr)) {
      LOG(ERROR) << "Failed to get closest dr pose for ts: " << std::fixed << std::setprecision(3) << loc_ts;
      return false;
    }
    auto dr_pose = PoseUtils::EigenMatToVector3d(closest_dr);
    auto gt_pose = PoseUtils::EigenMatToVector3d(closest_pose);
    LOG(INFO) <<" LANTENCY: "<< latest_dr_ts_.load() - loc_ts 
              <<" ENABLE ODOM RECEIVE (" << dr_pose(0)<< " , " << dr_pose(1) << ") (" << gt_pose(0) << " , " << gt_pose(1) << ")";
  } else {
    LOG(INFO) <<" LANTENCY: "<< latest_gt_ts_.load() - loc_ts  <<" =========== ";
  }
 
  last_ts_ = timestamp + 0.01;
  ts = loc_ts;
  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  LOG(INFO)<<" FETCH DATA SYNC: " << duration.count() << " ms";
  return true;
}

bool DataProvider::GetClosestPose(Eigen::Matrix4d& object_synced_state, double& ts, const std::map<double, Eigen::Matrix4d>& receive_pose) {
  double minimum_time_diff = std::numeric_limits<float>::max();
  const double in_timestamp = ts;
  if(receive_pose.empty()) {
    LOG(ERROR) << "No pose data received yet!";
    return false;
  } 
  for (const auto& lidar_state : receive_pose) {
    if (std::fabs(in_timestamp - lidar_state.first) < minimum_time_diff) {
      minimum_time_diff = fabs(in_timestamp - lidar_state.first);
      object_synced_state = lidar_state.second;
      ts = lidar_state.first;
    }

    if (minimum_time_diff < std::numeric_limits<float>::min()) {
      break;
    }
  }
  LOG(INFO) << "Get closest pose for timestamp: " << std::fixed << std::setprecision(3) << in_timestamp << " with time diff: " << minimum_time_diff;
  return true;

}


bool DataProvider::CalculateTargetTs(const std::set<double>& lidar_ts, const std::set<double>& cam_ts, const std::map<double, Eigen::Matrix4d>& pose_ts,
                                     double& loc_ts, double& sensor_ts) {
  if(lidar_ts.empty() || cam_ts.empty() || pose_ts.empty()) {
    LOG_EVERY_N(INFO,100) <<"lidar size: " << lidar_ts.size() << " cam size: " << cam_ts.size() << " pose size: " << pose_ts.size() << std::endl;
    return false;
  }

  std::set<double> refine_cam_ts = cam_ts;
  while(pose_ts.rbegin()->first < *refine_cam_ts.rbegin()) {
    refine_cam_ts.erase(*refine_cam_ts.rbegin());
  }

  for (auto it = refine_cam_ts.rbegin(); it != refine_cam_ts.rend(); ++it) {
    sensor_ts = *it;
    if (std::find(lidar_ts.begin(), lidar_ts.end(), sensor_ts) != lidar_ts.end()) {
      double minimum_time_diff = std::numeric_limits<float>::max();
      for(auto iter = pose_ts.rbegin(); iter != pose_ts.rend(); ++iter) {
        if (std::fabs(sensor_ts - iter->first) < minimum_time_diff) {
          minimum_time_diff = std::fabs(sensor_ts - iter->first);
          loc_ts = iter->first;
        }
      }
      return true;
    }
  }

  return false;
}

bool DataProvider::PublishOdomReset() {
  auto msg = std::make_shared<::century::localization::MsfReset>();
  auto* header_msg = msg->mutable_header();
  header_msg->set_timestamp_sec(::century::cyber::Clock::NowInSeconds());
  header_msg->set_frame_id("map");
  static uint32_t sequence_num = 0;
  header_msg->set_sequence_num(sequence_num++);
  msg->set_is_reset(true);
  odom_reset_publisher_->Write(msg);
  return true;
}

void DataProvider::SendOutPose(const LocalizationPtr& odom_msg, const double req_ts) {
  auto publish_pose  = AmclPosePublisher::GetInstance()->RolloutFuseddAmclPose(odom_msg, req_ts);
  if(publish_pose) {
    pose_publisher_->Write(publish_pose);
    if(need_recorder_) {
      Eigen::Vector3d amcl_position;
      auto Q = Eigen::Quaterniond(publish_pose->pose().orientation().qw(),
                                  publish_pose->pose().orientation().qx(),
                                  publish_pose->pose().orientation().qy(),
                                  publish_pose->pose().orientation().qz());
      auto R = Q.normalized().toRotationMatrix();
      amcl_position(0) = publish_pose->pose().position().x();
      amcl_position(1) = publish_pose->pose().position().y();
      amcl_position(2) = std::atan2(R(1,0), R(0,0));
      AmclRecorder::GetInstance()->AddResult(amcl_position, req_ts, 0);
    }
  } 
}

}
