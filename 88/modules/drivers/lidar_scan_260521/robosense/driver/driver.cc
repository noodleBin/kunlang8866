#include "driver.h"

//#include <cstddef>

#if defined(__linux__)
#include <pthread.h>
#endif

#include "cyber/scheduler/scheduler_factory.h"

namespace century {
namespace drivers {
namespace robosense {

namespace {

constexpr uint64_t POINT_CLOUD_POOL_MISS_LOG_EVERY_N = 100;

}  // namespace

// void RobosenseDriver::InitParam() {
//   RSDriverParam param;
//   RSInputParam input_param;
//   input_param.msop_port = conf_.msop_port();
//   input_param.difop_port = conf_.difop_port();
// }

#include <unordered_map>

LidarType RobosenseDriver::ConvertToLidarType(const std::string& lidar_type) {
  static const std::unordered_map<std::string, LidarType> lidar_map = {
      {"RSHELIOS", LidarType::RSHELIOS},
      {"RSBP", LidarType::RSBP},
      {"RSM1", LidarType::RSM1},
      {"RSM1P", LidarType::RSM1}};

  auto it = lidar_map.find(lidar_type);
  if (it != lidar_map.end()) {
    return it->second;
  } else {
    return LidarType::RSHELIOS;
  }
}

#pragma pack(push, 1)
struct PointPacked {
  float x;
  float y;
  float z;
  uint16_t intensity;
  uint16_t ring;
  double timestamp;
};
#pragma pack(pop)
// struct PointPacked {
//   float    x;
//   float    y;
//   float    z;
//   uint8_t  intensity;
//   uint8_t  padding;
//   uint16_t ring;
//   double timestamp;
// };

// static_assert(sizeof(PointPacked) == sizeof(PointT),
//               "PointPacked must match the Robosense SDK point layout.");
// static_assert(offsetof(PointPacked, x) == offsetof(PointT, x),
//               "PointPacked x offset must match PointT.");
// static_assert(offsetof(PointPacked, y) == offsetof(PointT, y),
//               "PointPacked y offset must match PointT.");
// static_assert(offsetof(PointPacked, z) == offsetof(PointT, z),
//               "PointPacked z offset must match PointT.");
// static_assert(offsetof(PointPacked, intensity) == offsetof(PointT,
// intensity),
//               "PointPacked intensity offset must match PointT.");
// static_assert(offsetof(PointPacked, ring) == offsetof(PointT, ring),
//               "PointPacked ring offset must match PointT.");
// static_assert(offsetof(PointPacked, timestamp) == offsetof(PointT,
// timestamp),
//               "PointPacked timestamp offset must match PointT.");

bool RobosenseDriver::Init() {
  RSDriverParam param;

  param.input_type = InputType::ONLINE_LIDAR;
  param.input_param.msop_port = conf_.msop_port();
  param.input_param.difop_port = conf_.difop_port();
  param.decoder_param.use_lidar_clock = conf_.use_lidar_clock();
  param.decoder_param.min_distance = conf_.min_distance();
  param.decoder_param.max_distance = conf_.max_distance();
  param.decoder_param.start_angle = conf_.start_angle();
  param.decoder_param.end_angle = conf_.end_angle();
  param.decoder_param.dense_points = conf_.is_dense();
  param.decoder_param.split_angle = conf_.split_angle();
  param.decoder_param.split_frame_mode =
      static_cast<SplitFrameMode>(conf_.split_frame_mode());

  param.lidar_type = ConvertToLidarType(conf_.model());

  to_exit_process_ = false;

  driver_.regPointCloudCallback(
      std::bind(&RobosenseDriver::DriverGetPointCloudFromCallerCallback, this),
      std::bind(&RobosenseDriver::DriverReturnPointCloudToCallerCallback, this,
                std::placeholders::_1));
  driver_.regPacketCallback(
      std::bind(&RobosenseDriver::PacketCallback, this, std::placeholders::_1));

  driver_.regExceptionCallback(std::bind(&RobosenseDriver::ExceptionCallback,
                                         this, std::placeholders::_1));

  if (!driver_.init(param)) {
    AERROR << name_ << ": Failed to initialize driver.";
    return false;
  }

  century::cyber::proto::RoleAttributes writer_attr;
  writer_attr.set_channel_name(conf_.pointcloud_channel());
  century::cyber::proto::QosProfile qos =
      century::cyber::transport::QosProfileConf::QOS_PROFILE_SENSOR_DATA;
  *writer_attr.mutable_qos_profile() = qos;

  if (!conf_.scan_channel().empty()) {
    century::cyber::proto::RoleAttributes scan_writer_attr;
    scan_writer_attr.set_channel_name(conf_.scan_channel());
    *scan_writer_attr.mutable_qos_profile() = qos;
    scan_writer_ = node_->CreateWriter<RobosenseScan>(scan_writer_attr);
  }

  if (conf_.use_packed_data()) {
    pointpackedcloud_writer_ =
        node_->CreateWriter<PointCloudPacked>(writer_attr);
    cloud_handle_thread_ =
        std::thread(std::bind(&RobosenseDriver::ProcessPackedCloud, this));
  } else {
    pointcloud_writer_ = node_->CreateWriter<PointXYZIRTCloud>(writer_attr);
    cloud_handle_thread_ =
        std::thread(std::bind(&RobosenseDriver::ProcessCloud, this));
  }
#if defined(__linux__)
  pthread_setname_np(cloud_handle_thread_.native_handle(), "rs_cloud");
#endif
  century::cyber::scheduler::Instance()->SetInnerThreadAttr(
      "rs_cloud", &cloud_handle_thread_);

  driver_.start();
  AINFO << name_ << ": Started driver." << RS_REND;

  return true;
}

void RobosenseDriver::Stop() {
  driver_.stop();

  std::vector<std::pair<uint32_t, std::shared_ptr<RobosenseScan>>> remaining_scans;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    for (auto& item : scan_msgs_) {
      remaining_scans.emplace_back(item.first, item.second);
    }
    scan_msgs_.clear();
  }
  for (const auto& scan : remaining_scans) {
    PublishScan(scan.first, scan.second);
  }

  to_exit_process_ = true;
  if (cloud_handle_thread_.joinable()) {
    cloud_handle_thread_.join();
  }
}

std::shared_ptr<PointCloudMsg>
RobosenseDriver::DriverGetPointCloudFromCallerCallback() {
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue_.pop();
  if (msg) {
    return msg;
  }
  const uint64_t miss_count = ++cloud_pool_miss_count_;

  AWARN_EVERY(POINT_CLOUD_POOL_MISS_LOG_EVERY_N)
      << name_ << ": point cloud pool miss count reached " << miss_count
      << ", allocating a new point cloud buffer.";

  return std::make_shared<PointCloudMsg>();
}

void RobosenseDriver::DriverReturnPointCloudToCallerCallback(
    std::shared_ptr<PointCloudMsg> msg) {
  stuffed_cloud_queue_.push(msg);
}

void RobosenseDriver::PacketCallback(const ::robosense::lidar::Packet& pkt) {
  if (!scan_writer_ || pkt.buf_.empty()) {
    return;
  }

  std::vector<std::pair<uint32_t, std::shared_ptr<RobosenseScan>>> ready_scans;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    auto& scan_msg = scan_msgs_[pkt.frame_seq];
    if (!scan_msg) {
      scan_msg = std::make_shared<RobosenseScan>();
      scan_msg->mutable_header()->set_timestamp_sec(
          pkt.timestamp > 0.0 ? pkt.timestamp
                              : century::cyber::Time::Now().ToSecond());
    }

    auto* scan_pkt = scan_msg->add_firing_pkts();
    scan_pkt->set_stamp(static_cast<uint64_t>(pkt.timestamp * 1e9));
    scan_pkt->set_data(pkt.buf_.data(), pkt.buf_.size());

    while (!scan_msgs_.empty() && scan_msgs_.begin()->first < pkt.frame_seq) {
      ready_scans.emplace_back(scan_msgs_.begin()->first,
                               scan_msgs_.begin()->second);
      scan_msgs_.erase(scan_msgs_.begin());
    }

    constexpr size_t kMaxPendingScanFrames = 8;
    while (scan_msgs_.size() > kMaxPendingScanFrames) {
      ready_scans.emplace_back(scan_msgs_.begin()->first,
                               scan_msgs_.begin()->second);
      scan_msgs_.erase(scan_msgs_.begin());
    }
  }

  for (const auto& ready_scan : ready_scans) {
    PublishScan(ready_scan.first, ready_scan.second);
  }
}

void RobosenseDriver::PublishScan(
    uint32_t frame_seq, const std::shared_ptr<RobosenseScan>& scan_msg) {
  if (!scan_writer_ || !scan_msg || scan_msg->firing_pkts().empty()) {
    return;
  }

  auto* scan_header = scan_msg->mutable_header();
  scan_header->set_sequence_num(frame_seq);
  scan_header->set_frame_id(conf_.frame_id());
  scan_msg->set_model(conf_.model());
  scan_msg->set_basetime(0);
  scan_writer_->Write(scan_msg);
}

void RobosenseDriver::ProcessCloud() {
  while (!to_exit_process_) {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
    if (!msg) {
      continue;
    }

    PointXYZIRTCloud point_clouds;
    point_clouds.set_measuretime(cyber::Time::Now().ToSecond());

    point_clouds.mutable_header()->set_sequence_num(msg->seq);
    point_clouds.set_width(msg->width);
    point_clouds.set_height(msg->height);
    point_clouds.mutable_header()->set_timestamp_sec(msg->timestamp);
    point_clouds.set_is_dense(msg->is_dense);
    point_clouds.mutable_point()->Reserve(msg->points.size());

    for (auto& point : msg->points) {
      auto point_cloud = point_clouds.add_point();
      point_cloud->set_x(point.x);
      point_cloud->set_y(point.y);
      point_cloud->set_z(point.z);
      point_cloud->set_ring(point.ring);
      point_cloud->set_intensity(point.intensity);
      point_cloud->set_timestamp(point.timestamp);
    }

    pointcloud_writer_->Write(point_clouds);
    free_cloud_queue_.push(msg);
  }
}

void RobosenseDriver::ProcessPackedCloud() {
  while (!to_exit_process_) {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
    if (!msg) {
      continue;
    }

    thread_local google::protobuf::Arena arena;
    arena.Reset();

    auto* point_clouds =
        google::protobuf::Arena::CreateMessage<PointCloudPacked>(&arena);
    // point_clouds->set_measuretime(cyber::Time::Now().ToSecond());
    point_clouds->mutable_header()->set_sequence_num(msg->seq);
    if (msg->points.empty()) {
      point_clouds->mutable_header()->set_timestamp_sec(msg->timestamp);
      point_clouds->set_measuretime(cyber::Time::Now().ToSecond());
      point_clouds->set_width(msg->width);
      point_clouds->set_height(msg->height);
      point_clouds->set_is_dense(msg->is_dense);
      point_clouds->set_point_size(0);
      pointpackedcloud_writer_->Write(*point_clouds);
      free_cloud_queue_.push(msg);
      continue;
    }

    point_clouds->mutable_header()->set_timestamp_sec(msg->points[0].timestamp);
    point_clouds->set_measuretime(cyber::Time::Now().ToSecond());
    point_clouds->set_width(msg->width);
    point_clouds->set_height(msg->height);
    point_clouds->set_is_dense(msg->is_dense);
    point_clouds->set_point_size(msg->points.size());

    auto* data = point_clouds->mutable_data();
    // data->assign(reinterpret_cast<const char*>(msg->points.data()),
    //              msg->points.size() * sizeof(PointPacked));
    data->resize(msg->points.size() * sizeof(PointPacked));
    auto* point_data = reinterpret_cast<PointPacked*>(data->data());

    for (size_t i = 0; i < msg->points.size(); ++i) {
      const auto& src = msg->points[i];
      point_data[i] = {src.x,
                       src.y,
                       src.z,
                       static_cast<uint16_t>(src.intensity),
                       static_cast<uint16_t>(src.ring),
                       src.timestamp};
    }

    pointpackedcloud_writer_->Write(*point_clouds);
    free_cloud_queue_.push(msg);
    // task.get();
  }
}

void RobosenseDriver::ExceptionCallback(const Error& code) {
  RS_WARNING << name_ << ": " << code.toString() << RS_REND;
}

}  // namespace robosense
}  // namespace drivers
}  // namespace century
