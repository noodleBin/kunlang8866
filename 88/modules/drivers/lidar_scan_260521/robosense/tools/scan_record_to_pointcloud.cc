#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "modules/drivers/lidar/proto/robosense.pb.h"
#include "modules/drivers/lidar/proto/robosense_config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

#include "cyber/common/file.h"
#include "cyber/cyber.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/api/lidar_driver.hpp"
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp"

namespace century {
namespace drivers {
namespace robosense {
namespace {

using ::century::cyber::record::RecordMessage;
using ::century::cyber::record::RecordReader;
using ::century::cyber::record::RecordWriter;
using ::robosense::lidar::InputType;
using ::robosense::lidar::LidarDriver;
using ::robosense::lidar::LidarType;
using ::robosense::lidar::Packet;
using ::robosense::lidar::RSDriverParam;
using ::robosense::lidar::SplitFrameMode;

using PointCloudMsg = ::PointCloudT<::PointXYZIRT>;

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

struct Options {
  std::string input_record;
  std::string output_record;
  std::vector<std::string> config_files;
  std::string scan_channel;
  std::string packed_channel;
  std::string cloud_channel;
};

void PrintUsage(const char* binary) {
  std::cerr
      << "Usage: " << binary
      << " --input scan.record --output packetpointcloud.record"
      << " --config modules/drivers/lidar/robosense/conf/rsm1_front.pb.txt"
      << " [--config modules/drivers/lidar/robosense/conf/rsm1_rear.pb.txt]"
      << " [--scan_channel /lidar/m1/front/Scan]"
      << " [--packed_channel /lidar/m1/front]"
      << " [--cloud_channel /lidar/m1/front/PointCloud]\n";
}

bool ParseArgs(int argc, char* argv[], Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const std::string& name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--input") {
      const char* value = need_value(arg);
      if (!value) return false;
      options->input_record = value;
    } else if (arg == "--output") {
      const char* value = need_value(arg);
      if (!value) return false;
      options->output_record = value;
    } else if (arg == "--config") {
      const char* value = need_value(arg);
      if (!value) return false;
      options->config_files.push_back(value);
    } else if (arg == "--scan_channel") {
      const char* value = need_value(arg);
      if (!value) return false;
      options->scan_channel = value;
    } else if (arg == "--pointcloud_channel" || arg == "--packed_channel") {
      const char* value = need_value(arg);
      if (!value) return false;
      options->packed_channel = value;
    } else if (arg == "--cloud_channel") {
      const char* value = need_value(arg);
      if (!value) return false;
      options->cloud_channel = value;
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return false;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }

  return !options->input_record.empty() && !options->output_record.empty() &&
         !options->config_files.empty();
}

LidarType ConvertToLidarType(const std::string& lidar_type) {
  static const std::unordered_map<std::string, LidarType> lidar_map = {
      {"RS16", LidarType::RS16},
      {"RS32", LidarType::RS32},
      {"RSBP", LidarType::RSBP},
      {"RSHELIOS", LidarType::RSHELIOS},
      {"RSHELIOS_16P", LidarType::RSHELIOS_16P},
      {"RS48", LidarType::RS48},
      {"RS80", LidarType::RS80},
      {"RS128", LidarType::RS128},
      {"RSP128", LidarType::RSP128},
      {"RSP80", LidarType::RSP80},
      {"RSP48", LidarType::RSP48},
      {"RSM1", LidarType::RSM1},
      {"RSM1P", LidarType::RSM1},
      {"RSM1_JUMBO", LidarType::RSM1_JUMBO},
      {"RSM2", LidarType::RSM2},
      {"RSM3", LidarType::RSM3},
      {"RSE1", LidarType::RSE1},
      {"RSMX", LidarType::RSMX},
      {"RSAIRY", LidarType::RSAIRY},
  };
  auto iter = lidar_map.find(lidar_type);
  if (iter == lidar_map.end()) {
    std::cerr << "Unsupported Robosense lidar model: " << lidar_type << "\n";
    return LidarType::RS32;
  }
  return iter->second;
}

void FillDriverParam(const Config& conf, RSDriverParam* param) {
  param->input_type = InputType::RAW_PACKET;
  param->frame_id = conf.frame_id();
  param->decoder_param.min_distance = conf.min_distance();
  param->decoder_param.max_distance = conf.max_distance();
  param->decoder_param.start_angle = conf.start_angle();
  param->decoder_param.end_angle = conf.end_angle();
  param->decoder_param.dense_points = conf.is_dense();
  param->decoder_param.split_angle = conf.split_angle();
  param->decoder_param.split_frame_mode =
      static_cast<SplitFrameMode>(conf.split_frame_mode());
  param->decoder_param.use_lidar_clock = conf.use_lidar_clock();
  param->lidar_type = ConvertToLidarType(conf.model());
}

PointCloudPacked ToPointCloudPacked(const std::shared_ptr<PointCloudMsg>& msg) {
  PointCloudPacked point_clouds;
  point_clouds.mutable_header()->set_sequence_num(msg->seq);
  point_clouds.mutable_header()->set_timestamp_sec(
      msg->points.empty() ? msg->timestamp : msg->points[0].timestamp);
  point_clouds.set_measuretime(century::cyber::Time::Now().ToSecond());
  point_clouds.set_width(msg->width);
  point_clouds.set_height(msg->height);
  point_clouds.set_is_dense(msg->is_dense);
  point_clouds.set_point_size(msg->points.size());

  auto* data = point_clouds.mutable_data();
  data->resize(msg->points.size() * sizeof(PointPacked));
  if (msg->points.empty()) {
    return point_clouds;
  }
  auto* point_data = reinterpret_cast<PointPacked*>(&(*data)[0]);
  for (size_t i = 0; i < msg->points.size(); ++i) {
    const auto& src = msg->points[i];
    point_data[i].x = src.x;
    point_data[i].y = src.y;
    point_data[i].z = src.z;
    point_data[i].intensity = static_cast<uint16_t>(src.intensity);
    point_data[i].ring = static_cast<uint16_t>(src.ring);
    point_data[i].timestamp = src.timestamp;
  }
  return point_clouds;
}

PointXYZIRTCloud ToPointCloud(const std::shared_ptr<PointCloudMsg>& msg) {
  PointXYZIRTCloud point_clouds;
  point_clouds.mutable_header()->set_sequence_num(msg->seq);
  point_clouds.mutable_header()->set_timestamp_sec(
      msg->points.empty() ? msg->timestamp : msg->points[0].timestamp);
  point_clouds.set_frame_id(msg->frame_id);
  point_clouds.set_measuretime(century::cyber::Time::Now().ToSecond());
  point_clouds.set_width(msg->width);
  point_clouds.set_height(msg->height);
  point_clouds.set_is_dense(msg->is_dense);
  point_clouds.mutable_point()->Reserve(msg->points.size());

  for (const auto& src : msg->points) {
    auto* point = point_clouds.add_point();
    point->set_x(src.x);
    point->set_y(src.y);
    point->set_z(src.z);
    point->set_intensity(src.intensity);
    point->set_ring(src.ring);
    point->set_timestamp(src.timestamp);
  }
  return point_clouds;
}

class ScanRecordConverter {
 public:
  ScanRecordConverter(const Config& conf, const std::string& packed_channel,
                      const std::string& cloud_channel,
                      const std::string& scan_channel)
      : scan_channel_(scan_channel),
        packed_channel_(packed_channel),
        cloud_channel_(cloud_channel) {
    driver_.regPointCloudCallback(
        std::bind(&ScanRecordConverter::GetPointCloud, this),
        std::bind(&ScanRecordConverter::OnPointCloud, this,
                  std::placeholders::_1));

    RSDriverParam param;
    FillDriverParam(conf, &param);
    if (!driver_.init(param)) {
      throw std::runtime_error(
          "failed to initialize Robosense RAW_PACKET driver");
    }
    driver_.start();
  }

  ~ScanRecordConverter() { driver_.stop(); }

  void ConvertScan(const RobosenseScan& scan, uint64_t record_time,
                   RecordWriter* writer) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      writer_ = writer;
      pending_times_.push_back(record_time);
    }
    for (const auto& scan_pkt : scan.firing_pkts()) {
      Packet pkt;
      pkt.timestamp = scan_pkt.stamp() * 1e-9;
      pkt.buf_.assign(scan_pkt.data().begin(), scan_pkt.data().end());
      driver_.decodePacket(pkt);
    }
  }

  uint64_t written_frames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return written_frames_;
  }

  const std::string& scan_channel() const { return scan_channel_; }

  size_t pending_frames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_times_.size();
  }

 private:
  std::shared_ptr<PointCloudMsg> GetPointCloud() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto msg = free_clouds_.empty() ? std::make_shared<PointCloudMsg>()
                                    : free_clouds_.front();
    if (!free_clouds_.empty()) {
      free_clouds_.pop_front();
    }
    return msg;
  }

  void OnPointCloud(std::shared_ptr<PointCloudMsg> msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_) {
      return;
    }
    uint64_t record_time = 0;
    if (!pending_times_.empty()) {
      record_time = pending_times_.front();
      pending_times_.pop_front();
    } else {
      record_time = static_cast<uint64_t>(msg->timestamp * 1e9);
    }
    PointCloudPacked packed = ToPointCloudPacked(msg);
    PointXYZIRTCloud cloud = ToPointCloud(msg);
    if (!packed_channel_.empty()) {
      writer_->WriteMessage(packed_channel_, packed, record_time);
    }
    if (!cloud_channel_.empty()) {
      writer_->WriteMessage(cloud_channel_, cloud, record_time);
    }
    ++written_frames_;
    free_clouds_.push_back(msg);
  }

  std::string scan_channel_;
  std::string packed_channel_;
  std::string cloud_channel_;
  RecordWriter* writer_ = nullptr;
  LidarDriver<PointCloudMsg> driver_;
  mutable std::mutex mutex_;
  std::deque<uint64_t> pending_times_;
  std::deque<std::shared_ptr<PointCloudMsg>> free_clouds_;
  uint64_t written_frames_ = 0;
};

std::string DefaultPointCloudChannel(const std::string& scan_channel,
                                     const Config& conf) {
  constexpr char kScanSuffix[] = "/Scan";
  if (scan_channel.size() > sizeof(kScanSuffix) - 1 &&
      scan_channel.compare(scan_channel.size() - (sizeof(kScanSuffix) - 1),
                           sizeof(kScanSuffix) - 1, kScanSuffix) == 0) {
    return scan_channel.substr(0,
                               scan_channel.size() - (sizeof(kScanSuffix) - 1));
  }
  return conf.pointcloud_channel();
}

std::string DefaultCloudChannel(const std::string& packed_channel) {
  if (packed_channel.empty()) {
    return "";
  }
  return packed_channel + "/PointCloud";
}

}  // namespace
}  // namespace robosense
}  // namespace drivers
}  // namespace century

int main(int argc, char* argv[]) {
  century::cyber::Init(argv[0]);

  century::drivers::robosense::Options options;
  if (!century::drivers::robosense::ParseArgs(argc, argv, &options)) {
    century::drivers::robosense::PrintUsage(argv[0]);
    return 1;
  }

  ::century::cyber::record::RecordReader reader(options.input_record);
  if (!reader.IsValid()) {
    std::cerr << "Failed to open input record: " << options.input_record
              << "\n";
    return 1;
  }

  ::century::cyber::record::RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  if (!writer.Open(options.output_record)) {
    std::cerr << "Failed to open output record: " << options.output_record
              << "\n";
    return 1;
  }

  struct ConverterEntry {
    std::string scan_channel;
    std::unique_ptr<century::drivers::robosense::ScanRecordConverter> converter;
  };
  std::vector<ConverterEntry> converters;
  converters.reserve(options.config_files.size());
  for (const auto& config_file : options.config_files) {
    century::drivers::robosense::Config local_conf;
    if (!century::cyber::common::GetProtoFromFile(config_file, &local_conf)) {
      std::cerr << "Failed to read config: " << config_file << "\n";
      return 1;
    }
    std::string scan_channel =
        options.scan_channel.empty() ? local_conf.scan_channel()
                                     : options.scan_channel;
    std::string packed_channel = century::drivers::robosense::DefaultPointCloudChannel(
        scan_channel, local_conf);
    std::string cloud_channel = options.cloud_channel.empty()
                                    ? century::drivers::robosense::DefaultCloudChannel(
                                          packed_channel)
                                    : options.cloud_channel;
    converters.push_back(ConverterEntry{
        scan_channel,
        std::make_unique<century::drivers::robosense::ScanRecordConverter>(
            local_conf, packed_channel, cloud_channel, scan_channel)});
  }

  ::century::cyber::record::RecordMessage message;
  uint64_t scan_count = 0;
  while (reader.ReadMessage(&message)) {
    bool handled = false;
    for (auto& entry : converters) {
      if (message.channel_name != entry.scan_channel) {
        continue;
      }
      century::drivers::robosense::RobosenseScan scan;
      if (!scan.ParseFromString(message.content)) {
        std::cerr << "Failed to parse RobosenseScan at record time "
                  << message.time << "\n";
        return 1;
      }
      writer.WriteMessage(entry.scan_channel, scan, message.time);
      entry.converter->ConvertScan(scan, message.time, &writer);
      handled = true;
      break;
    }
    if (!handled) {
      continue;
    }
    ++scan_count;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  uint64_t total_written_frames = 0;
  for (const auto& entry : converters) {
    total_written_frames += entry.converter->written_frames();
    if (entry.converter->pending_frames() > 0) {
      std::cerr << "Warning: " << entry.converter->pending_frames()
                << " scan frames are still pending for channel "
                << entry.scan_channel
                << ". The last frame may need one more scan frame to trigger "
                   "rs_driver split.\n";
    }
  }
  writer.Close();

  std::cout << "Converted scan frames: " << scan_count
            << ", wrote pointcloud frames: " << total_written_frames
            << ", output: " << options.output_record << "\n";
  return 0;
}
