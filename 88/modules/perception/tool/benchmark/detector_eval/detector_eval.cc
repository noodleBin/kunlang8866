// boxes3d n* 9
// label 3d n,
// score  n
#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <array>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/app/lidar_obstacle_detector.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/singleton.h"

namespace fs = boost::filesystem;
using namespace century::perception::base;

const std::map<ObjectSubType, int> kSubType2Label = {
  // {ObjectSubType::UNKNOWN, ObjectType::UNKNOWN},
  // {ObjectSubType::UNKNOWN_MOVABLE, ObjectType::UNKNOWN_MOVABLE},
  // {ObjectSubType::UNKNOWN_UNMOVABLE, ObjectType::UNKNOWN_UNMOVABLE},
  {ObjectSubType::PEDESTRIAN, 0},
  {ObjectSubType::CAR, 1},
  {ObjectSubType::IGV_FULL, 2},
  {ObjectSubType::TRUCK, 3},
  {ObjectSubType::TRAILER_EMPTY, 4},
  {ObjectSubType::TRAILER_FULL, 5},
  {ObjectSubType::IGV_EMPTY, 6},
  {ObjectSubType::CRANE, 7},
  {ObjectSubType::OTHER_VEHICLE, 8},
  {ObjectSubType::CONE, 9},
  {ObjectSubType::CONTAINER_FORKLIFT, 10},
  {ObjectSubType::FORKLIFT, 11},
  {ObjectSubType::LORRY, 12},
  {ObjectSubType::CONSTRUCTION_VEHICLE, 13},
  {ObjectSubType::WHEELCRANE, 14},
};

class BinFileProcessor {
private:
  // Structure to store information about each .bin file
  struct BinFileInfo {
    std::string path;    // Full file path
    int number;          // Number extracted from filename
    size_t size;         // File size in bytes
    
    // Comparison operator for sorting by file number
    bool operator<(const BinFileInfo& other) const {
      return number < other.number;
    }
  };

public:
  // Format file size to human-readable string (B, KB, MB, GB)
  static std::string format_file_size(size_t bytes) {
    const char* suffixes[] = {"B", "KB", "MB", "GB"};
    size_t suffix_index = 0;
    double size = static_cast<double>(bytes);
    
    while (size >= 1024 && suffix_index < 3) {
      size /= 1024;
      suffix_index++;
    }
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << size << " " << suffixes[suffix_index];
    return oss.str();
  }

public:
  // Get information about all .bin files in a folder
  static std::vector<BinFileInfo> get_bin_files_info(const std::string& folder_path) {
    std::vector<BinFileInfo> file_infos;
    
    try {
      fs::path dir_path(folder_path);
      
      if (!fs::exists(dir_path)) {
        std::cerr << "Error: Path does not exist - " << folder_path << std::endl;
        return file_infos;
      }
      
      if (!fs::is_directory(dir_path)) {
        std::cerr << "Error: Not a directory - " << folder_path << std::endl;
        return file_infos;
      }
      
      // Iterate through directory
      for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (fs::is_regular_file(entry.status())) {
          fs::path file_path = entry.path();
          
          // Check if file has .bin extension
          if (file_path.extension() == ".bin") {
            std::string stem = file_path.stem().string();
            
            // Try to parse number from filename
            try {
              int file_number = boost::lexical_cast<int>(stem);
              
              // Get file size
              size_t file_size = fs::file_size(file_path);
              
              file_infos.push_back({
                file_path.string(),
                file_number,
                file_size
              });
              
            } catch (const boost::bad_lexical_cast&) {
              // Filename is not a pure number, skip this file
              continue;
            }
          }
        }
      }
      
      // Sort by number in filename
      std::sort(file_infos.begin(), file_infos.end());
      
    } catch (const fs::filesystem_error& e) {
      std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
    
    return file_infos;
  }
  
  // Read single .bin file containing n×4 float32 array
  static std::vector<std::array<float, 4>> read_bin_file(const std::string& filepath) {
    std::vector<std::array<float, 4>> data;
    
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
      std::cerr << "Cannot open file: " << filepath << std::endl;
      return data;
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    // Check if file size is valid for float32 array (should be multiple of 4)
    if (file_size % sizeof(float) != 0) {
      std::cerr << "Warning: File size is not multiple of float32 size" << std::endl;
    }
    
    size_t num_floats = file_size / sizeof(float);
    size_t num_points = num_floats / 4;  // Each point has 4 floats
    
    // Reserve space for data
    data.resize(num_points);
    
    // Read all data at once
    file.read(reinterpret_cast<char*>(data.data()), file_size);
    
    if (!file) {
      std::cerr << "File read incomplete: " << filepath << std::endl;
    }
    
    file.close();
    return data;
  }
  
  // Process all .bin files in sequential order
  static void process_sequentially(const std::string& folder_path, const std::string& out_path) {
    std::cout << "Scanning folder: " << folder_path << std::endl;
    century::perception::base::Singleton<std::mutex>::Instance();
    std::shared_ptr<century::perception::lidar::BaseLidarDetector> detector;
    century::perception::lidar::BaseLidarDetector* detector_use =
      century::perception::lidar::BaseLidarDetectorRegisterer::GetInstanceByName("CenterPointDetector");
    CHECK_NOTNULL(detector_use);
    detector.reset(detector_use);
    century::perception::lidar::LidarDetectorInitOptions detection_init_options;
    detection_init_options.sensor_name = "helios_bp_32";
    detection_init_options.cfg_file = "/century/modules/perception/production/conf/perception/lidar";
    ACHECK(detector->Init(detection_init_options))
      << "lidar detector init error";

    century::perception::lidar::LidarDetectorOptions detection_options;

    auto file_infos = get_bin_files_info(folder_path);
    
    if (file_infos.empty()) {
      std::cout << "No .bin files found" << std::endl;
      return;
    }
    
    std::cout << "Found " << file_infos.size() << " files" << std::endl;
    std::cout << "Starting sequential processing..." << std::endl;
    
    // Calculate and display total size
    size_t total_size = 0;
    for (const auto& info : file_infos) {
      total_size += info.size;
    }
    std::cout << "Total file size: " << format_file_size(total_size) << "\n" << std::endl;
    
    // Process each file in order
    for (size_t i = 0; i < file_infos.size(); ++i) {
      const auto& info = file_infos[i];

      std::cout << "[" << std::setw(3) << (i + 1) << "/" 
                << std::setw(3) << file_infos.size() << "] "
                << "Processing: " << fs::path(info.path).filename().string()
                << " (" << format_file_size(info.size) << ")" << std::endl;
      
      // Read file
      auto point_data = read_bin_file(info.path);
      std::string out_put_file = out_path + "/" + fs::path(info.path).stem().string() + ".txt";
      std::ofstream file(out_put_file);
      if (!file.is_open()) {
        std::cerr << "Cannot open file: " << out_put_file << std::endl;
        return;
      }
      if (!point_data.empty()) {
        std::cout << "  Read " << point_data.size() << " points" << std::endl;
        
        // Add your processing logic here
        std::shared_ptr<century::perception::lidar::LidarFrame> frame = century::perception::lidar::LidarFramePool::Instance().Get();
        frame->raw_cloud = century::perception::base::PointFCloudPool::Instance().Get();
        const size_t point_size = point_data.size();
        for (size_t j = 0; j < point_size; ++j) {
          const auto& point = point_data[j];
          century::perception::base::PointF detect_point;
          detect_point.x = point[0];
          detect_point.y = point[1];
          detect_point.z = point[2];
          detect_point.intensity = point[3];
          frame->raw_cloud->push_back(detect_point);
        }
        frame->raw_cloud->resize(frame->raw_cloud->size());

        bool ret =
          detector->Detect(detection_options, frame.get());
        //[x, y, z, h, w, l, ry]
        for (auto& obj : frame->segmented_objects) {
          file << obj->center(0) << " " << obj->center(1) << " " << obj->center(2) << " " 
               << obj->size(2) << " " << obj-> size(1) << " " << obj->size(0) << " "
               << obj->theta << " " << obj->confidence << " " << (kSubType2Label.at(obj->sub_type)) << "\n";  
        }
          

        std::cout << ret << std::endl;
      }
      
      file.close();
      // if (i == 2)
      //   break;
    }
    
    std::cout << "Processing completed!" << std::endl;
  }
};

// bool ensure_folder_exists(const std::string& folder_path) {
//   try {
//     fs::path dir_path(folder_path);
    
//     // Check if path exists
//     if (fs::exists(dir_path)) {
//       if (fs::is_directory(dir_path)) {
//         std::cout << "Folder exists: " << folder_path << std::endl;
//         return true;
//       } else {
//         std::cerr << "Error: Path exists but is not a folder: " << folder_path << std::endl;
//         return false;
//       }
//     }
    
//     // Create folder (including parent directories)
//     bool created = fs::create_directories(dir_path);
    
//     if (created) {
//       std::cout << "Created folder: " << folder_path << std::endl;
//       return true;
//     } else {
//       std::cerr << "Failed to create folder: " << folder_path << std::endl;
//       return false;
//     }
    
//   } catch (const fs::filesystem_error& e) {
//     std::cerr << "Filesystem error: " << e.what() << std::endl;
//     return false;
//   }
// }
bool ensure_folder_exists(const std::string& folder_path, bool verbose = true) {
  try {
    fs::path dir(folder_path);
    
    // Remove if exists
    if (fs::exists(dir)) {
      if (fs::is_directory(dir)) {
        // Remove directory and all contents
        fs::remove_all(dir);
        if (verbose) {
          std::cout << "Removed existing directory: " << folder_path << std::endl;
        }
      } else {
        // Remove file if it's not a directory
        fs::remove(dir);
        if (verbose) {
          std::cout << "Removed file (not directory): " << folder_path << std::endl;
        }
      }
    }
    
    // Create directory
    return fs::create_directories(dir);
    
  } catch (const std::exception& e) {
    std::cerr << "Error recreating folder " << folder_path << ": " << e.what() << std::endl;
    return false;
  }
}


int main() {
  // Set folder path
  std::string data_folder = "/century/data/infer_data";
  std::string out_folder = data_folder + "_out";
  if (!ensure_folder_exists(out_folder)) {
    return 0;
  }
  // Method 1: Get file information
  auto files_info = BinFileProcessor::get_bin_files_info(data_folder);
  
  std::cout << "=== File Statistics ===" << std::endl;
  std::cout << "Total files: " << files_info.size() << std::endl;
  
  // if (!files_info.empty()) {
  //   std::cout << "\nFile list (sorted by number):" << std::endl;
  //   std::cout << std::setw(6) << "Index" 
  //             << std::setw(10) << "File No"
  //             << std::setw(12) << "Size"
  //             << std::setw(20) << "Filename" << std::endl;
  //   std::cout << std::string(50, '-') << std::endl;
    
  //   for (size_t i = 0; i < files_info.size(); ++i) {
  //     const auto& info = files_info[i];
  //     std::cout << std::setw(6) << i + 1
  //               << std::setw(10) << info.number
  //               << std::setw(12) << BinFileProcessor::format_file_size(info.size)
  //               << std::setw(20) << fs::path(info.path).filename().string() 
  //               << std::endl;
  //   }
  // }
  
  // Method 2: Process all files sequentially
  std::cout << "\n=== Starting Sequential Processing ===" << std::endl;
  BinFileProcessor::process_sequentially(data_folder, out_folder);
  
  return 0;
}