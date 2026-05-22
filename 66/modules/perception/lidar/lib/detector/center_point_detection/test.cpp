#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <fstream>
#include <memory>
#include <chrono>
#include <dirent.h>
#include "modules/perception/lidar/lib/detector/center_point_detection/src/centerpoint.h"
#include <memory>

std::string Save_Dir   = "./data/prediction/";

namespace century {
namespace perception {
namespace lidar {
namespace centerpoint {

bool hasEnding(std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int getFolderFile(const char *path, std::vector<std::string>& files, const char *suffix = ".bin")
{
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(path)) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            std::string file = ent->d_name;
            if(hasEnding(file, suffix)){
                files.push_back(file.substr(0, file.length()-4));
            }
        }
        closedir(dir);
    } else {
        printf("No such folder: %s.", path);
        exit(EXIT_FAILURE);
    }
    return EXIT_SUCCESS;
}

int loadData(const char *file, void **data, unsigned int *length)
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

void SaveBoxPred(std::vector<Bndbox> boxes, std::string file_name)
{
    std::ofstream ofs;
    ofs.open(file_name, std::ios::out);
    ofs.setf(std::ios::fixed, std::ios::floatfield);
    ofs.precision(5);
    if (ofs.is_open()) {
        for (const auto box : boxes) {
          ofs << box.x << " ";
          ofs << box.y << " ";
          ofs << box.z << " ";
          ofs << box.w << " ";
          ofs << box.l << " ";
          ofs << box.h << " ";
        //   ofs << box.vx << " ";
        //   ofs << box.vy << " ";
          ofs << -box.rt - M_PI / 2  << " ";
          ofs << box.id << " ";
          ofs << box.score << " ";
          ofs << "\n";
        }
    }
    else {
      std::cerr << "Output file cannot be opened!" << std::endl;
    }
    ofs.close();
    std::cout << "Saved prediction in: " << file_name << std::endl;
    return;
}

static bool startswith(const char *s, const char *with, const char **last)
{
    while (*s++ == *with++)
    {
        if (*s == 0 || *with == 0)
            break;
    }
    if (*with == 0)
        *last = s + 1;
    return *with == 0;
}

static void help()
{
    printf(
        "Usage: \n"
        "    ./centerpoint_infer ../data/test/\n"
        "    Run centerpoint(voxelnet) inference with data under ../data/test/\n"
        "    Optional: --verbose, enable verbose log level\n"
    );
    exit(EXIT_SUCCESS);
}

void print() {
  std::cout << "Hello, world!" << std::endl;
  
  std::shared_ptr<CenterPointVoxel> test_center;
  test_center.reset(new CenterPointVoxel("test.cfg", "../model/pcdet_neck_head_sim", "../model/centerpoint_pre.scn.onnx"));


  
}





} // namespace centerpoint
} // namespace lidar
} // namespace perception
} // namespace optimus

using namespace century::perception::lidar::lidar_detector::centerpoint;

int main(int argc, char** argv) {
  // optimus::perception::lidar::lidar_detector::centerpoint::print();
  // std::cout << "Hello, world!" << std::endl;

  if (argc < 2)
        help();

  const char *value = nullptr;
  // bool verbose = false;
  for (int i = 2; i < argc; ++i) {
      if (startswith(argv[i], "--verbose", &value)) {
          // verbose = true;
      } else {
          help();
      }
  }

  const char *data_folder  = argv[1];

  // GetDeviceInfo();

  std::vector<std::string> files;
  getFolderFile(data_folder, files);

  std::cout << "Total " << files.size() << std::endl;

  Params params;
  cudaStream_t stream = NULL;
  checkCudaErrors(cudaStreamCreate(&stream));

  std::shared_ptr<CenterPointVoxel> test_center;
  test_center.reset(new CenterPointVoxel("test.cfg", "./data/model/pcdet_neck_head_sim", "./data/model/centerpoint_pre.scn.onnx"));

  float *d_points = nullptr;    
  checkCudaErrors(cudaMalloc((void **)&d_points, MAX_POINTS_NUM * params.feature_num * sizeof(float)));
  for (const auto & file : files)
  {
      std::string dataFile = data_folder + file + ".bin";

      std::cout << "\n<<<<<<<<<<<" <<std::endl;
      std::cout << "load file: "<< dataFile <<std::endl;

      unsigned int length = 0;
      void *pc_data = NULL;

      loadData(dataFile.c_str() , &pc_data, &length);
      size_t points_num = length / (params.feature_num * sizeof(float)) ;
      std::cout << "find points num: " << points_num << std::endl;

      checkCudaErrors(cudaMemcpy(d_points, pc_data, length, cudaMemcpyHostToDevice));
      std::cout << "load data done." << std::endl;

      test_center->doinfer((float *)d_points, points_num, stream);

      std::string save_file_name = Save_Dir + file + ".txt";
      std::cout << "save file: " << save_file_name << std::endl;
      SaveBoxPred(test_center->getbboxes(), save_file_name);

      std::cout << ">>>>>>>>>>>" <<std::endl;
  }

  // centerpoint.perf_report();
  checkCudaErrors(cudaFree(d_points));
  checkCudaErrors(cudaStreamDestroy(stream));
  return 0;
}
