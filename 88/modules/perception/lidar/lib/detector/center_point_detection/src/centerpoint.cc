#include "centerpoint.h"

#include <dlfcn.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/time.h>

using std::chrono::duration;
using std::chrono::high_resolution_clock;

namespace century {
namespace perception {
namespace lidar {
namespace centerpoint {


int loadData1(const char *file, void **data, unsigned int *length)
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

void CenterPointVoxel::get_info(void) {
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1],
           prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1],
           prop.maxGridSize[2]);
  }
  printf("\n");
}

CenterPointVoxel::CenterPointVoxel(CenterPointDetectionConfig& centerpoint_config) {
  // init base info
  get_info();
  checkCudaErrors(cudaStreamCreate(&stream_));

  {
    // 1. load config
      params_.load_config(centerpoint_config);
      d_reg_.assign(params_.num_tasks, nullptr);
      d_height_.assign(params_.num_tasks, nullptr);
      d_dim_.assign(params_.num_tasks, nullptr);
      d_rot_.assign(params_.num_tasks, nullptr);
      d_vel_.assign(params_.num_tasks, nullptr);
      d_hm_.assign(params_.num_tasks, nullptr);

    // 2. init scn
    {
      // 2.1 check if the model file exists
      std::string centerpoint_scn_onnx_file = FLAGS_centerpoint_engine_path + "centerpoint_pre.scn.onnx";
      if (access(centerpoint_scn_onnx_file.c_str(), F_OK) == -1) {
        AERROR << "centerpoint_scn_onnx_file model file not found: " << centerpoint_scn_onnx_file;
        abort();
      }

      // 2.2 create the scn engine's voxelization parameter
      VoxelizationParameter voxelization;
      voxelization.min_range =
          nvtype::Float3(params_.min_x_range, params_.min_y_range, params_.min_z_range);
      voxelization.max_range =
          nvtype::Float3(params_.max_x_range, params_.max_y_range, params_.max_z_range);
      voxelization.voxel_size =
          nvtype::Float3(params_.pillar_x_size, params_.pillar_y_size, params_.pillar_z_size);
      voxelization.grid_size = voxelization.compute_grid_size(
          voxelization.max_range, voxelization.min_range, voxelization.voxel_size);
      voxelization.max_points_per_voxel = params_.max_points_per_voxel;
      voxelization.max_points = params_.max_points;
      voxelization.max_voxels = params_.max_voxels;
      voxelization.num_feature = params_.feature_num;

      // 2.3 create the scn engine
      lidar_scn_param_.voxelization = voxelization;
      lidar_scn_param_.model = centerpoint_scn_onnx_file;
      lidar_scn_param_.order = CoordinateOrder::ZYX;
      lidar_scn_param_.precision = Precision::Float16;

      capacity_points_ = 300000;
      bytes_capacity_points_ =
          capacity_points_ * lidar_scn_param_.voxelization.num_feature * sizeof(nvtype::half);
      checkRuntime(cudaMalloc(&lidar_points_device_, bytes_capacity_points_));
      checkRuntime(cudaMallocHost(&lidar_points_host_, bytes_capacity_points_));

      lidar_scn_ = create_scn(lidar_scn_param_);
      if (lidar_scn_ == nullptr) {
        AERROR << "Failed to create lidar scn.";
        abort();
      }
    }

    // 3. init load rpn_centerhead_sim file
    {
      // 3.1 check if the model file with .plan extension exists
      // rpn_centerhead_sim_onnx_file end with .onnx , but we need to load the .plan file
      std::string trt_plan_file;
#ifdef __aarch64__
      trt_plan_file = FLAGS_centerpoint_engine_path + "pcdet_neck_head_sim_arm.plan";
#else
      trt_plan_file = FLAGS_centerpoint_engine_path + "pcdet_neck_head_sim_x86.plan";
#endif

      if (access(trt_plan_file.c_str(), F_OK) == -1) {
        AERROR << "plan file not found " << trt_plan_file << std::endl;
        abort();
      } else {
        AINFO << "plan file found: " << trt_plan_file << std::endl;
      }

      // 3.3 load the trt plan file
      trt_ = TensorRT::load_plan(trt_plan_file);
      if (trt_ == nullptr) {
        AERROR << "Failed to load trt plan file.";
        abort();
      }
    }

    // 4. init postprocess
    {
      post_.reset(new PostProcessCuda(centerpoint_config));
      // post_.reset(new PostProcessCuda());
      if (post_ == nullptr) {
        AERROR << "Failed to create postprocess.";
        abort();
      }
    }
  }

  checkCudaErrors(
      cudaMallocHost(reinterpret_cast<void**>(&h_detections_num_), sizeof(unsigned int)));
  checkCudaErrors(cudaMemset(h_detections_num_, 0, sizeof(unsigned int)));
  checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&d_detections_),
                             MAX_DET_NUM * DET_CHANNEL * sizeof(float)));
  checkCudaErrors(cudaMemset(d_detections_, 0, MAX_DET_NUM * DET_CHANNEL * sizeof(float)));

  // add d_detections_reshape_
  checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&d_detections_reshape_),
                             MAX_DET_NUM * DET_CHANNEL * sizeof(float)));
  checkCudaErrors(cudaMemset(d_detections_reshape_, 0, MAX_DET_NUM * DET_CHANNEL * sizeof(float)));

  detections_.resize(MAX_DET_NUM,
                     {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

  for (unsigned int i = 0; i < params_.num_tasks; i++) {
    checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&d_reg_[i]),
                               trt_->getBindingNumel("reg_" + std::to_string(i)) * sizeof(half)));
    checkCudaErrors(
        cudaMalloc(reinterpret_cast<void**>(&d_height_[i]),
                   trt_->getBindingNumel("height_" + std::to_string(i)) * sizeof(half)));
    checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&d_dim_[i]),
                               trt_->getBindingNumel("dim_" + std::to_string(i)) * sizeof(half)));
    checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&d_rot_[i]),
                               trt_->getBindingNumel("rot_" + std::to_string(i)) * sizeof(half)));
    // checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&d_vel_[i]),
    //                            trt_->getBindingNumel("vel_" + std::to_string(i)) * sizeof(half)));
    checkCudaErrors(cudaMalloc(reinterpret_cast<void**>(&d_hm_[i]),
                               trt_->getBindingNumel("hm_" + std::to_string(i)) * sizeof(half)));

    if (i == 0) {
      auto d = trt_->getBindingDims("reg_" + std::to_string(i));
      reg_n_ = d[0];
      reg_c_ = d[1];
      reg_h_ = d[2];
      reg_w_ = d[3];

      d = trt_->getBindingDims("height_" + std::to_string(i));
      height_c_ = d[1];
      d = trt_->getBindingDims("dim_" + std::to_string(i));
      dim_c_ = d[1];
      d = trt_->getBindingDims("rot_" + std::to_string(i));
      rot_c_ = d[1];
      // d = trt_->getBindingDims("vel_" + std::to_string(i));
      // vel_c_ = d[1];
    }
    auto d = trt_->getBindingDims("hm_" + std::to_string(i));
    hm_c_[i] = d[1];
  }
  h_mask_size_ = params_.nms_pre_max_size * DIVUP(params_.nms_pre_max_size, NMS_THREADS_PER_BLOCK) *
                 sizeof(uint64_t);
  checkCudaErrors(cudaMallocHost(reinterpret_cast<void**>(&h_mask_), h_mask_size_));
  checkCudaErrors(cudaMemset(h_mask_, 0, h_mask_size_));
  AINFO << "center point init done" << std::endl;
  return;
}

int CenterPointVoxel::doinfer(float* lidar_arr, int lidar_num, cudaStream_t& stream) {
  // Step 1: lidar arr data type conversion
  // 1.1 cpu float32 to nv::Tensor

  std::vector<int64_t> lidar_shape{lidar_num, 4};
  auto lidar_data =
      nv::Tensor::from_data_reference((void*)lidar_arr, lidar_shape, nv::DataType::Float32, false);
  // 1.2 nv::Tensor to cuda half
  auto lidar_half = lidar_data.to_device().to_half().to_host();
  struct timeval scn_begin, trt_begin, trt_end, end;
  gettimeofday(&scn_begin, NULL);
  // Step 2: SCN(voxelization + feature extraction)
  const nvtype::half* lidar_feature =
      scn_forward(lidar_half.ptr<nvtype::half>(), lidar_data.size(0), stream);
  cudaStreamSynchronize(stream);
  gettimeofday(&trt_begin, NULL);
  long seconds = trt_begin.tv_sec - scn_begin.tv_sec;
  long useconds = trt_begin.tv_usec - scn_begin.tv_usec;
  double elapsed_ms =
      ((seconds)*1000 + useconds / 1000.0) + (useconds % 1000) / 1000.0;
  AINFO << "Detector Time taken voxel + scn: " << elapsed_ms;
  // Step 3: RPN + Head
  if (6 == params_.num_tasks) {
    trt_->forward({const_cast<nvtype::half*>(lidar_feature), d_reg_[0], d_height_[0], d_dim_[0], d_rot_[0],d_hm_[0],
                                          d_reg_[1], d_height_[1], d_dim_[1], d_rot_[1], d_hm_[1],
                                          d_reg_[2], d_height_[2], d_dim_[2], d_rot_[2], d_hm_[2],
                                          d_reg_[3], d_height_[3], d_dim_[3], d_rot_[3], d_hm_[3],
                                          d_reg_[4], d_height_[4], d_dim_[4], d_rot_[4], d_hm_[4],
                                          d_reg_[5], d_height_[5], d_dim_[5], d_rot_[5], d_hm_[5]}, stream);
  } else {
    trt_->forward({const_cast<nvtype::half*>(lidar_feature), d_reg_[0], d_height_[0], d_dim_[0], d_rot_[0],d_hm_[0]}, stream);
  }


  cudaStreamSynchronize(stream);
  gettimeofday(&trt_end, NULL);
  seconds = trt_end.tv_sec - trt_begin.tv_sec;
  useconds = trt_end.tv_usec - trt_begin.tv_usec;
  elapsed_ms =
      ((seconds)*1000 + useconds / 1000.0) + (useconds % 1000) / 1000.0;

  AINFO << "Detector Time taken trt head : " << elapsed_ms;
  // Step 4: Postprocess
  {
    nms_pred_.clear();

    for (unsigned int i_task = 0; i_task < params_.num_tasks; i_task++) {
      cudaStreamSynchronize(stream);
      checkCudaErrors(cudaMemset(h_detections_num_, 0, sizeof(unsigned int)));
      checkCudaErrors(cudaMemset(d_detections_, 0, MAX_DET_NUM * DET_CHANNEL * sizeof(float)));
      checkCudaErrors(
          cudaMemset(d_detections_reshape_, 0, MAX_DET_NUM * DET_CHANNEL * sizeof(float)));
      d_vel_[i_task] = nullptr;
      vel_c_ = 0;
      post_->doPostDecodeCuda(reg_n_, reg_h_, reg_w_, reg_c_, height_c_, dim_c_, rot_c_, vel_c_,
                              hm_c_[i_task], i_task, d_reg_[i_task], d_height_[i_task], d_dim_[i_task],
                              d_rot_[i_task], d_vel_[i_task], d_hm_[i_task], h_detections_num_,
                              d_detections_, stream);


      checkCudaErrors(cudaMemcpyAsync(detections_.data(), d_detections_,
                                      MAX_DET_NUM * DET_CHANNEL * sizeof(float),
                                      cudaMemcpyDeviceToHost, stream));
      checkCudaErrors(cudaStreamSynchronize(stream));
      if (*h_detections_num_ == 0) continue;

      std::sort(detections_.begin(), detections_.end(),
                [](float9 boxes1, float9 boxes2) { return boxes1.val[8] > boxes2.val[8]; });

      checkCudaErrors(cudaMemcpyAsync(d_detections_, detections_.data(),
                                      MAX_DET_NUM * DET_CHANNEL * sizeof(float),
                                      cudaMemcpyHostToDevice, stream));
      checkCudaErrors(cudaMemsetAsync(h_mask_, 0, h_mask_size_, stream));

      post_->doPermuteCuda(*h_detections_num_, d_detections_, d_detections_reshape_, stream);
      checkCudaErrors(cudaStreamSynchronize(stream));

      post_->doPostNMSCuda(*h_detections_num_, i_task, d_detections_reshape_, h_mask_, stream);
      checkCudaErrors(cudaStreamSynchronize(stream));

      int col_blocks = DIVUP(*h_detections_num_, NMS_THREADS_PER_BLOCK);
      std::vector<uint64_t> remv(col_blocks, 0);
      std::vector<bool> keep(*h_detections_num_, false);
      int max_keep_size = 0;
      for (unsigned int i_nms = 0; i_nms < *h_detections_num_; i_nms++) {
        unsigned int nblock = i_nms / NMS_THREADS_PER_BLOCK;
        unsigned int inblock = i_nms % NMS_THREADS_PER_BLOCK;

        if (!(remv[nblock] & (1ULL << inblock))) {
          keep[i_nms] = true;
          if (max_keep_size++ < static_cast<int>(params_.nms_post_max_size)) {
            nms_pred_.push_back(Bndbox(detections_[i_nms].val[0], detections_[i_nms].val[1], detections_[i_nms].val[2],
                                       detections_[i_nms].val[3], detections_[i_nms].val[4], detections_[i_nms].val[5],
                                       detections_[i_nms].val[6],
                                       params_.task_num_stride[i_task] + static_cast<int>(detections_[i_nms].val[7]), detections_[i_nms].val[8]));
            
          }

          uint64_t* p = h_mask_ + i_nms * col_blocks;
          for (int j_nms = nblock; j_nms < col_blocks; j_nms++) {
            remv[j_nms] |= p[j_nms];
          }
        }
      }
    }
  }
  gettimeofday(&end, NULL);
  seconds = end.tv_sec - trt_end.tv_sec;
  useconds = end.tv_usec - trt_end.tv_usec;
  elapsed_ms =
      ((seconds)*1000 + useconds / 1000.0) + (useconds % 1000) / 1000.0;
  AINFO << "Detector Time taken nms : " << elapsed_ms;
  AINFO << "nms _ pred_ size: " << nms_pred_.size();
  return 0;
}

std::vector<Bndbox>& CenterPointVoxel::getbboxes() {
  return nms_pred_;
}

const nvtype::half* CenterPointVoxel::scn_forward(const nvtype::half* lidar_points,
                                                  int num_points, cudaStream_t& stream) {
  int max_points = static_cast<int>(capacity_points_);
  if (num_points > max_points) {
    printf("If it exceeds %d points, the default processing will simply crop it out.\n", max_points);
  }

  num_points = std::min(max_points, num_points);

  // cudaStream_t _stream = static_cast<cudaStream_t>(stream);
  size_t bytes_points =
      num_points * lidar_scn_param_.voxelization.num_feature * sizeof(nvtype::half);
  checkRuntime(cudaMemcpyAsync(lidar_points_host_, lidar_points, bytes_points, cudaMemcpyHostToHost,
                               stream));
  checkRuntime(cudaMemcpyAsync(lidar_points_device_, lidar_points_host_, bytes_points,
                               cudaMemcpyHostToDevice, stream));

  const nvtype::half* lidar_feature =
      lidar_scn_->forward(lidar_points_device_, num_points, stream);
      
  return lidar_feature;
}

CenterPointVoxel::~CenterPointVoxel(void) {
  lidar_scn_.reset();
  trt_.reset();
  post_.reset();

  if (lidar_points_device_) checkRuntime(cudaFree(lidar_points_device_));
  if (lidar_points_host_) checkRuntime(cudaFreeHost(lidar_points_host_));

  checkCudaErrors(cudaFreeHost(h_detections_num_));
  checkCudaErrors(cudaFree(d_detections_));
  checkCudaErrors(cudaFree(d_detections_reshape_));

  for (unsigned int i = 0; i < params_.num_tasks; i++) {
    checkCudaErrors(cudaFree(d_reg_[i]));
    checkCudaErrors(cudaFree(d_height_[i]));
    checkCudaErrors(cudaFree(d_dim_[i]));
    checkCudaErrors(cudaFree(d_rot_[i]));
    checkCudaErrors(cudaFree(d_vel_[i]));
    checkCudaErrors(cudaFree(d_hm_[i]));
  }

  checkCudaErrors(cudaFreeHost(h_mask_));
  return;
}

}  // namespace centerpoint
}  // namespace lidar
}  // namespace perception
}  // namespace century
