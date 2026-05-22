
#pragma once

#include <memory>
#include <string>
#include <vector>
#include "modules/perception/camera/lib/traffic_light/detect_yolo/infer/backend.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/utils/utils.hpp"
#include "option.hpp"
#include "result.hpp"

namespace century {
namespace perception {
namespace camera {

template <typename ResultType>
class BaseModel {
 public:
    BaseModel() = default;
    ~BaseModel() = default;
    explicit BaseModel(const std::string& trt_engine_file, const InferOption& infer_option)
        : backend_(std::make_unique<TrtBackend>(trt_engine_file, infer_option)) {
        if (backend_->option.enable_performance_report) {
            infer_gpu_trace_ = std::make_unique<GpuTimer>(backend_->stream);
            infer_cpu_trace_ = std::make_unique<CpuTimer>();
        }
    }
    std::unique_ptr<BaseModel<ResultType>> clone() const;
    ResultType predict(const Image& image);
    std::vector<ResultType> predict(const std::vector<Image>& images);
    std::tuple<std::string, std::string, std::string> performanceReport();
    int batch_size() const;
 protected:
    ResultType postProcess(int idx);
    std::unique_ptr<TrtBackend> backend_;
    unsigned long long total_request_{0};
    std::unique_ptr<GpuTimer> infer_gpu_trace_;
    std::unique_ptr<CpuTimer> infer_cpu_trace_;
};

template class BaseModel<ClassifyRes>;
template class BaseModel<DetectRes>;
template class BaseModel<OBBRes>;
template class BaseModel<SegmentRes>;
template class BaseModel<PoseRes>;

typedef BaseModel<ClassifyRes> ClassifyModel;
typedef BaseModel<DetectRes> DetectModel;
typedef BaseModel<OBBRes> OBBModel;
typedef BaseModel<SegmentRes> SegmentModel;
typedef BaseModel<PoseRes> PoseModel;

}
}
}
