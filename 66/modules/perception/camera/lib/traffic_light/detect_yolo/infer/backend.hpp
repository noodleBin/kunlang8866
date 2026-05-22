#pragma once

#include <NvInferRuntime.h>
#include <memory>
#include <string>
#include <vector>
#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/buffer.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/core.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/infer/warpaffine.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/option.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/result.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/macro.hpp"

namespace century {
namespace perception {
namespace camera {
class KLDEPLOY TrtBackend {
public:
    TrtBackend(const std::string& trt_engine_file, const InferOption& infer_option);
    TrtBackend() = default;
    ~TrtBackend();
    std::unique_ptr<TrtBackend> clone();
    void infer(const std::vector<Image>& inputs);
    cudaStream_t stream;
    InferOption option;
    std::vector<TensorInfo> tensor_infos;
    std::vector<AffineTransform> affine_transforms;
    int4 min_shape;
    int4 max_shape;
    bool dynamic;

 private:
    void getTensorInfo();
    void initialize();
    void captureCudaGraph();
    void dynamicInfer(const std::vector<Image>& inputs);
    void staticInfer(const std::vector<Image>& inputs);
    std::unique_ptr<TRTManager> manager_;
    CudaGraph cuda_graph_;
    std::unique_ptr<BaseBuffer> inputs_buffer_;
    BufferType buffer_type_;
    bool zero_copy_;
    int input_size_;
    int infer_size_;
};

}
}
}
