#pragma once

#include <cassert>
#include <vector>
#include <vector_functions.hpp>
#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/macro.hpp"

namespace century {
namespace perception {
namespace camera {

struct ProcessConfig {
    bool swap_rb = false;
    float border_value = 114.0f;
    float3 alpha = make_float3(1.0 / 255.0f, 1.0 / 255.0f, 1.0 / 255.0f);
    float3 beta = make_float3(0.0f, 0.0f, 0.0f);
    void enableSwapRB() {
        this->swap_rb = true;
    }
    void setBorderValue(float border_value) {
        this->border_value = border_value;
    }
    void setNormalizeParams(const std::vector<float>& mean, const std::vector<float>& std) {
        assert(3 == mean.size() && 3 == std.size() &&
               "ProcessConfig: requires mean and std with size 3.");
        alpha.x = 1.0 / 255.0f / std[0];
        alpha.y = 1.0 / 255.0f / std[1];
        alpha.z = 1.0 / 255.0f / std[2];
        beta.x = -mean[0] / std[0];
        beta.y = -mean[1] / std[1];
        beta.z = -mean[2] / std[2];
    }
};

struct KLDEPLOY InferOption {
    int device_id = 0;
    bool cuda_mem = false;
    bool enable_managed_memory = false;
    bool enable_performance_report = false;
    std::vector<int> input_shape = {1, 3, 1080, 1920};
    ProcessConfig config;
    void setDeviceId(int id) {
        device_id = id;
    }
    void enableCudaMem() {
        cuda_mem = true;
    }
    void enableManagedMemory() {
        enable_managed_memory = true;
    }
    void enablePerformanceReport() {
        enable_performance_report = true;
    }
    void enableSwapRB() {
        config.enableSwapRB();
    }
    void setBorderValue(float border_value) {
        config.setBorderValue(border_value);
    }
    void setNormalizeParams(const std::vector<float>& mean, const std::vector<float>& std) {
        config.setNormalizeParams(mean, std);
    }
    void setInputDimensions(int width, int height) {
        input_shape = {1, 3, height, width};
    }
};

}
}
}
