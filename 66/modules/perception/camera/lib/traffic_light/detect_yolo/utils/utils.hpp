#pragma once

#include <cuda_runtime_api.h>

#include <chrono>
#include <numeric>
#include <string>
#include <vector>

#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/macro.hpp"

namespace century {
namespace perception {
namespace camera {

void ReadBinaryFromFile(const std::string& file, std::string* contents);

bool SupportsIntegratedZeroCopy(const int gpu_id);

float findPercentile(float percentile, std::vector<float> const& timings);

float findMedian(std::vector<float> const& timings);

struct PerformanceResult {
    float min{0.f};
    float max{0.f};
    float mean{0.f};
    float median{0.f};
    std::vector<float> percentiles;
};

PerformanceResult getPerformanceResult(std::vector<float> const& timings, std::vector<float> const& percentiles);

class TimerBase {
 public:
    virtual void start() {}
    virtual void stop() {}
    std::vector<float> milliseconds() const noexcept {
        return mMs;
    }
    void reset() noexcept {
        mMs.clear();
    }
    float totalMilliseconds() const noexcept {
        return std::accumulate(mMs.begin(), mMs.end(), 0.0F);
    }

 protected:
    std::vector<float> mMs;
};

class KLDEPLOY CpuTimer : public TimerBase {
 public:
    void start() override;
    void stop() override;

 private:
    std::chrono::time_point<std::chrono::high_resolution_clock> mStart, mStop;
};

class KLDEPLOY GpuTimer : public TimerBase {
 public:
    explicit GpuTimer(cudaStream_t stream);
    ~GpuTimer();
    void start() override;
    void stop() override;

 private:
    cudaEvent_t mStart, mStop;
    cudaStream_t mStream;
};

}
}
}
