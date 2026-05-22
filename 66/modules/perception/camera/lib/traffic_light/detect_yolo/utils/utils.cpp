#include <algorithm>
#include <fstream>
#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/macro.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/utils/utils.hpp"
#include <opencv2/opencv.hpp>

namespace century {
namespace perception {
namespace camera {

void ReadBinaryFromFile(const std::string& file, std::string* contents) {
    std::ifstream fin(file, std::ios::in | std::ios::binary);
    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open file: " + file + " to read.");
    }
    fin.seekg(0, std::ios::end);
    contents->clear();
    contents->resize(fin.tellg());
    fin.seekg(0, std::ios::beg);
    fin.read(&(contents->at(0)), contents->size());
    fin.close();
}

bool SupportsIntegratedZeroCopy(const int gpu_id) {
    cudaDeviceProp cuprops;
    CUDA_CHECK(cudaGetDeviceProperties(&cuprops, gpu_id));
    return cuprops.integrated && cuprops.canMapHostMemory;
}

float findPercentile(float percentile, std::vector<float> const& timings) {
    int32_t const all = static_cast<int32_t>(timings.size());
    int32_t const exclude = static_cast<int32_t>((1 - percentile / 100) * all);
    if (timings.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    if (percentile < 0.F || percentile > 100.F) {
        throw std::runtime_error("percentile is not in [0, 100]!");
    }
    return timings[std::max(all - 1 - exclude, 0)];
}

float findMedian(std::vector<float> const& timings) {
    if (timings.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    int32_t const m = timings.size() / 2;
    if (timings.size() % 2) {
        return timings[m];
    }
    return (timings[m - 1] + timings[m]) / 2;
}

PerformanceResult getPerformanceResult(std::vector<float> const& timings, std::vector<float> const& percentiles) {
    auto metricComparator = [](float const& a, float const& b) { return a < b; };
    auto metricAccumulator = [](float acc, float const& a) { return acc + a; };
    std::vector<float> newTimings = timings;
    std::sort(newTimings.begin(), newTimings.end(), metricComparator);
    PerformanceResult result;
    result.min = newTimings.front();
    result.max = newTimings.back();
    result.mean = std::accumulate(newTimings.begin(), newTimings.end(), 0.0F, metricAccumulator) / newTimings.size();
    result.median = findMedian(newTimings);
    for (auto percentile : percentiles) {
        result.percentiles.emplace_back(findPercentile(percentile, newTimings));
    }
    return result;
}

void CpuTimer::start() {
    mStart = std::chrono::high_resolution_clock::now();
}

void CpuTimer::stop() {
    mStop = std::chrono::high_resolution_clock::now();
    mMs.emplace_back(std::chrono::duration<float, std::milli>{mStop - mStart}.count());
}

GpuTimer::GpuTimer(cudaStream_t stream) : mStream(stream) {
    CUDA_CHECK(cudaEventCreate(&mStart));
    CUDA_CHECK(cudaEventCreate(&mStop));
}

GpuTimer::~GpuTimer() {
    CUDA_CHECK(cudaEventDestroy(mStart));
    CUDA_CHECK(cudaEventDestroy(mStop));
}

void GpuTimer::start() {
    CUDA_CHECK(cudaEventRecord(mStart, mStream));
}

void GpuTimer::stop() {
    CUDA_CHECK(cudaEventRecord(mStop, mStream));
    CUDA_CHECK(cudaEventSynchronize(mStop));
    float ms{0.0F};
    CUDA_CHECK(cudaEventElapsedTime(&ms, mStart, mStop));
    mMs.emplace_back(ms);
}

}
}
}
