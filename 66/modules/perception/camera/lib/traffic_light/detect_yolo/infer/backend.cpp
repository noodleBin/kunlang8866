#include <algorithm>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>
#include <utility>
#include "NvOnnxParser.h"
#include <opencv2/opencv.hpp>
#include "cyber/common/log.h"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/core/core.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/infer/backend.hpp"
#include "modules/perception/camera/lib/traffic_light/detect_yolo/utils/utils.hpp"

namespace century {
namespace perception {
namespace camera {

class Int8MinMaxCalibrator : public nvinfer1::IInt8MinMaxCalibrator {
 public:
    Int8MinMaxCalibrator(int batch_size, int width, int height, const std::string& calib_dirs, const std::string& calib_table_name, const std::string& input_blob_name)
        : batch_size_(batch_size), width_(width), height_(height), calib_table_name_(calib_table_name), input_blob_name_(input_blob_name) {
        DIR* dir;
        struct dirent* ent;
        if (nullptr != (dir = opendir(calib_dirs.c_str()))) {
            while (nullptr != (ent = readdir(dir))) {
                const std::string filename = ent->d_name;
                if ("." == filename || ".." == filename) {
                    continue;
                }
                if (std::string::npos != filename.find(".bin")) {
                    img_paths_.emplace_back(calib_dirs + "/" + filename);
                }
            }
            closedir(dir);
        } else {
            AERROR << "[Calibrator] Could not open directory: " << calib_dirs;
        }
        AINFO << "[Calibrator] Found " << img_paths_.size() << " images for calibration.";
        input_count_ = 3 * width * height * batch_size;
        cudaMalloc(&device_input_, input_count_ * sizeof(float));
    }
    ~Int8MinMaxCalibrator() override {
        if (device_input_) {
            cudaFree(device_input_);
        }
    }
    int getBatchSize() const noexcept override {
        return batch_size_;
    }
    bool getBatch(void* bindings[], const char* names[], int nbBindings) noexcept override {
        if (cursor_ >= img_paths_.size()) {
            return false;
        }

        std::vector<float> input_data(input_count_);
        int current_batch = 0;
        while (current_batch < batch_size_ && cursor_ < img_paths_.size()) {
            std::ifstream file(img_paths_[cursor_++], std::ios::binary);
            if (file) {
                const size_t single_img_size = 3 * width_ * height_ * sizeof(float);
                file.read(reinterpret_cast<char*>(input_data.data() + current_batch * 3 * width_ * height_), single_img_size);
                current_batch++;
            }
        }
        if (0 == current_batch) {
            return false;
        }
        cudaMemcpy(device_input_, input_data.data(), input_count_ * sizeof(float), cudaMemcpyHostToDevice);
        bindings[0] = device_input_;
        AINFO << "[Calibrator] Calibrated batch " << (cursor_ / batch_size_) << " / " << (img_paths_.size() / batch_size_);
        return true;
    }
    const void* readCalibrationCache(size_t& length) noexcept override {
        calibration_cache_.clear();
        std::ifstream input(calib_table_name_, std::ios::binary);
        if (!input.is_open()) {
            AINFO << "[Calibrator] Calibration cache file not found: "
                  << calib_table_name_ << ". Starting recalibration.";
            length = 0;
            return nullptr;
        }
        input.seekg(0, std::ios::end);
        const size_t file_size = input.tellg();
        input.seekg(0, std::ios::beg);
        calibration_cache_.resize(file_size);
        input.read(reinterpret_cast<char*>(calibration_cache_.data()), file_size);
        if (input.good() || input.eof()) {
            AINFO << "[Calibrator] Loaded calibration cache from: "
                  << calib_table_name_ << " (Size: " << file_size << " bytes)";
            length = file_size;
            return calibration_cache_.data();
        }
        AERROR << "[Calibrator] Error reading cache file!";
        length = 0;
        return nullptr;
    }

    void writeCalibrationCache(const void* cache, size_t length) noexcept override {
        std::ofstream output(calib_table_name_, std::ios::binary);
        if (output.is_open()) {
            output.write(reinterpret_cast<const char*>(cache), length);
            output.close();
            AINFO << "[Calibrator] Saved calibration cache to: " << calib_table_name_;
        } else {
            AERROR << "[Calibrator] Failed to write calibration cache to: " << calib_table_name_;
        }
    }

 private:
    int batch_size_;
    int width_;
    int height_;
    int cursor_ = 0;
    size_t input_count_;
    std::string calib_table_name_;
    std::string input_blob_name_;
    std::vector<std::string> img_paths_;
    void* device_input_ = nullptr;
    std::vector<char> calibration_cache_;
};

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            AWARN << "[TRT Build] " << msg;
        }
    }
} gLogger;

inline bool exists(const std::string& name) {
    struct stat buffer;
    return (0 == stat(name.c_str(), &buffer));
}

bool BuildEngineFromOnnx(const std::string& onnx_file, const std::string& engine_file, int gpu_id,
        bool use_int8 = false, const std::string& calib_dir = "") {
    AINFO << "Building TensorRT Engine from ONNX: " << onnx_file << " ...";
    AINFO << "This may take a few minutes...";
    cudaSetDevice(gpu_id);
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(gLogger));
    if (!builder) {
        return false;
    }
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
    if (!network) {
        return false;
    }
    auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, gLogger));
    if (!parser) {
        return false;
    }
    if (!parser->parseFromFile(onnx_file.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        AERROR << "Failed to parse ONNX file.";
        return false;
    }
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    if (!config) {
        return false;
    }
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 31);
    if (builder->platformHasFastFp16()) {
        AINFO << "Enabling FP16 Mode.";
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    std::unique_ptr<Int8MinMaxCalibrator> calibrator = nullptr;
    if (use_int8 && builder->platformHasFastInt8()) {
        for (int i = 0; i < network->getNbLayers(); ++i) {
            auto layer = network->getLayer(i);
            std::string name = layer->getName();
            std::transform(name.begin(), name.end(), name.begin(), ::tolower);
            if (name.find("transpose") != std::string::npos ||
                name.find("reshape") != std::string::npos ||
                name.find("detect") != std::string::npos) {
                layer->setPrecision(nvinfer1::DataType::kHALF);
                layer->setOutputType(0, nvinfer1::DataType::kHALF);
                AINFO << " > (Keyword Match) Force Layer to FP16: " << layer->getName();
            }
        }
        if (calib_dir.empty()) {
            AERROR << "[Error] INT8 requested but calibration directory not provided!";
            return false;
        }
        AINFO << "Enabling INT8 Mode with calibration data from: " << calib_dir;
        config->setFlag(nvinfer1::BuilderFlag::kINT8);
        std::string table_name = engine_file + ".calib";
        calibrator = std::make_unique<Int8MinMaxCalibrator>(16, 640, 480, calib_dir, table_name, "images");
        config->setInt8Calibrator(calibrator.get());
    }
    auto plan = std::unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
    if (!plan) {
        AERROR << "Failed to build serialized network.";
        return false;
    }
    std::ofstream outfile(engine_file, std::ios::binary);
    outfile.write(reinterpret_cast<const char*>(plan->data()), plan->size());
    outfile.close();
    AINFO << "Engine saved to: " << engine_file;
    return true;
}

void DebugDumpModelInput(float* gpu_ptr, int batch_idx, int channels, int height, int width, const std::string& prefix) {
    int area = height * width;
    int vol = channels * area;
    std::vector<float> host_data(vol);
    cudaMemcpy(host_data.data(), gpu_ptr + batch_idx * vol, vol * sizeof(float), cudaMemcpyDeviceToHost);
    std::vector<cv::Mat> split_channels;
    for (int c = 0; c < channels; ++c) {
        split_channels.emplace_back(cv::Mat(height, width, CV_32FC1, host_data.data() + c * area));
    }
    cv::Mat merged;
    cv::merge(split_channels, merged);
    merged.convertTo(merged, CV_8UC3, 255.0);
    cv::cvtColor(merged, merged, cv::COLOR_RGB2BGR);
    std::string filename = prefix + "_batch" + std::to_string(batch_idx) + ".jpg";
    cv::imwrite(filename, merged);
    AINFO << "[DEBUG] Saved model input to: " << filename;
}

TrtBackend::TrtBackend(const std::string& trt_engine_file, const InferOption& infer_option) : option(infer_option) {
    cudaSetDevice(option.device_id);
    CUDA_CHECK(cudaStreamCreate(&stream));
    if (!exists(trt_engine_file)) {
        AINFO << "Engine file not found: " << trt_engine_file;
        std::string onnx_file = trt_engine_file;
        size_t lastindex = onnx_file.find_last_of(".");
        std::string rawname = onnx_file.substr(0, lastindex);
        onnx_file = rawname + ".onnx";
        AINFO << "Trying to build from ONNX: " << onnx_file;
        if (exists(onnx_file)) {
            bool use_int8 = (trt_engine_file.find("int8") != std::string::npos);
            std::string calib_dir = "/century/modules/perception/camera/lib/traffic_light/detect_yolo/calib_data";
            if (!BuildEngineFromOnnx(onnx_file, trt_engine_file, option.device_id, use_int8, calib_dir)) {
                throw std::runtime_error("Failed to build TensorRT engine from ONNX.");
            }
        } else {
             throw std::runtime_error("Neither Engine nor ONNX file found!");
        }
    }
    zero_copy_ = SupportsIntegratedZeroCopy(option.device_id);
    manager_ = std::make_unique<TRTManager>();
    std::string engine_buffer;
    ReadBinaryFromFile(trt_engine_file, &engine_buffer);
    manager_->initialize(engine_buffer.data(), engine_buffer.size());
    getTensorInfo();
    initialize();
    if (!dynamic) {
        captureCudaGraph();
    }
}

std::unique_ptr<TrtBackend> TrtBackend::clone() {
    auto clone_backend = std::make_unique<TrtBackend>();
    clone_backend->option = option;
    cudaSetDevice(option.device_id);
    CUDA_CHECK(cudaStreamCreate(&clone_backend->stream));
    clone_backend->zero_copy_ = zero_copy_;
    clone_backend->manager_ = manager_->clone();
    clone_backend->getTensorInfo();
    clone_backend->initialize();
    if (!clone_backend->dynamic) {
        clone_backend->captureCudaGraph();
    }
    return clone_backend;
}

TrtBackend::~TrtBackend() {
    std::vector<TensorInfo>().swap(tensor_infos);
    std::vector<AffineTransform>().swap(affine_transforms);
    if (!dynamic) {
        cuda_graph_.destroy();
    }
    CUDA_CHECK(cudaStreamDestroy(stream));
}

void TrtBackend::getTensorInfo() {
    std::vector<TensorInfo>().swap(tensor_infos);
    buffer_type_ = option.enable_managed_memory ? BufferType::Unified : (zero_copy_ ? BufferType::Mapped : BufferType::Discrete);
    auto num_tensors = manager_->getNbIOTensors();
    for (auto i = 0; i < num_tensors; ++i) {
        std::string name = std::string(manager_->getIOTensorName(i));
        auto shape = manager_->getTensorShape(name.c_str());
        auto dtype = manager_->getTensorDataType(name.c_str());
        bool input = (nvinfer1::TensorIOMode::kINPUT ==
                             manager_->getTensorIOMode(name.c_str()));
        if (input) {
            dynamic = std::any_of(shape.d, shape.d + shape.nbDims,
                                  [](int val) { return -1 == val; });
            if (dynamic) {
                shape = manager_->getProfileShape(name.c_str(), 0, nvinfer1::OptProfileSelector::kMIN);
                min_shape = make_int4(shape.d[0], shape.d[1], shape.d[2], shape.d[3]);
                shape = manager_->getProfileShape(name.c_str(), 0, nvinfer1::OptProfileSelector::kMAX);
            }
            max_shape = make_int4(shape.d[0], shape.d[1], shape.d[2], shape.d[3]);
        } else if (!input && dynamic) {
            shape.d[0] = max_shape.x;
        }
        tensor_infos.emplace_back(name, shape, dtype, input, input ? BufferType::Device : buffer_type_);
    }
}

void TrtBackend::initialize() {
    std::vector<AffineTransform>().swap(affine_transforms);
    inputs_buffer_ = BufferFactory::createBuffer(buffer_type_);
    infer_size_ = max_shape.y * max_shape.w * max_shape.z;
    if (!option.input_shape.empty()) {
        int h = option.input_shape[2];
        int w = option.input_shape[3];
        input_size_ = max_shape.y * h * w;
        affine_transforms.emplace_back(AffineTransform());
        affine_transforms.front().updateMatrix(
            w,
            h,
            max_shape.w,
            max_shape.z);
        inputs_buffer_->allocate(max_shape.x * input_size_);
    } else {
        affine_transforms.resize(max_shape.x, AffineTransform());
        if (!dynamic) {
            inputs_buffer_->allocate(max_shape.x * infer_size_);
        }
    }
}

void TrtBackend::captureCudaGraph() {
    {
        for (auto& tensor_info : tensor_infos) {
            manager_->setTensorAddress(tensor_info.name.c_str(), tensor_info.buffer->device());
        }
        if (!manager_->enqueueV3(stream)) {
            throw std::runtime_error("captureCudaGraph: EnqueueV3 failed before graph creation.");
        }
        CUDA_CHECK(cudaStreamSynchronize(stream));
    }
    auto calculate_input_size_and_device = [&](int idx, int input_width, int input_height) {
        size_t input_size = input_height * input_width * max_shape.y;
        void* input_device = static_cast<uint8_t*>(inputs_buffer_->device()) + idx * input_size;
        void* infer_device = static_cast<float*>(tensor_infos.front().buffer->device()) + idx * infer_size_;
        return std::make_pair(input_device, infer_device);
    };

    auto warp_affine_func = [&](bool multi, int input_width, int input_height) {
        if (multi) {
            cudaMutliWarpAffine(
                inputs_buffer_->device(),
                input_width,
                input_height,
                tensor_infos.front().buffer->device(),
                max_shape.w,
                max_shape.z,
                affine_transforms.front().matrix,
                option.config,
                max_shape.x,
                stream);
        } else {
            for (int idx = 0; idx < max_shape.x; ++idx) {
                std::pair<void*, void*> ptrs = calculate_input_size_and_device(idx, input_width, input_height);
                void* input_device = ptrs.first;
                void* infer_device = ptrs.second;
                cudaWarpAffine(
                    input_device,
                    input_width,
                    input_height,
                    infer_device,
                    max_shape.w,
                    max_shape.z,
                    affine_transforms[idx].matrix,
                    option.config,
                    stream);
            }
        }
    };
    cuda_graph_.beginCapture(stream);
    int input_width = !option.input_shape.empty() ? option.input_shape[3] : max_shape.w;
    int input_height = !option.input_shape.empty() ? option.input_shape[2] : max_shape.z;
    bool has_fixed_shape = !option.input_shape.empty();
    if (option.cuda_mem) {
        warp_affine_func(false, input_width, input_height);
    } else {
        inputs_buffer_->hostToDevice(stream);
        warp_affine_func(has_fixed_shape, input_width, input_height);
    }
    if (!manager_->enqueueV3(stream)) {
        throw std::runtime_error("captureCudaGraph: EnqueueV3 failed when graph creation.");
    }
    for (auto& tensor_info : tensor_infos) {
        if (!tensor_info.input) {
            tensor_info.buffer->deviceToHost(stream);
        }
    }
    cuda_graph_.endCapture(stream);
    if (!(has_fixed_shape && !option.cuda_mem)) {
        int num_nodes = max_shape.x + (option.cuda_mem ? 0 : 1);
        cuda_graph_.initializeNodes(num_nodes);
    }
}

void TrtBackend::staticInfer(const std::vector<Image>& inputs) {
    const auto num = inputs.size();
    if (num < 1 || num > static_cast<size_t>(max_shape.x)) {
        throw std::invalid_argument("Number of inputs out of range");
    }
    bool force_cpu_preprocess = false;
    if (force_cpu_preprocess) {
        float* const input_device_ptr =
            static_cast<float*>(tensor_infos.front().buffer->device());
        std::vector<float> host_nchw_buffer(num * 3 * max_shape.z * max_shape.w);

        for (size_t idx = 0; idx < num; ++idx) {
            cv::Mat raw_img(inputs[idx].height, inputs[idx].width, CV_8UC3,
                            const_cast<void*>(inputs[idx].data));
            affine_transforms[idx].updateMatrix(inputs[idx].width,
                                                inputs[idx].height, max_shape.w,
                                                max_shape.z);
            auto* m_ptr = reinterpret_cast<float*>(affine_transforms[idx].matrix);
            cv::Mat M(2, 3, CV_32F);
            M.at<float>(0, 0) = m_ptr[0];
            M.at<float>(0, 1) = m_ptr[1];
            M.at<float>(0, 2) = m_ptr[2];
            M.at<float>(1, 0) = m_ptr[3];
            M.at<float>(1, 1) = m_ptr[4];
            M.at<float>(1, 2) = m_ptr[5];
            cv::Mat resized_img;
            cv::warpAffine(raw_img, resized_img, M,
                           cv::Size(max_shape.w, max_shape.z),
                           cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
                           cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
            cv::cvtColor(resized_img, resized_img, cv::COLOR_BGR2RGB);
            cv::Mat float_img;
            resized_img.convertTo(float_img, CV_32FC3, 1.0 / 255.0f);
            std::vector<cv::Mat> split_channels;
            cv::split(float_img, split_channels);
            float* batch_ptr =
                host_nchw_buffer.data() + idx * 3 * max_shape.z * max_shape.w;
            const int area = max_shape.z * max_shape.w;
            std::memcpy(batch_ptr, split_channels[0].data, area * sizeof(float));
            std::memcpy(batch_ptr + area, split_channels[1].data,
                        area * sizeof(float));
            std::memcpy(batch_ptr + 2 * area, split_channels[2].data,
                        area * sizeof(float));
        }
        CUDA_CHECK(cudaMemcpyAsync(input_device_ptr, host_nchw_buffer.data(),
                                   host_nchw_buffer.size() * sizeof(float),
                                   cudaMemcpyHostToDevice, stream));
        if (!tensor_infos.empty()) {
            tensor_infos[0].shape = {static_cast<int>(num), 3, max_shape.z,
                                     max_shape.w};
        }
        manager_->enqueueV3(stream);
        for (size_t i = 1; i < tensor_infos.size(); ++i) {
            auto& info = tensor_infos[i];
            if (!info.input) {
                CUDA_CHECK(cudaMemcpyAsync(info.buffer->host(),
                                           info.buffer->device(), info.bytes_,
                                           cudaMemcpyDeviceToHost, stream));
            }
        }
        CUDA_CHECK(cudaStreamSynchronize(stream));
        return;
    }

    if (!option.input_shape.empty()) {
        if (option.cuda_mem) {
            for (size_t idx = 0; idx < num; ++idx) {
                auto infer_device_ptr =
                    static_cast<float*>(tensor_infos.front().buffer->device()) +
                    idx * infer_size_;
                void* kernel_params[] = {
                    (void*)&inputs[idx].data,
                    (void*)&inputs[idx].width,
                    (void*)&inputs[idx].height,
                    (void*)&infer_device_ptr,
                    (void*)&max_shape.w,
                    (void*)&max_shape.z,
                    (void*)&affine_transforms.front().matrix[0],
                    (void*)&affine_transforms.front().matrix[1],
                    (void*)&option.config};
                cuda_graph_.updateKernelNodeParams(idx, kernel_params);
            }
        } else {
            uint8_t* const host_buffer_base =
                static_cast<uint8_t*>(inputs_buffer_->host());
            for (size_t idx = 0; idx < num; ++idx) {
                const auto& img = inputs[idx];
                uint8_t* dst_ptr = host_buffer_base + idx * input_size_;
                const uint8_t* src_ptr = static_cast<const uint8_t*>(img.data);
                const int w = img.width;
                const int h = img.height;
                const int c = 3;
                const int src_step = w * c;
                const int dst_step = w * c;

                if (src_step == dst_step) {
                    std::memcpy(dst_ptr, src_ptr, h * dst_step);
                } else {
                    for (int row = 0; row < h; ++row) {
                        std::memcpy(dst_ptr + row * dst_step,
                                    src_ptr + row * src_step, dst_step);
                    }
                }
            }
        }

        cuda_graph_.launch(stream);
        manager_->enqueueV3(stream);
        CUDA_CHECK(cudaStreamSynchronize(stream));
        return;
    }
    if (!option.cuda_mem) {
        int total_size = 0;
        std::vector<int> input_sizes(num);
        for (size_t idx = 0; idx < num; ++idx) {
            input_sizes[idx] = inputs[idx].width * inputs[idx].height * max_shape.y;
            total_size += input_sizes[idx];
        }
        inputs_buffer_->allocate(total_size);
        uint8_t* input_ptr = static_cast<uint8_t*>(inputs_buffer_->host());
        for (size_t idx = 0; idx < num; ++idx) {
            std::memcpy(input_ptr, inputs[idx].data, input_sizes[idx]);
            input_ptr += input_sizes[idx];
        }
        if (BufferType::Discrete == buffer_type_) {
            cuda_graph_.updateMemcpyNodeParams(0, inputs_buffer_->host(),
                                               inputs_buffer_->device(),
                                               total_size);
        }
    }
    uint8_t* input_ptr =
        option.cuda_mem ? nullptr : static_cast<uint8_t*>(inputs_buffer_->device());
    for (size_t idx = 0; idx < num; ++idx) {
        affine_transforms[idx].updateMatrix(inputs[idx].width, inputs[idx].height,
                                            max_shape.w, max_shape.z);
        auto infer_device_ptr =
            static_cast<float*>(tensor_infos.front().buffer->device()) +
            idx * infer_size_;
        void* kernel_params[] = {
            option.cuda_mem ? (void*)&inputs[idx].data : (void*)&input_ptr,
            (void*)&inputs[idx].width,
            (void*)&inputs[idx].height,
            (void*)&infer_device_ptr,
            (void*)&max_shape.w,
            (void*)&max_shape.z,
            (void*)&affine_transforms[idx].matrix[0],
            (void*)&affine_transforms[idx].matrix[1],
            (void*)&option.config};
        int node_idx = (option.cuda_mem || buffer_type_ != BufferType::Discrete)
                           ? idx
                           : idx + 1;
        cuda_graph_.updateKernelNodeParams(node_idx, kernel_params);
        if (!option.cuda_mem) {
            input_ptr += inputs[idx].width * inputs[idx].height * max_shape.y;
        }
    }
    cuda_graph_.launch(stream);
    manager_->enqueueV3(stream);
    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void TrtBackend::dynamicInfer(const std::vector<Image>& inputs) {
    auto num = inputs.size();
    if (num < static_cast<size_t>(min_shape.x) || num > static_cast<size_t>(max_shape.x)) {
        throw std::invalid_argument("Number of inputs out of range");
    }
    for (auto& tensor_info : tensor_infos) {
        tensor_info.shape.d[0] = num;
        tensor_info.update();
        manager_->setTensorAddress(tensor_info.name.c_str(), tensor_info.buffer->device());
        if (tensor_info.input) {
            manager_->setInputShape(tensor_info.name.c_str(), tensor_info.shape);
        }
    }
    if (!option.input_shape.empty()) {
        if (!option.cuda_mem) {
            for (size_t idx = 0; idx < num; ++idx) {
                std::memcpy(static_cast<uint8_t*>(inputs_buffer_->host()) + idx * input_size_, inputs[idx].data, input_size_);
            }
            inputs_buffer_->hostToDevice(stream);
        }
        for (size_t idx = 0; idx < num; ++idx) {
            cudaWarpAffine(
                option.cuda_mem ? (void*)inputs[idx].data : static_cast<void*>(static_cast<uint8_t*>(inputs_buffer_->device()) + idx * input_size_),
                inputs[idx].width,
                inputs[idx].height,
                static_cast<float*>(tensor_infos.front().buffer->device()) + idx * infer_size_,
                max_shape.w,
                max_shape.z,
                affine_transforms.front().matrix,
                option.config,
                stream);
        }
    } else {
        int total_size = 0;
        std::vector<int> input_sizes(num);
        for (size_t idx = 0; idx < num; ++idx) {
            input_sizes[idx] = inputs[idx].width * inputs[idx].height * max_shape.y;
            total_size += input_sizes[idx];
            affine_transforms[idx].updateMatrix(inputs[idx].width, inputs[idx].height, max_shape.w, max_shape.z);
        }
        if (!option.cuda_mem) {
            inputs_buffer_->allocate(total_size);
            uint8_t* input_host = static_cast<uint8_t*>(inputs_buffer_->host());

            for (size_t idx = 0; idx < num; ++idx) {
                std::memcpy(input_host, inputs[idx].data, input_sizes[idx]);
                input_host += input_sizes[idx];
            }
            inputs_buffer_->hostToDevice(stream);
            uint8_t* input_device = static_cast<uint8_t*>(inputs_buffer_->device());
            for (size_t idx = 0; idx < num; ++idx) {
                cudaWarpAffine(
                    input_device,
                    inputs[idx].width,
                    inputs[idx].height,
                    static_cast<float*>(tensor_infos.front().buffer->device()) + idx * infer_size_,
                    max_shape.w,
                    max_shape.z,
                    affine_transforms[idx].matrix,
                    option.config,
                    stream);
                input_device += input_sizes[idx];
            }
        } else {

            for (size_t idx = 0; idx < num; ++idx) {
                cudaWarpAffine(
                    (void*)inputs[idx].data,
                    inputs[idx].width,
                    inputs[idx].height,
                    static_cast<float*>(tensor_infos.front().buffer->device()) + idx * infer_size_,
                    max_shape.w,
                    max_shape.z,
                    affine_transforms[idx].matrix,
                    option.config,
                    stream);
            }
        }
    }
    if (!manager_->enqueueV3(stream)) {
        throw std::runtime_error("Infer Error.");
    }
    for (auto& tensor_info : tensor_infos) {
        if (!tensor_info.input) {
            tensor_info.buffer->deviceToHost(stream);
        }
    }
    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void TrtBackend::infer(const std::vector<Image>& inputs) {
    if (dynamic) {
        dynamicInfer(inputs);
    } else {
        staticInfer(inputs);
    }
}
}
}
}
