#include "tensorrt.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include "NvInfer.h"
#include "NvInferRuntime.h"

namespace century {
namespace perception {
namespace lidar {
namespace centerpoint {
namespace TensorRT {

static class Logger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
      std::cerr << "[NVINFER LOG]: " << msg << std::endl;
    }
  }
} gLogger_;

static std::string format_shape(const nvinfer1::Dims& shape) {
  char buf[200] = {0};
  char* p = buf;
  for (int i = 0; i < shape.nbDims; ++i) {
    if (i + 1 < shape.nbDims)
      p += sprintf(p, "%d x ", shape.d[i]);  // NOLINT
    else
      p += sprintf(p, "%d", shape.d[i]);  // NOLINT
  }
  return buf;
}

static std::vector<uint8_t> load_file(const std::string& file) {
  std::ifstream in(file, std::ios::in | std::ios::binary);
  if (!in.is_open()) return {};

  in.seekg(0, std::ios::end);
  size_t length = in.tellg();

  std::vector<uint8_t> data;
  if (length > 0) {
    in.seekg(0, std::ios::beg);
    data.resize(length);

    in.read(reinterpret_cast<char*>(&data[0]), length);
  }
  in.close();
  return data;
}

static const char* data_type_string(nvinfer1::DataType dt) {
  switch (dt) {
    case nvinfer1::DataType::kFLOAT:
      return "Float32";
    case nvinfer1::DataType::kHALF:
      return "Float16";
    case nvinfer1::DataType::kINT32:
      return "Int32";
    // case nvinfer1::DataType::kUINT8: return "UInt8";
    case nvinfer1::DataType::kINT8:
      return "Int8";
    case nvinfer1::DataType::kBOOL:
      return "BOOL";
    default:
      return "Unknow";
  }
}

template <typename T>
void DestroyTensorRTObject(T*& object) {
  if (nullptr == object) {
    return;
  }
#if NV_TENSORRT_MAJOR >= 10
  delete object;
#else
  object->destroy();
#endif
  object = nullptr;
}

int GetNumBindingsCompat(nvinfer1::ICudaEngine* engine) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getNbIOTensors();
#else
  return engine->getNbBindings();
#endif
}

const char* GetBindingNameCompat(nvinfer1::ICudaEngine* engine, int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getIOTensorName(index);
#else
  return engine->getBindingName(index);
#endif
}

int GetBindingIndexCompat(nvinfer1::ICudaEngine* engine, const std::string& name) {
#if NV_TENSORRT_MAJOR >= 10
  for (int i = 0; i < engine->getNbIOTensors(); ++i) {
    if (name == engine->getIOTensorName(i)) {
      return i;
    }
  }
  return -1;
#else
  return engine->getBindingIndex(name.c_str());
#endif
}

nvinfer1::Dims GetBindingDimsCompat(nvinfer1::ICudaEngine* engine, int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getTensorShape(GetBindingNameCompat(engine, index));
#else
  return engine->getBindingDimensions(index);
#endif
}

nvinfer1::DataType GetBindingDataTypeCompat(nvinfer1::ICudaEngine* engine,
                                            int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getTensorDataType(GetBindingNameCompat(engine, index));
#else
  return engine->getBindingDataType(index);
#endif
}

bool IsInputCompat(nvinfer1::ICudaEngine* engine, int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getTensorIOMode(GetBindingNameCompat(engine, index)) ==
         nvinfer1::TensorIOMode::kINPUT;
#else
  return engine->bindingIsInput(index);
#endif
}

class EngineImplCenter : public Engine {
 public:
  nvinfer1::IExecutionContext* context_ = nullptr;
  nvinfer1::ICudaEngine* engine_ = nullptr;
  nvinfer1::IRuntime* runtime_ = nullptr;
  std::vector<std::string> binding_names_;

  virtual ~EngineImplCenter() {
    DestroyTensorRTObject(context_);
    DestroyTensorRTObject(engine_);
    DestroyTensorRTObject(runtime_);
  }

  bool load(const std::string& file) {
    auto data = load_file(file);
    if (data.empty()) {
      printf("Load engine %s failed.\n", file.c_str());
      return false;
    }

    runtime_ = nvinfer1::createInferRuntime(gLogger_);
    if (nullptr == runtime_) {
      printf("Failed to create runtime.\n");
      return false;
    }

    engine_ =
#if NV_TENSORRT_MAJOR >= 10
        runtime_->deserializeCudaEngine(data.data(), data.size());
#else
        runtime_->deserializeCudaEngine(data.data(), data.size(), 0);
#endif
    if (nullptr == engine_) {
      printf("Failed to deserial CUDAEngine.\n");
      return false;
    }

    context_ = engine_->createExecutionContext();
    if (nullptr == context_) {
      printf("Failed to create execution context.\n");
      return false;
    }
    binding_names_.clear();
    for (int i = 0; i < GetNumBindingsCompat(engine_); ++i) {
      binding_names_.emplace_back(GetBindingNameCompat(engine_, i));
    }
    return true;
  }

  int64_t getBindingNumel(const std::string& name) override {
    int index = GetBindingIndexCompat(engine_, name);
    nvinfer1::Dims d = GetBindingDimsCompat(engine_, index);
    return std::accumulate(d.d, d.d + d.nbDims, 1, std::multiplies<int64_t>());
  }

  std::vector<int64_t> getBindingDims(const std::string& name) override {
    int index = GetBindingIndexCompat(engine_, name);
    nvinfer1::Dims dims = GetBindingDimsCompat(engine_, index);
    std::vector<int64_t> output(dims.nbDims);
    std::transform(dims.d, dims.d + dims.nbDims, output.begin(), [](int32_t v) { return v; });
    return output;
  }

  bool forward(const std::initializer_list<void*>& buffers, void* stream = nullptr) override {
#if NV_TENSORRT_MAJOR >= 10
    int index = 0;
    for (void* buffer : buffers) {
      if (index >= static_cast<int>(binding_names_.size())) {
        return false;
      }
      if (!context_->setTensorAddress(binding_names_[index].c_str(), buffer)) {
        return false;
      }
      ++index;
    }
    return context_->enqueueV3(static_cast<cudaStream_t>(stream));
#else
    return context_->enqueueV2(buffers.begin(), (cudaStream_t)stream, nullptr);
#endif
  }

  void print() override {
    if (!context_) {
      printf("Infer print, nullptr.\n");
      return;
    }

    int numInput = 0;
    int numOutput = 0;
    for (int i = 0; i < GetNumBindingsCompat(engine_); ++i) {
      if (IsInputCompat(engine_, i)) {
        ++numInput;
      }
      else {
        ++numOutput;
      }
    }

    printf("Engine %p detail\n", this);
    printf("Inputs: %d\n", numInput);
    int input_index = 0;
    int output_index = 0;
    for (int ibinding = 0; ibinding < GetNumBindingsCompat(engine_); ++ibinding) {
      if (!IsInputCompat(engine_, ibinding)) {
        continue;
      }
      printf("\t%d.%s : \tshape {%s}, %s\n", input_index++,
             GetBindingNameCompat(engine_, ibinding),
             format_shape(GetBindingDimsCompat(engine_, ibinding)).c_str(),
             data_type_string(GetBindingDataTypeCompat(engine_, ibinding)));
    }

    printf("Outputs: %d\n", numOutput);
    for (int ibinding = 0; ibinding < GetNumBindingsCompat(engine_); ++ibinding) {
      if (IsInputCompat(engine_, ibinding)) {
        continue;
      }
      printf("\t%d.%s : \tshape {%s}, %s\n", output_index++,
             GetBindingNameCompat(engine_, ibinding),
             format_shape(GetBindingDimsCompat(engine_, ibinding)).c_str(),
             data_type_string(GetBindingDataTypeCompat(engine_, ibinding)));
    }
  }
};

std::shared_ptr<Engine> load_plan(const std::string& file) {
  std::shared_ptr<EngineImplCenter> impl(new EngineImplCenter());
  if (!impl->load(file)) impl.reset();
  return impl;
}

}  // namespace TensorRT
}  // namespace centerpoint
}  // namespace lidar
}  // namespace perception
}  // namespace century
