/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved. SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "tensorrt.hpp"

#include <string.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>

struct CUstream_st;
using cudaStream_t = CUstream_st *;
struct CUevent_st;
using cudaEvent_t = CUevent_st *;

#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "check.hpp"

#include "cyber/common/log.h"

namespace TensorRT {

static class Logger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char *msg) noexcept override {
    if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
      std::cerr << "[NVINFER LOG]: " << msg << std::endl;
    }
  }
} gLogger_;

static std::string format_shape(const nvinfer1::Dims &shape) {
  char buf[200] = {0};
  char *p = buf;
  for (int i = 0; i < shape.nbDims; ++i) {
    if (i + 1 < shape.nbDims)
      p += sprintf(p, "%d x ", (int)shape.d[i]);
    else
      p += sprintf(p, "%d", (int)shape.d[i]);
  }
  return buf;
}

static std::vector<uint8_t> load_file(const std::string &file) {
  std::ifstream in(file, std::ios::in | std::ios::binary);
  if (!in.is_open()) return {};

  in.seekg(0, std::ios::end);
  size_t length = in.tellg();

  std::vector<uint8_t> data;
  if (length > 0) {
    in.seekg(0, std::ios::beg);
    data.resize(length);

    in.read((char *)&data[0], length);
  }
  in.close();
  return data;
}

static const char *data_type_string(nvinfer1::DataType dt) {
  switch (dt) {
    case nvinfer1::DataType::kFLOAT:
      return "float32";
    case nvinfer1::DataType::kHALF:
      return "float16";
    case nvinfer1::DataType::kINT8:
      return "int8";
    case nvinfer1::DataType::kINT32:
      return "int32";
    case nvinfer1::DataType::kBOOL:
      return "bool";
    case nvinfer1::DataType::kUINT8:
      return "uint8";

#if NV_TENSORRT_MAJOR >= 10
    case nvinfer1::DataType::kFP8:
      return "fp8";
    case nvinfer1::DataType::kBF16:
      return "bf16";
    case nvinfer1::DataType::kINT64:
      return "int64";
    case nvinfer1::DataType::kINT4:
      return "int4";
#endif

    default:
      return "Unknow";
  }
}

template <typename _T>
static void destroy_pointer(_T *ptr) {
  if (nullptr != ptr) {
    delete ptr;
    ptr = nullptr;
  }
}

class __native_engine_context {
 public:
  virtual ~__native_engine_context() { destroy(); }

  bool construct(const void *pdata, size_t size, const char *message_name) {
    destroy();

    if (nullptr == pdata || 0 == size) {
      printf("Construct for empty data found.\n");
      return false;
    }

    runtime_ = std::shared_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(gLogger_),
        destroy_pointer<nvinfer1::IRuntime>);
    if (nullptr == runtime_) {
      printf("Failed to create tensorRT runtime: %s.\n", message_name);
      return false;
    }

    engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(
#if NV_TENSORRT_MAJOR >= 10
        runtime_->deserializeCudaEngine(pdata, size),
#else
        runtime_->deserializeCudaEngine(pdata, size, nullptr),
#endif
        destroy_pointer<nvinfer1::ICudaEngine>);

    if (nullptr == engine_) {
      printf("Failed to deserialize engine: %s\n", message_name);
      return false;
    }

    context_ = std::shared_ptr<nvinfer1::IExecutionContext>(
        engine_->createExecutionContext(),
        destroy_pointer<nvinfer1::IExecutionContext>);
    if (nullptr == context_) {
      printf("Failed to create execution context: %s\n", message_name);
      return false;
    }
    return nullptr != context_;
  }

 private:
  void destroy() {
    context_.reset();
    engine_.reset();
    runtime_.reset();
  }

 public:
  std::shared_ptr<nvinfer1::IExecutionContext> context_;
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IRuntime> runtime_ = nullptr;
};

class EngineImplement : public Engine {
 public:
  std::shared_ptr<__native_engine_context> context_;
  std::unordered_map<std::string, int> binding_name_to_index_;
  std::vector<std::string> binding_names_;

  virtual ~EngineImplement() = default;

  bool construct(const void *data, size_t size, const char *message_name) {
    context_ = std::make_shared<__native_engine_context>();
    if (!context_->construct(data, size, message_name)) {
      return false;
    }

    setup();
    return true;
  }

  bool load(const std::string &file) {
    auto data = load_file(file);
    if (data.empty()) {
      printf(
          "An empty file has been loaded. Please confirm your file path: %s\n",
          file.c_str());
      return false;
    }
    return this->construct(data.data(), data.size(), file.c_str());
  }

  void setup() {
    auto engine = this->context_->engine_;
    int nbBindings = 0;
#if NV_TENSORRT_MAJOR >= 10
    nbBindings = engine->getNbIOTensors();
#else
    nbBindings = engine->getNbBindings();
#endif

    binding_names_.clear();
    binding_names_.reserve(nbBindings);
    binding_name_to_index_.clear();
    for (int i = 0; i < nbBindings; ++i) {
      const char *bindingName = nullptr;
#if NV_TENSORRT_MAJOR >= 10
      bindingName = engine->getIOTensorName(i);
#else
      bindingName = engine->getBindingName(i);
#endif
      binding_names_.emplace_back(bindingName);
      binding_name_to_index_[bindingName] = i;
    }
  }

  virtual int index(const std::string &name) override {
    auto iter = binding_name_to_index_.find(name);
    Assertf(iter != binding_name_to_index_.end(),
            "Can not found the binding name: %s", name.c_str());
    return iter->second;
  }

  virtual bool forward(const std::vector<const void *> &bindings, void *stream,
                       void *input_consum_event) override {
#if NV_TENSORRT_MAJOR >= 10
    auto context = this->context_->context_;
    if (bindings.size() != binding_names_.size()) {
      printf("Tensor count mismatch, expected %zu but got %zu\n",
             binding_names_.size(), bindings.size());
      return false;
    }
    for (size_t i = 0; i < bindings.size(); ++i) {
      if (!context->setTensorAddress(binding_names_[i].c_str(),
                                     const_cast<void *>(bindings[i]))) {
        printf("Failed to set tensor address for tensor %s\n",
               binding_names_[i].c_str());
        return false;
      }
    }
    (void)input_consum_event;
    return context->enqueueV3((cudaStream_t)stream);
#else
    return this->context_->context_->enqueueV2(
        (void **)bindings.data(), (cudaStream_t)stream,
        (cudaEvent_t *)input_consum_event);
#endif
  }

  virtual bool forward(
      const std::unordered_map<std::string, const void *> &bindings,
      void *stream, void *input_consum_event) override {
#if NV_TENSORRT_MAJOR >= 10
    auto engine = this->context_->engine_;
    auto context = this->context_->context_;
    int ibinding = 0;
    for (; ibinding < engine->getNbIOTensors(); ++ibinding) {
      auto tensor_name = engine->getIOTensorName(ibinding);
      auto binding_iter = bindings.find(tensor_name);
      if (binding_iter == bindings.end()) {
        printf(
            "Failed to set the tensor address, can not found tensor %s in "
            "bindings provided.",
            tensor_name);
        return false;
      }

      if (!context->setTensorAddress(tensor_name,
                                     (void *)binding_iter->second)) {
        printf("Failed to set tensor address for tensor %s\n", tensor_name);
        return false;
      }
    }
    return context->enqueueV3((cudaStream_t)stream);
#else
    std::vector<const void *> ordered_bindings(num_bindings(), nullptr);
    for (int i = 0; i < num_bindings(); ++i) {
      const auto binding_iter = bindings.find(binding_names_[i]);
      if (binding_iter == bindings.end()) {
        printf(
            "Failed to set the tensor address, can not found tensor %s in "
            "bindings provided.",
            binding_names_[i].c_str());
        return false;
      }
      ordered_bindings[i] = binding_iter->second;
    }
    return forward(ordered_bindings, stream, input_consum_event);
#endif
  }


  virtual std::vector<int> run_dims(const std::string &name) override {
    return run_dims(index(name));
  }

  virtual std::vector<int> run_dims(int ibinding) override {
    auto dim =
#if NV_TENSORRT_MAJOR >= 10
        this->context_->context_->getTensorShape(binding_names_[ibinding].c_str());
#else
        this->context_->context_->getBindingDimensions(ibinding);
#endif
    return std::vector<int>(dim.d, dim.d + dim.nbDims);
  }

  virtual std::vector<int> static_dims(const std::string &name) override {
    return static_dims(index(name));
  }

  virtual std::vector<int> static_dims(int ibinding) override {
    auto dim =
#if NV_TENSORRT_MAJOR >= 10
        this->context_->engine_->getTensorShape(binding_names_[ibinding].c_str());
#else
        this->context_->engine_->getBindingDimensions(ibinding);
#endif
    return std::vector<int>(dim.d, dim.d + dim.nbDims);
  }

  virtual int num_bindings() override {
    return static_cast<int>(binding_names_.size());
  }

  virtual bool is_input(int ibinding) override {
#if NV_TENSORRT_MAJOR >= 10
    return this->context_->engine_->getTensorIOMode(
               binding_names_[ibinding].c_str()) ==
           nvinfer1::TensorIOMode::kINPUT;
#else
    return this->context_->engine_->bindingIsInput(ibinding);
#endif
  }

  virtual bool is_input(const std::string &name) override {
#if NV_TENSORRT_MAJOR >= 10
    auto engine = this->context_->engine_;
    return engine->getTensorIOMode(name.c_str()) ==
           nvinfer1::TensorIOMode::kINPUT;
#else
    return is_input(index(name));
#endif
  }

  virtual bool set_run_dims(const std::string &name,
                            const std::vector<int> &dims) override {
    return this->set_run_dims(index(name), dims);
  }

  virtual bool set_run_dims(int ibinding,
                            const std::vector<int> &dims) override {
    nvinfer1::Dims d;
    memcpy(d.d, dims.data(), sizeof(int) * dims.size());
    d.nbDims = dims.size();
#if NV_TENSORRT_MAJOR >= 10
    return this->context_->context_->setInputShape(
        binding_names_[ibinding].c_str(), d);
#else
    return this->context_->context_->setBindingDimensions(ibinding, d);
#endif
  }

  virtual int numel(const std::string &name) override {
    return numel(index(name));
  }

  virtual int numel(int ibinding) override {
    auto dim =
#if NV_TENSORRT_MAJOR >= 10
        this->context_->context_->getTensorShape(binding_names_[ibinding].c_str());
#else
        this->context_->context_->getBindingDimensions(ibinding);
#endif
    return std::accumulate(dim.d, dim.d + dim.nbDims, 1,
                           std::multiplies<int>());
  }

  virtual DType dtype(const std::string &name) override {
    return dtype(index(name));
  }

  virtual DType dtype(int ibinding) override {
#if NV_TENSORRT_MAJOR >= 10
    return (DType)this->context_->engine_->getTensorDataType(
        binding_names_[ibinding].c_str());
#else
    return (DType)this->context_->engine_->getBindingDataType(ibinding);
#endif
  }

  virtual bool has_dynamic_dim() override {
    int numBindings = num_bindings();
    for (int i = 0; i < numBindings; ++i) {
      auto dims = static_dims(i);
      for (int dim : dims) {
        if (-1 == dim) {
          return true;
        }
          
      }
    }
    return false;
  }

  virtual void print(const char *name) override {
    printf("------------------------------------------------------\n");
    printf("%s 🌱 is %s model\n", name,
           has_dynamic_dim() ? "Dynamic Shape" : "Static Shape");

    int num_input = 0;
    int num_output = 0;
    for (int i = 0; i < num_bindings(); ++i) {
      if (is_input(i)) {
        num_input++;
      }
      else {
        num_output++;
      }
    }

    printf("Inputs: %d\n", num_input);
    for (int i = 0; i < num_bindings(); ++i) {
      if (!is_input(i)) {
        continue;
      }
      auto dim = static_dims(i);
      nvinfer1::Dims dims;
      dims.nbDims = dim.size();
      memcpy(dims.d, dim.data(), sizeof(int) * dim.size());
      printf("\t%d.%s : shape {%s}, %s\n", i, binding_names_[i].c_str(),
             format_shape(dims).c_str(),
             data_type_string((nvinfer1::DataType)dtype(i)));
    }

    printf("Outputs: %d\n", num_output);
    for (int i = 0; i < num_bindings(); ++i) {
      if (is_input(i)) {
        continue;
      }
      auto dim = static_dims(i);
      nvinfer1::Dims dims;
      dims.nbDims = dim.size();
      memcpy(dims.d, dim.data(), sizeof(int) * dim.size());
      printf("\t%d.%s : shape {%s}, %s\n", i, binding_names_[i].c_str(),
             format_shape(dims).c_str(),
             data_type_string((nvinfer1::DataType)dtype(i)));
    }
  }
};

std::shared_ptr<Engine> load(const std::string &file) {
  std::shared_ptr<EngineImplement> impl(new EngineImplement());
  if (!impl->load(file)) impl.reset();
  return impl;
}

};  // namespace TensorRT
