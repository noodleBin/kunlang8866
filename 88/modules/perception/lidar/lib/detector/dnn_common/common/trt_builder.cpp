// trt_builder.cpp
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

namespace {

using nvinfer1::BuilderFlag;
using nvinfer1::DataType;
using nvinfer1::Dims;
using nvinfer1::IBuilder;
using nvinfer1::IBuilderConfig;
using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;
using nvinfer1::IHostMemory;
using nvinfer1::ILogger;
using nvinfer1::INetworkDefinition;
using nvinfer1::IRuntime;
using nvinfer1::MemoryPoolType;
using nvinfer1::NetworkDefinitionCreationFlag;
using nvinfer1::ProfilingVerbosity;

class Logger : public ILogger {
 public:
  void SetSeverity(Severity severity) { reportable_severity_ = severity; }

  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= reportable_severity_) {
      std::cout << "[TRT] " << msg << std::endl;
    }
  }

 private:
  Severity reportable_severity_ = Severity::kINFO;
};

Logger g_logger;

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

int GetNumBindingsCompat(ICudaEngine* engine) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getNbIOTensors();
#else
  return engine->getNbBindings();
#endif
}

const char* GetBindingNameCompat(ICudaEngine* engine, int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getIOTensorName(index);
#else
  return engine->getBindingName(index);
#endif
}

bool IsInputCompat(ICudaEngine* engine, int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getTensorIOMode(GetBindingNameCompat(engine, index)) ==
         nvinfer1::TensorIOMode::kINPUT;
#else
  return engine->bindingIsInput(index);
#endif
}

DataType GetBindingDataTypeCompat(ICudaEngine* engine, int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getTensorDataType(GetBindingNameCompat(engine, index));
#else
  return engine->getBindingDataType(index);
#endif
}

Dims GetBindingDimensionsCompat(ICudaEngine* engine, int index) {
#if NV_TENSORRT_MAJOR >= 10
  return engine->getTensorShape(GetBindingNameCompat(engine, index));
#else
  return engine->getBindingDimensions(index);
#endif
}

struct BuildConfig {
  std::string onnx_path;
  std::string engine_path;
  std::string plugins_csv;
  std::string input_io_formats;
  std::string output_io_formats;
  std::string export_layer_json;
  std::string profile_json;
  std::string profiling_verbosity = "detailed";
  size_t workspace_mb = 4096;
  bool use_fp16 = false;
  bool use_int8 = false;
  bool dump_layer_info = false;
  bool dump_profile = false;
  bool separate_profile_run = false;
  bool verbose = false;
};

struct BindingInfo {
  std::string name;
  bool is_input;
  DataType dtype;
  Dims dims;
  size_t elem_size_bytes;
  size_t elem_count;
  size_t total_bytes;
};

std::vector<std::string> SplitString(const std::string& str, char delim) {
  std::vector<std::string> elements;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (!item.empty()) {
      elements.emplace_back(item);
    }
  }
  return elements;
}

size_t GetElementSize(DataType dtype) {
  switch (dtype) {
    case DataType::kFLOAT:
      return 4;
    case DataType::kHALF:
      return 2;
    case DataType::kINT8:
      return 1;
    case DataType::kINT32:
      return 4;
    default:
      return 4;
  }
}

size_t ComputeElementCount(const Dims& dims) {
  if (dims.nbDims <= 0) {
    return 0;
  }

  size_t count = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
    count *= static_cast<size_t>(dims.d[i]);
  }
  return count;
}

void LoadPluginLibraries(const std::string& plugins_csv) {
  if (plugins_csv.empty()) {
    return;
  }

  auto libraries = SplitString(plugins_csv, ',');
  for (const auto& lib : libraries) {
    void* handle = dlopen(lib.c_str(), RTLD_NOW);
    if (!handle) {
      std::cerr << "[ERROR] Failed to load plugin " << lib << ": " << dlerror()
                << std::endl;
      exit(1);
    }
    std::cout << "[PLUGIN] Loaded " << lib << std::endl;
  }
}

void ExportLayerInfo(INetworkDefinition* network, const std::string& path) {
  if (path.empty()) {
    return;
  }

  std::ofstream ofs(path);
  ofs << "{\n  \"layers\": [\n";

  for (int i = 0; i < network->getNbLayers(); ++i) {
    auto layer = network->getLayer(i);
    ofs << "    {\n";
    ofs << "      \"index\": " << i << ",\n";
    ofs << "      \"name\": \"" << (layer->getName() ? layer->getName() : "")
        << "\",\n";
    ofs << "      \"type\": " << static_cast<int>(layer->getType()) << ",\n";
    ofs << "      \"nbInputs\": " << layer->getNbInputs() << ",\n";
    ofs << "      \"nbOutputs\": " << layer->getNbOutputs() << "\n";
    ofs << "    }";
    if (i + 1 < network->getNbLayers()) ofs << ",";
    ofs << "\n";
  }

  ofs << "  ]\n}\n";
  std::cout << "[EXPORT] Layer info saved to " << path << std::endl;
}

ICudaEngine* LoadEngine(const std::string& engine_path, IRuntime* runtime) {
  std::ifstream ifs(engine_path, std::ios::binary | std::ios::ate);
  if (!ifs) {
    std::cerr << "[PROFILE] Cannot open engine file: " << engine_path
              << std::endl;
    return nullptr;
  }

  std::streamsize size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::vector<char> buffer(size);

  if (!ifs.read(buffer.data(), size)) {
    std::cerr << "[PROFILE] Failed to read engine file" << std::endl;
    return nullptr;
  }

  return
#if NV_TENSORRT_MAJOR >= 10
      runtime->deserializeCudaEngine(buffer.data(), size);
#else
      runtime->deserializeCudaEngine(buffer.data(), size, nullptr);
#endif
}

bool ExtractBindingInfo(ICudaEngine* engine,
                        std::vector<BindingInfo>* bindings) {
  int num_bindings = GetNumBindingsCompat(engine);
  bindings->reserve(num_bindings);

  for (int i = 0; i < num_bindings; ++i) {
    BindingInfo info;
    info.name = GetBindingNameCompat(engine, i);
    info.is_input = IsInputCompat(engine, i);
    info.dtype = GetBindingDataTypeCompat(engine, i);
    info.dims = GetBindingDimensionsCompat(engine, i);
    info.elem_size_bytes = GetElementSize(info.dtype);
    info.elem_count = ComputeElementCount(info.dims);

    if (0 == info.elem_count) {
      std::cerr << "[PROFILE] Binding '" << info.name
                << "' has dynamic/unknown dimensions. "
                << "Cannot run simple profile. Use trtexec instead."
                << std::endl;
      return false;
    }

    info.total_bytes = info.elem_count * info.elem_size_bytes;
    bindings->emplace_back(info);
  }

  return true;
}

bool AllocateDeviceBuffers(const std::vector<BindingInfo>& bindings,
                           std::vector<void*>* device_ptrs) {
  device_ptrs->resize(bindings.size(), nullptr);

  for (size_t i = 0; i < bindings.size(); ++i) {
    cudaError_t err = cudaMalloc(&(*device_ptrs)[i], bindings[i].total_bytes);
    if (err != cudaSuccess) {
      std::cerr << "[PROFILE] cudaMalloc failed for binding " << i << std::endl;
      return false;
    }
    cudaMemset((*device_ptrs)[i], 0, bindings[i].total_bytes);
  }

  return true;
}

void FreeDeviceBuffers(const std::vector<void*>& device_ptrs) {
  for (void* ptr : device_ptrs) {
    if (ptr) {
      cudaFree(ptr);
    }
  }
}

bool WarmupEngine(IExecutionContext* context, void** bindings, int iterations) {
  for (int i = 0; i < iterations; ++i) {
    if (!context->executeV2(bindings)) {
      std::cerr << "[PROFILE] Warmup execution failed" << std::endl;
      return false;
    }
  }
  return true;
}

bool RunTimedInference(IExecutionContext* context, void** bindings,
                       int iterations, double* avg_ms) {
  using Clock = std::chrono::high_resolution_clock;
  auto start = Clock::now();

  for (int i = 0; i < iterations; ++i) {
    if (!context->executeV2(bindings)) {
      std::cerr << "[PROFILE] Timed execution failed" << std::endl;
      return false;
    }
  }

  auto end = Clock::now();
  double total_ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  *avg_ms = total_ms / iterations;

  return true;
}

void ExportProfilingResults(const std::string& engine_path,
                            const std::vector<BindingInfo>& bindings,
                            int iterations, double avg_ms,
                            const std::string& output_path) {
  if (output_path.empty()) {
    return;
  }

  std::ofstream ofs(output_path);
  ofs << "{\n";
  ofs << "  \"engine\": \"" << engine_path << "\",\n";
  ofs << "  \"iterations\": " << iterations << ",\n";
  ofs << "  \"total_ms\": " << (avg_ms * iterations) << ",\n";
  ofs << "  \"avg_ms\": " << avg_ms << ",\n";
  ofs << "  \"bindings\": [\n";

  for (size_t i = 0; i < bindings.size(); ++i) {
    const auto& info = bindings[i];
    ofs << "    { \"name\": \"" << info.name << "\", "
        << "\"isInput\": " << (info.is_input ? "true" : "false") << ", "
        << "\"dtype\": " << static_cast<int>(info.dtype) << ", "
        << "\"bytes\": " << info.total_bytes << " }";
    if (i + 1 < bindings.size()) {
      ofs << ",";
    }
    ofs << "\n";
  }

  ofs << "  ]\n}\n";
  std::cout << "[PROFILE] Results saved to " << output_path << std::endl;
}

bool RunSimpleProfile(const std::string& engine_path,
                      const std::string& profile_json_path) {
  IRuntime* runtime = nvinfer1::createInferRuntime(g_logger);
  if (!runtime) {
    std::cerr << "[PROFILE] Failed to create runtime" << std::endl;
    return false;
  }

  ICudaEngine* engine = LoadEngine(engine_path, runtime);
  if (!engine) {
    DestroyTensorRTObject(runtime);
    return false;
  }

  std::vector<BindingInfo> bindings;
  if (!ExtractBindingInfo(engine, &bindings)) {
    DestroyTensorRTObject(engine);
    DestroyTensorRTObject(runtime);
    return false;
  }

  IExecutionContext* context = engine->createExecutionContext();
  if (!context) {
    std::cerr << "[PROFILE] Failed to create execution context" << std::endl;
    DestroyTensorRTObject(engine);
    DestroyTensorRTObject(runtime);
    return false;
  }

  std::vector<void*> device_ptrs;
  if (!AllocateDeviceBuffers(bindings, &device_ptrs)) {
    FreeDeviceBuffers(device_ptrs);
    DestroyTensorRTObject(context);
    DestroyTensorRTObject(engine);
    DestroyTensorRTObject(runtime);
    return false;
  }

  const int kWarmupIterations = 3;
  const int kTimedIterations = 20;

  if (!WarmupEngine(context, device_ptrs.data(), kWarmupIterations)) {
    FreeDeviceBuffers(device_ptrs);
    DestroyTensorRTObject(context);
    DestroyTensorRTObject(engine);
    DestroyTensorRTObject(runtime);
    return false;
  }

  double avg_ms = 0.0;
  if (!RunTimedInference(context, device_ptrs.data(), kTimedIterations,
                         &avg_ms)) {
    FreeDeviceBuffers(device_ptrs);
    DestroyTensorRTObject(context);
    DestroyTensorRTObject(engine);
    DestroyTensorRTObject(runtime);
    return false;
  }

  ExportProfilingResults(engine_path, bindings, kTimedIterations, avg_ms,
                         profile_json_path);

  FreeDeviceBuffers(device_ptrs);
  DestroyTensorRTObject(context);
  DestroyTensorRTObject(engine);
  DestroyTensorRTObject(runtime);

  return true;
}

void SetInputOutputFormats(INetworkDefinition* network,
                           const std::string& input_formats,
                           const std::string& output_formats) {
  std::vector<std::string> in_formats = input_formats.empty()
                                            ? std::vector<std::string>()
                                            : SplitString(input_formats, ',');
  std::vector<std::string> out_formats = output_formats.empty()
                                             ? std::vector<std::string>()
                                             : SplitString(output_formats, ',');

  int num_inputs = network->getNbInputs();
  int num_outputs = network->getNbOutputs();

  for (int i = 0; i < std::min(num_inputs, static_cast<int>(in_formats.size()));
       ++i) {
    auto input = network->getInput(i);
    const std::string& fmt = in_formats[i];

    if (0 == fmt.rfind("fp16", 0)) {
      input->setType(DataType::kHALF);
    } else if (0 == fmt.rfind("fp32", 0)) {
      input->setType(DataType::kFLOAT);
    } else if (0 == fmt.rfind("int8", 0)) {
      input->setType(DataType::kINT8);
    }

    std::cout << "[IOFMT] Input[" << i << "] set to " << fmt << std::endl;
  }

  for (int i = 0;
       i < std::min(num_outputs, static_cast<int>(out_formats.size())); ++i) {
    auto output = network->getOutput(i);
    const std::string& fmt = out_formats[i];

    if (0 == fmt.rfind("fp16", 0)) {
      output->setType(DataType::kHALF);
    } else if (0 == fmt.rfind("fp32", 0)) {
      output->setType(DataType::kFLOAT);
    } else if (0 == fmt.rfind("int8", 0)) {
      output->setType(DataType::kINT8);
    }

    std::cout << "[IOFMT] Output[" << i << "] set to " << fmt << std::endl;
  }
}

void ConfigureBuilder(IBuilderConfig* config, const BuildConfig& build_config) {
  config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE,
                             build_config.workspace_mb << 20);

  if (build_config.profiling_verbosity == "none") {
    config->setProfilingVerbosity(ProfilingVerbosity::kNONE);
  } else {
    config->setProfilingVerbosity(ProfilingVerbosity::kDETAILED);
  }

  if (build_config.use_fp16) {
    config->setFlag(BuilderFlag::kFP16);
    std::cout << "[INFO] FP16 enabled" << std::endl;
  }

  if (build_config.use_int8) {
    config->setFlag(BuilderFlag::kINT8);
    std::cout << "[INFO] INT8 enabled (no calibrator provided)" << std::endl;
  }
}

IHostMemory* BuildEngine(IBuilder* builder, INetworkDefinition* network,
                         const BuildConfig& build_config) {
  auto config = std::unique_ptr<IBuilderConfig>(builder->createBuilderConfig());

  ConfigureBuilder(config.get(), build_config);

  std::cout << "[BUILD] Building serialized network..." << std::endl;
  IHostMemory* engine = builder->buildSerializedNetwork(*network, *config);

  if (!engine) {
    std::cerr << "[ERROR] buildSerializedNetwork failed" << std::endl;
  }

  return engine;
}

bool SaveEngine(IHostMemory* engine, const std::string& output_path) {
  std::ofstream ofs(output_path, std::ios::binary);
  if (!ofs) {
    std::cerr << "[ERROR] Cannot open output file: " << output_path
              << std::endl;
    return false;
  }

  ofs.write(reinterpret_cast<const char*>(engine->data()), engine->size());
  std::cout << "[SAVED] Engine saved to " << output_path << " ("
            << engine->size() << " bytes)" << std::endl;

  return true;
}

bool ParseArguments(int argc, char** argv, BuildConfig* config) {
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);

    if (0 == arg.rfind("--onnx=", 0)) {
      config->onnx_path = arg.substr(7);
    } else if (0 == arg.rfind("--saveEngine=", 0)) {
      config->engine_path = arg.substr(13);
    } else if (0 == arg.rfind("--plugins=", 0)) {
      config->plugins_csv = arg.substr(10);
    } else if ("--fp16" == arg) {
      config->use_fp16 = true;
    } else if ("--int8" == arg) {
      config->use_int8 = true;
    } else if ("--dumpLayerInfo" == arg) {
      config->dump_layer_info = true;
    } else if (0 == arg.rfind("--exportLayerInfo=", 0)) {
      config->export_layer_json = arg.substr(18);
    } else if ("--dumpProfile" == arg) {
      config->dump_profile = true;
    } else if ("--separateProfileRun" == arg) {
      config->separate_profile_run = true;
    } else if (0 == arg.rfind("--memPoolSize=workspace:", 0)) {
      std::string value = arg.substr(24);
      value.erase(0, value.find_first_not_of(" \t"));
      value.erase(value.find_last_not_of(" \t") + 1);

      if (value.empty() ||
          !std::all_of(value.begin(), value.end(), ::isdigit)) {
        std::cerr << "[ERROR] Invalid memPoolSize value: " << value
                  << std::endl;
        return false;
      }

      config->workspace_mb = std::stoul(value);
      std::cout << "[INFO] Workspace size = " << config->workspace_mb << " MB"
                << std::endl;
    } else if (0 == arg.rfind("--inputIOFormats=", 0)) {
      config->input_io_formats = arg.substr(17);
    } else if (0 == arg.rfind("--outputIOFormats=", 0)) {
      config->output_io_formats = arg.substr(18);
    } else if (0 == arg.rfind("--profilingVerbosity=", 0)) {
      config->profiling_verbosity = arg.substr(21);
    } else if ("--verbose" == arg) {
      config->verbose = true;
    } else if (0 == arg.rfind("--exportProfileJson=", 0)) {
      config->profile_json = arg.substr(20);
    }
  }

  return true;
}

void PrintUsage() {
  std::cerr
      << "Usage: trt_builder --onnx=<path> --saveEngine=<path> [options]\n"
      << "\nRequired:\n"
      << "  --onnx=<path>              Path to ONNX model file\n"
      << "  --saveEngine=<path>        Path to save TensorRT engine\n"
      << "\nOptional:\n"
      << "  --fp16                     Enable FP16 precision\n"
      << "  --int8                     Enable INT8 precision\n"
      << "  --plugins=<libs>           Comma-separated plugin libraries\n"
      << "  --memPoolSize=workspace:<MB>  Workspace size in MB (default: "
         "4096)\n"
      << "  --inputIOFormats=<fmt>     Input formats (e.g., fp16:chw)\n"
      << "  --outputIOFormats=<fmt>    Output formats (e.g., fp16:chw)\n"
      << "  --dumpLayerInfo            Export layer information\n"
      << "  --exportLayerInfo=<path>   Export layer info to JSON\n"
      << "  --dumpProfile              Enable profiling\n"
      << "  --separateProfileRun       Run separate profiling pass\n"
      << "  --exportProfileJson=<path> Export profile results to JSON\n"
      << "  --profilingVerbosity=<lvl> Profiling verbosity (none/detailed)\n"
      << "  --verbose                  Enable verbose logging\n";
}

}  // namespace

int main(int argc, char** argv) {
  BuildConfig config;

  if (!ParseArguments(argc, argv, &config)) {
    return 1;
  }

  if (config.onnx_path.empty() || config.engine_path.empty()) {
    PrintUsage();
    return 1;
  }

  if (config.verbose) {
    g_logger.SetSeverity(ILogger::Severity::kVERBOSE);
  }

  LoadPluginLibraries(config.plugins_csv);

  auto builder =
      std::unique_ptr<IBuilder>(nvinfer1::createInferBuilder(g_logger));
  if (!builder) {
    std::cerr << "[ERROR] Failed to create builder" << std::endl;
    return 1;
  }

  const auto explicit_batch =
      1U << static_cast<uint32_t>(
          NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<INetworkDefinition>(
      builder->createNetworkV2(explicit_batch));
  if (!network) {
    std::cerr << "[ERROR] Failed to create network" << std::endl;
    return 1;
  }

  auto parser = std::unique_ptr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, g_logger));
  if (!parser) {
    std::cerr << "[ERROR] Failed to create parser" << std::endl;
    return 1;
  }

  std::cout << "[INFO] Parsing " << config.onnx_path << std::endl;
  if (!parser->parseFromFile(config.onnx_path.c_str(),
                             static_cast<int>(ILogger::Severity::kVERBOSE))) {
    std::cerr << "[ERROR] Failed to parse ONNX file" << std::endl;
    return 1;
  }

  SetInputOutputFormats(network.get(), config.input_io_formats,
                        config.output_io_formats);

  if (config.dump_layer_info || !config.export_layer_json.empty()) {
    std::string layer_path = config.export_layer_json.empty()
                                 ? "layer_info.json"
                                 : config.export_layer_json;
    ExportLayerInfo(network.get(), layer_path);
  }

  auto engine = std::unique_ptr<IHostMemory>(
      BuildEngine(builder.get(), network.get(), config));
  if (!engine) {
    return 1;
  }

  if (!SaveEngine(engine.get(), config.engine_path)) {
    return 1;
  }

  if (config.dump_profile && config.separate_profile_run) {
    std::cout << "[PROFILE] Running separate profiling pass..." << std::endl;
    std::string profile_path =
        config.profile_json.empty() ? "profile.json" : config.profile_json;

    if (!RunSimpleProfile(config.engine_path, profile_path)) {
      std::cerr << "[PROFILE] Profiling failed or skipped" << std::endl;
    }
  } else if (config.dump_profile && !config.separate_profile_run) {
    std::cout << "[INFO] Profiling requested but --separateProfileRun not set. "
              << "Skipping profiling." << std::endl;
  }

  std::cout << "[DONE] Build completed successfully" << std::endl;
  return 0;
}
