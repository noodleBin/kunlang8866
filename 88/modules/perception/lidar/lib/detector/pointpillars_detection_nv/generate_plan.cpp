
#include <iostream>
#include <fstream>
#include <memory>
#include "NvInfer.h"
#include "NvOnnxParser.h"

class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING)
      std::cout << msg << std::endl;
  }
} gLogger;

bool buildEngine(const std::string& onnxPath, const std::string& enginePath, 
                const std::vector<std::string>& inputFormats) {


  auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(gLogger));
  if (!builder) return false;

  const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
  if (!network) return false;

  auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, gLogger));
  if (!parser || !parser->parseFromFile(onnxPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
    return false;

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) return false;

  config->setFlag(nvinfer1::BuilderFlag::kFP16);
  config->setProfilingVerbosity(nvinfer1::ProfilingVerbosity::kDETAILED);


  for (int i = 0; i < network->getNbInputs(); ++i) {
    auto input = network->getInput(i);
    if (inputFormats[i].find("fp16") != std::string::npos)
      input->setType(nvinfer1::DataType::kHALF);
    else if (inputFormats[i].find("int32") != std::string::npos)
      input->setType(nvinfer1::DataType::kINT32);
  }

  auto plan = builder->buildSerializedNetwork(*network, *config);
  if (!plan) return false;

  std::ofstream engineFile(enginePath, std::ios::binary);
  engineFile.write(static_cast<const char*>(plan->data()), plan->size());
  return true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <onnx_path> <engine_path> [input_formats...]\n";
    return -1;
  }

  std::vector<std::string> inputFormats;
  for (int i = 3; i < argc; ++i)
    inputFormats.emplace_back(argv[i]);

  if (inputFormats.empty())
    inputFormats = {"fp16:chw", "int32:chw", "int32:chw"};

  if (!buildEngine(argv[1], argv[2], inputFormats)) {
    std::cerr << "Failed to build TensorRT engine\n";
    return -1;
  }

  std::cout << "Successfully built TensorRT engine: " << argv[2] << "\n";
  return 0;
}
