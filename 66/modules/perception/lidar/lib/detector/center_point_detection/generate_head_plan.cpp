#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <iostream>
#include <fstream>
#include <memory>

class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        std::ofstream logfile("trt_conversion.log", std::ios::app);
        logfile << "[TRT] " << msg << std::endl;
    }
} gLogger;

void convertONNXToTRT(const std::string& onnxPath, 
                     const std::string& trtPath,
                     size_t workspaceSize = 4096) {
    std::cout << "Converting ONNX model to TensorRT engine..." << std::endl;
    std::cout << "ONNX Path: " << onnxPath << std::endl;
    std::cout << "TRT Path: " << trtPath << std::endl;
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(
        nvinfer1::createInferBuilder(gLogger));
    
    const auto explicitBatch = 1U << static_cast<uint32_t>(
        nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
        builder->createNetworkV2(explicitBatch));
    
    auto parser = std::unique_ptr<nvonnxparser::IParser>(
        nvonnxparser::createParser(*network, gLogger));
    parser->parseFromFile(onnxPath.c_str(), 
        static_cast<int>(nvinfer1::ILogger::Severity::kVERBOSE));

    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(
        builder->createBuilderConfig());
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 
                             workspaceSize << 20);
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    config->setProfilingVerbosity(nvinfer1::ProfilingVerbosity::kDETAILED);


    for(int i=0; i<network->getNbInputs(); ++i) {
      auto input = network->getInput(i);
      input->setType(nvinfer1::DataType::kHALF);
    }

    for(int i=0; i<network->getNbOutputs(); ++i) {
        auto output = network->getOutput(i);
        output->setType(nvinfer1::DataType::kHALF);
    }

    
    auto engine = std::unique_ptr<nvinfer1::IHostMemory>(
        builder->buildSerializedNetwork(*network, *config));
    
    std::ofstream planFile(trtPath, std::ios::binary);
    std::cout << trtPath << std::endl;
    planFile.write((const char*)engine->data(), engine->size());
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <onnx_path> <engine_path> \n";
    return -1;
  }
  // convertONNXToTRT("./modules/perception/production/data/perception/lidar/models/detection/center_point/pcdet_neck_head.onnx", 
  //             "./modules/perception/production/data/perception/lidar/models/detection/center_point/pcdet_neck_head_sim.plan");
  convertONNXToTRT(argv[1], argv[2]);

  std::cout << "Successfully built TensorRT engine: " << argv[2] << "\n";
  return 0;
}