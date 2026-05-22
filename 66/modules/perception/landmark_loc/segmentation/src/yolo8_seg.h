// Created by xiaxinrong on 2025/8/15.
#pragma once


#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "glog/logging.h"

// ============================================================================
// Debug/Timer Utilities (Optional)
// ============================================================================
#ifdef DEBUG
  #define DEBUG_PRINT(msg) LOG(INFO) << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG_PRINT(msg) /* no-op */
#endif

static inline int64_t get_thread_cpu_time_ms() {
 struct timespec ts;
 clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
 return (ts.tv_sec * 1000000000LL + ts.tv_nsec)/1e6;
}

// Simple scoped timer (optional)
class ScopedTimer {
public:
  explicit ScopedTimer(const std::string &name_)
    : name(name_), start(std::chrono::high_resolution_clock::now()) {}
  ~ScopedTimer() {
#ifdef DEBUG
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    LOG(INFO) << "[TIMER] " << name << ": " << ms << " ms" << std::endl;
#endif
  }
private:
  std::string name;
  std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

// ============================================================================
// Constants / Thresholds
// ============================================================================
static const float CONFIDENCE_THRESHOLD = 0.7f; // Filter boxes below this confidence
static const float IOU_THRESHOLD    = 0.7f; // NMS IoU threshold
static const float MASK_THRESHOLD    = 0.40f; // Slightly lower to capture partial objects

// ============================================================================
// Structs
// ============================================================================
struct BoundingBox {
  int x{0};
  int y{0};
  int width{0};
  int height{0};

  BoundingBox() = default;
  BoundingBox(int _x, int _y, int w, int h)
    : x(_x), y(_y), width(w), height(h) {}

  float area() const { return static_cast<float>(width * height); }

  BoundingBox intersect(const BoundingBox &other) const {
    int xStart = std::max(x, other.x);
    int yStart = std::max(y, other.y);
    int xEnd  = std::min(x + width, other.x + other.width);
    int yEnd  = std::min(y + height, other.y + other.height);
    int iw   = std::max(0, xEnd - xStart);
    int ih   = std::max(0, yEnd - yStart);
    return BoundingBox(xStart, yStart, iw, ih);
  }
};

struct Segmentation {
  BoundingBox box;
  float    conf{0.f};
  int     classId{0};
  cv::Mat   mask; // Single-channel (8UC1) mask in full resolution
};

// ============================================================================
// Utility Namespace
// ============================================================================
namespace utils {

  template <typename T>
  T clamp(const T &val, const T &low, const T &high) {
    return std::max(low, std::min(val, high));
  }

  inline std::vector<std::string> getClassNames(const std::string &path) {
    std::vector<std::string> classNames;
    std::ifstream f(path);
    if (!f) {
      std::cerr << "[ERROR] Could not open class names file: " << path << std::endl;
      return classNames;
    }
    std::string line;
    while (std::getline(f, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      classNames.push_back(line);
    }
    DEBUG_PRINT("Loaded " << classNames.size() << " class names from " << path);
    return classNames;
  }

  inline size_t vectorProduct(const std::vector<int64_t> &shape) {
    return std::accumulate(shape.begin(), shape.end(), 1ull, std::multiplies<size_t>());
  }

  inline void letterBox(const cv::Mat &image,
             cv::Mat &outImage,
             const cv::Size &newShape,
             const cv::Scalar &color   = cv::Scalar(114, 114, 114),
             bool auto_    = true,
             bool scaleFill  = false,
             bool scaleUp   = true,
             int stride    = 32) {
    float r = std::min((float)newShape.height / (float)image.rows,
              (float)newShape.width / (float)image.cols);
    if (!scaleUp) {
      r = std::min(r, 1.0f);
    }

    int newW = static_cast<int>(std::round(image.cols * r));
    int newH = static_cast<int>(std::round(image.rows * r));

    int dw = newShape.width - newW;
    int dh = newShape.height - newH;

    if (auto_) {
      dw = dw % stride;
      dh = dh % stride;
    }
    else if (scaleFill) {
      newW = newShape.width;
      newH = newShape.height;
      dw = 0;
      dh = 0;
    }

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(newW, newH), 0, 0, cv::INTER_LINEAR);

    int top = dh / 2;
    int bottom = dh - top;
    int left = dw / 2;
    int right = dw - left;
    cv::copyMakeBorder(resized, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
  }

  inline void sigmoid_inplace(const cv::Mat &src, cv::Mat &dst) {
   CV_Assert(src.type() == CV_32F);
   dst.create(src.size(), CV_32F);
   const int total = src.rows * src.cols;
   const float* s = reinterpret_cast<const float*>(src.data);
   float* d = reinterpret_cast<float*>(dst.data);
 
   for (int i = 0; i < total; ++i) {
     d[i] = 1.0f / (1.0f + std::exp(-s[i]));
   }
 }
  inline BoundingBox scaleCoords(const cv::Size &letterboxShape,
                  const BoundingBox &coords,
                  const cv::Size &originalShape,
                  bool p_Clip = true) {
    float gain = std::min((float)letterboxShape.height / (float)originalShape.height,
               (float)letterboxShape.width / (float)originalShape.width);

    int padW = static_cast<int>(std::round(((float)letterboxShape.width - (float)originalShape.width * gain) / 2.f));
    int padH = static_cast<int>(std::round(((float)letterboxShape.height - (float)originalShape.height * gain) / 2.f));

    BoundingBox ret;
    ret.x   = static_cast<int>(std::round(((float)coords.x   - (float)padW) / gain));
    ret.y   = static_cast<int>(std::round(((float)coords.y   - (float)padH) / gain));
    ret.width = static_cast<int>(std::round((float)coords.width  / gain));
    ret.height = static_cast<int>(std::round((float)coords.height / gain));

    if (p_Clip) {
      ret.x = clamp(ret.x, 0, originalShape.width);
      ret.y = clamp(ret.y, 0, originalShape.height);
      ret.width = clamp(ret.width, 0, originalShape.width - ret.x);
      ret.height = clamp(ret.height, 0, originalShape.height - ret.y);
    }

    return ret;
  }

  inline std::vector<cv::Scalar> generateColors(const std::vector<std::string> &classNames, int seed = 42) {
    static std::unordered_map<size_t, std::vector<cv::Scalar>> cache;
    size_t key = 0;
    for (const auto &name : classNames) {
      size_t h = std::hash<std::string>{}(name);
      key ^= (h + 0x9e3779b9 + (key << 6) + (key >> 2));
    }
    auto it = cache.find(key);
    if (it != cache.end()) {
      return it->second;
    }
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, 255);
    std::vector<cv::Scalar> colors;
    colors.reserve(classNames.size());
    for (size_t i = 0; i < classNames.size(); ++i) {
      colors.emplace_back(cv::Scalar(dist(rng), dist(rng), dist(rng)));
    }
    cache[key] = colors;
    return colors;
  }



  cv::Mat sigmoid(const cv::Mat& src) {
    cv::Mat dst;
    cv::exp(-src, dst);
    dst = 1.0 / (1.0 + dst);
    return dst;
  }
    inline void NMSBoxes(const std::vector<BoundingBox> &boxes,
             const std::vector<float> &scores,
             float scoreThreshold,
             float nmsThreshold,
             std::vector<int> &indices) {
    indices.clear();
    if (boxes.empty()) {
      return;
    }

    std::vector<int> order;
    order.reserve(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i) {
      if (scores[i] >= scoreThreshold) {
        order.push_back((int)i);
      }
    }
    if (order.empty()) return;

    std::sort(order.begin(), order.end(),
         [&scores](int a, int b) {
           return scores[a] > scores[b];
         });

    std::vector<float> areas(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i) {
      areas[i] = (float)(boxes[i].width * boxes[i].height);
    }

    std::vector<bool> suppressed(boxes.size(), false);
    for (size_t i = 0; i < order.size(); ++i) {
      int idx = order[i];
      if (suppressed[idx]) continue;

      indices.push_back(idx);

      for (size_t j = i + 1; j < order.size(); ++j) {
        int idx2 = order[j];
        if (suppressed[idx2]) continue;

        const BoundingBox &a = boxes[idx];
        const BoundingBox &b = boxes[idx2];
        int interX1 = std::max(a.x, b.x);
        int interY1 = std::max(a.y, b.y);
        int interX2 = std::min(a.x + a.width, b.x + b.width);
        int interY2 = std::min(a.y + a.height, b.y + b.height);

        int w = interX2 - interX1;
        int h = interY2 - interY1;
        if (w > 0 && h > 0) {
          float interArea = (float)(w * h);
          float unionArea = areas[idx] + areas[idx2] - interArea;
          float iou = (unionArea > 0.f)? (interArea / unionArea) : 0.f;
          if (iou > nmsThreshold) {
            suppressed[idx2] = true;
          }
        }
      }
    }
  }

} // namespace utils

// ============================================================================
// YOLOv8SegDetector Class
// ============================================================================
class YOLOv8SegDetector {
public:
  YOLOv8SegDetector(const std::string &modelPath,
           const std::string &labelsPath,
           bool useGPU = false, bool speed_up = true);

  // Main API
  std::vector<Segmentation> segment(const cv::Mat &image,
                   float confThreshold = CONFIDENCE_THRESHOLD,
                   float iouThreshold = IOU_THRESHOLD);

  // Accessors
  const std::vector<std::string> &getClassNames() const { return classNames; }
  const std::vector<cv::Scalar> &getClassColors() const { return classColors; }

private:
  Ort::Env      env;
  Ort::SessionOptions sessionOptions;
  Ort::Session    session{nullptr};

  bool   isDynamicInputShape{false};
  cv::Size inputImageShape; 

  std::vector<Ort::AllocatedStringPtr> inputNameAllocs;
  std::vector<const char*>       inputNames;
  std::vector<Ort::AllocatedStringPtr> outputNameAllocs;
  std::vector<const char*>       outputNames;

  size_t numInputNodes = 0;
  size_t numOutputNodes = 0;

  std::vector<std::string> classNames;
  std::vector<cv::Scalar> classColors;

  // Helpers
  void preprocess(const cv::Mat &image,
          float *&blobPtr,
          std::vector<int64_t> &inputTensorShape);
          /*
  cv::Mat preprocess(const cv::Mat &image,
            float *&blobPtr,
            std::vector<int64_t> &inputTensorShape);*/

  std::vector<Segmentation> postprocess(const cv::Size &origSize,
                     const cv::Size &letterboxSize,
                     const std::vector<Ort::Value> &outputs,
                     float confThreshold,
                     float iouThreshold);
};

inline YOLOv8SegDetector::YOLOv8SegDetector(const std::string &modelPath,
                      const std::string &labelsPath,
                      bool useGPU, bool speed_up)
  : env(ORT_LOGGING_LEVEL_WARNING, "YOLOv8Seg") 
{
  ScopedTimer timer("YOLOv8SegDetector Constructor");
  if(speed_up) {
   cv::setNumThreads(1);
  }
  
  sessionOptions.SetIntraOpNumThreads(std::min(6, static_cast<int>(std::thread::hardware_concurrency())));
  sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  std::vector<std::string> providers = Ort::GetAvailableProviders();
  if (useGPU && std::find(providers.begin(), providers.end(), "CUDAExecutionProvider") != providers.end()) {
    if(!speed_up) {
     OrtCUDAProviderOptions cudaOptions;
     sessionOptions.AppendExecutionProvider_CUDA(cudaOptions);
     LOG(INFO) << "[INFO] Using ONNX for YOLOv8 Seg inference.\n";
    } else {
      // ===== TensorRT EP with engine cache =====
      OrtTensorRTProviderOptionsV2* trtOptions = nullptr;
    
      // Create
      Ort::ThrowOnError(
          Ort::GetApi().CreateTensorRTProviderOptions(&trtOptions));
    
      std::vector<const char*> keys{
          "trt_engine_cache_enable",
          "trt_engine_cache_path",
          "trt_fp16_enable",
          "trt_max_workspace_size",
          "trt_force_sequential_engine_build"
      };
    
      std::vector<const char*> values{
          "1",
          "/century/data/trt_cache",
          "1",
          "1073741824",
          "1"
      };
    
      // Update options
      Ort::ThrowOnError(
          Ort::GetApi().UpdateTensorRTProviderOptions(
              trtOptions,
              keys.data(),
              values.data(),
              keys.size()));
    
      // Append EP
      Ort::ThrowOnError(
          Ort::GetApi().SessionOptionsAppendExecutionProvider_TensorRT_V2(
              sessionOptions,
              trtOptions));
    
      // ✅ Release：不要 ThrowOnError
      Ort::GetApi().ReleaseTensorRTProviderOptions(trtOptions);
    
      LOG(INFO) << "[INFO] Using TRT with engine cache for YOLOv8 Seg inference.\n";
    }
    
  } else {
    LOG(INFO) << "[INFO] Using CPU for YOLOv8 Seg inference.\n";
  }

#ifdef _WIN32
  std::wstring w_modelPath(modelPath.begin(), modelPath.end());
  session = Ort::Session(env, w_modelPath.c_str(), sessionOptions);
#else
  session = Ort::Session(env, modelPath.c_str(), sessionOptions);
#endif

  numInputNodes = session.GetInputCount();
  numOutputNodes = session.GetOutputCount();

  Ort::AllocatorWithDefaultOptions allocator;

  // Input
  {
    auto inNameAlloc = session.GetInputNameAllocated(0, allocator);
    inputNameAllocs.emplace_back(std::move(inNameAlloc));
    inputNames.push_back(inputNameAllocs.back().get());

    auto inTypeInfo = session.GetInputTypeInfo(0);
    auto inShape  = inTypeInfo.GetTensorTypeAndShapeInfo().GetShape();

    if (inShape.size() == 4) {
      if (inShape[2] == -1 || inShape[3] == -1) {
        isDynamicInputShape = true;
        inputImageShape = cv::Size(640, 640); // Fallback if dynamic
      } else {
        inputImageShape = cv::Size(static_cast<int>(inShape[3]), static_cast<int>(inShape[2]));
      }
    } else {
      throw std::runtime_error("Model input is not 4D! Expect [N, C, H, W].");
    }
  }

  // Outputs
  if (numOutputNodes != 2) {
    throw std::runtime_error("Expected exactly 2 output nodes: output0 and output1.");
  }

  for (size_t i = 0; i < numOutputNodes; ++i) {
    auto outNameAlloc = session.GetOutputNameAllocated(i, allocator);
    outputNameAllocs.emplace_back(std::move(outNameAlloc));
    outputNames.push_back(outputNameAllocs.back().get());
  }

  classNames = utils::getClassNames(labelsPath);
  classColors = utils::generateColors(classNames);

  LOG(INFO) << "[INFO] YOLOv8Seg loaded: " << modelPath << std::endl
       << "   Input shape: " << inputImageShape 
       << (isDynamicInputShape ? " (dynamic)" : "") << std::endl
       << "   #Outputs  : " << numOutputNodes << std::endl
       << "   #Classes  : " << classNames.size() << std::endl;
}

inline void YOLOv8SegDetector::preprocess(const cv::Mat &image,
                     float *&blobPtr,
                     std::vector<int64_t> &inputTensorShape) {
 ScopedTimer timer("Preprocess");
 auto start = std::chrono::high_resolution_clock::now();
 
 cv::Mat resized;
 utils::letterBox(image, resized, inputImageShape,
          cv::Scalar(114,114,114),
          isDynamicInputShape,
          false, true, 32);

 // Update dynamic size
 int rh = resized.rows;
 int rw = resized.cols;
 inputTensorShape[2] = rh;
 inputTensorShape[3] = rw;

 size_t size = (size_t)rh * rw * 3;
 blobPtr = new float[size];


 const int hw = rh * rw;
 const unsigned char* src = resized.data;

 float* blobR = blobPtr;
 float* blobG = blobPtr + hw;
 float* blobB = blobPtr + hw * 2;

 for (int i = 0; i < hw; i++) {
  blobR[i] = src[i * 3 + 2] / 255.0f; // R
  blobG[i] = src[i * 3 + 1] / 255.0f; // G
  blobB[i] = src[i * 3 + 0] / 255.0f; // B
 }
 auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
 std::chrono::high_resolution_clock::now() - start).count();
 (void)(duration);
 //LOG(INFO) << "pre-process took " << duration << " milliseconds." << std::endl;
 
}



std::vector<Segmentation> YOLOv8SegDetector::postprocess(
  const cv::Size &origSize,
  const cv::Size &letterboxSize,
  const std::vector<Ort::Value> &outputs,
  float confThreshold,
  float iouThreshold) 
{
 ScopedTimer timer("PostprocessSeg"); 

 std::vector<Segmentation> results;
 results.reserve(64); 

 const float* output0_ptr = outputs[0].GetTensorData<float>();
 const float* output1_ptr = outputs[1].GetTensorData<float>();

 const auto& shape0 = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
 const auto& shape1 = outputs[1].GetTensorTypeAndShapeInfo().GetShape();

 const int numFeatures  = shape0[1];
 const int numDet    = shape0[2];
 const int numClasses  = numFeatures - 4 - 32;
 const int maskH     = shape1[2];
 const int maskW     = shape1[3];

 if (numDet == 0) return results;


 cv::Mat proto(32, maskH * maskW, CV_32F, const_cast<float*>(output1_ptr));

 std::vector<BoundingBox> boxes; boxes.reserve(numDet);
 std::vector<float>    confs; confs.reserve(numDet);
 std::vector<int>     cls;  cls.reserve(numDet);
 std::vector<std::array<float, 32>> coeffs;
 coeffs.reserve(numDet);

 for (int i = 0; i < numDet; ++i)
 {
   const float xc = output0_ptr[i + 0*numDet];
   const float yc = output0_ptr[i + 1*numDet];
   const float w = output0_ptr[i + 2*numDet];
   const float h = output0_ptr[i + 3*numDet];

   int best = 0;
   float maxConf = 0.f;
   const float* clsPtr = output0_ptr + numDet * 4 + i;

   for (int c = 0; c < numClasses; ++c)
   {
     float v = clsPtr[c * numDet];
     if (v > maxConf) { maxConf = v; best = c; }
   }
   if (maxConf < confThreshold) continue;

   boxes.emplace_back(int(xc - w*0.5f), int(yc - h*0.5f), int(w), int(h));
   confs.emplace_back(maxConf);
   cls.emplace_back(best);

   std::array<float,32> mc;
   const float* mptr = clsPtr + numClasses * numDet;
   for (int m = 0; m < 32; ++m) mc[m] = mptr[m * numDet];
   coeffs.emplace_back(mc);
 }

 if (boxes.empty()) return results;

 // ------------------ NMS --------------------
 std::vector<int> keep;
 utils::NMSBoxes(boxes, confs, confThreshold, iouThreshold, keep);
 if (keep.empty()) return results;

 results.reserve(keep.size());


 const float gain = std::min(
   float(letterboxSize.width) / origSize.width,
   float(letterboxSize.height) / origSize.height);

 const float padW = (letterboxSize.width - origSize.width * gain) * 0.5f;
 const float padH = (letterboxSize.height - origSize.height * gain) * 0.5f;

 const float sx = float(maskW) / letterboxSize.width;
 const float sy = float(maskH) / letterboxSize.height;

 cv::Mat buf(maskH, maskW, CV_32F); 

 for (int id : keep)
 {
   Segmentation seg;
   seg.box = utils::scaleCoords(letterboxSize, boxes[id], origSize, true);
   seg.conf = confs[id];
   seg.classId = cls[id];

   // ----------- mask = sum(coeff[i] * proto[i]) --------------
   const float* pc = coeffs[id].data();
   float* dst = reinterpret_cast<float*>(buf.data);


   std::memset(dst, 0, maskH * maskW * sizeof(float));

   for (int m = 0; m < 32; ++m)
   {
     const float k = pc[m];
     if (std::abs(k) < 1e-6f) continue; 

     const float* src = proto.ptr<float>(m);
     for (int i = 0; i < maskH*maskW; ++i) dst[i] += src[i] * k;
   }

   // -------- sigmoid inplace -------------
   for (int i = 0; i < maskH * maskW; ++i)
   {
     float v = dst[i];
     dst[i] = 1.f / (1.f + std::exp(-v));
   }

   // -------- crop + resize + roi --------
   const int x1 = std::max(0, int((padW) * sx));
   const int y1 = std::max(0, int((padH) * sy));
   const int x2 = std::min(maskW, int((letterboxSize.width - padW) * sx));
   const int y2 = std::min(maskH, int((letterboxSize.height - padH) * sy));

   if (x2 <= x1 || y2 <= y1) continue;

   cv::Mat cropped(buf, cv::Rect(x1, y1, x2 - x1, y2 - y1));
   cv::Mat resized, binary;
   cv::resize(cropped, resized, origSize);

   cv::threshold(resized, binary, 0.5, 255, cv::THRESH_BINARY);
   binary.convertTo(binary, CV_8U);

   seg.mask = cv::Mat::zeros(origSize, CV_8U);
   cv::Rect roi(seg.box.x, seg.box.y, seg.box.width, seg.box.height);
   roi &= cv::Rect(0, 0, binary.cols, binary.rows);
   binary(roi).copyTo(seg.mask(roi));

   results.emplace_back(std::move(seg));
 }

 return results;
}


inline std::vector<Segmentation> YOLOv8SegDetector::segment(const cv::Mat &image,
                              float confThreshold,
                              float iouThreshold) 
{
  ScopedTimer timer("YOLOv8Seg: segment()");

  float *blobPtr = nullptr;
  std::vector<int64_t> inputShape = {1, 3, inputImageShape.height, inputImageShape.width};
  // cv::Mat letterboxImg = preprocess(image, blobPtr, inputShape);
  preprocess(image, blobPtr, inputShape);

  auto start = std::chrono::high_resolution_clock::now();
  size_t inputSize = utils::vectorProduct(inputShape);
  std::vector<float> inputVals(blobPtr, blobPtr + inputSize);
  delete[] blobPtr;

  Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
    memInfo,
    inputVals.data(),
    inputSize,
    inputShape.data(),
    inputShape.size()
  );

  std::vector<Ort::Value> outputs = session.Run(
    Ort::RunOptions{nullptr},
    inputNames.data(),
    &inputTensor,
    numInputNodes,
    outputNames.data(),
    numOutputNodes);
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
   std::chrono::high_resolution_clock::now() - start).count();
   (void)(duration);
   //LOG(INFO) << "inference took " << duration << " milliseconds." << std::endl;

  cv::Size letterboxSize(static_cast<int>(inputShape[3]), static_cast<int>(inputShape[2]));
  return postprocess(image.size(), letterboxSize, outputs, confThreshold, iouThreshold);
}

