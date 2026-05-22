#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <cstring>

constexpr float SCORE_THRESH = 1e-4f;
constexpr int   NMS_RADIUS   = 4;
constexpr int   MAX_KP     = 1000;
constexpr int   CELL_SIZE  = 8;  
struct HFKeypoint {
  int x;
  int y;
  float score;
};

std::vector<HFKeypoint> ExtractKeypoints(
  const float* scores,
  int H, int W)
{
  constexpr int BORDER = 8;

  std::vector<HFKeypoint> candidates;

  // 1. threshold + border mask
  for (int y = BORDER; y < H - BORDER; ++y) {
      for (int x = BORDER; x < W - BORDER; ++x) {
          float s = scores[y * W + x];
          if (s > SCORE_THRESH) {
              candidates.push_back({x, y, s});
          }
      }
  }

  // 2. sort by score
  std::sort(candidates.begin(), candidates.end(),
            [](auto& a, auto& b) {
                return a.score > b.score;
            });

  // 3. top-K pruning (非常关键)
  if ((int)candidates.size() > MAX_KP * 5) {
      candidates.resize(MAX_KP * 5);
  }

  // 4. NMS
  std::vector<HFKeypoint> keypoints;
  for (const auto& kp : candidates) {
      bool keep = true;
      for (const auto& sel : keypoints) {
          int dx = kp.x - sel.x;
          int dy = kp.y - sel.y;
          if (dx*dx + dy*dy < NMS_RADIUS*NMS_RADIUS) {
              keep = false;
              break;
          }
      }
      if (keep) {
          keypoints.push_back(kp);
          if ((int)keypoints.size() >= MAX_KP)
              break;
      }
  }

  return keypoints;
}


int main() {
  // ============================================================
  // 1. ONNX Runtime 
  // ============================================================
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "hfnet");
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  session_options.SetGraphOptimizationLevel(
    GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

  Ort::Session session(env, "/century/data/data/hfnet.onnx", session_options);

  Ort::AllocatorWithDefaultOptions allocator;

  // ============================================================
  // 2.input / output 
  // ============================================================
  size_t num_inputs = session.GetInputCount();
  size_t num_outputs = session.GetOutputCount();

  std::vector<Ort::AllocatedStringPtr> input_name_ptrs;
  std::vector<Ort::AllocatedStringPtr> output_name_ptrs;
  std::vector<const char*> input_names;
  std::vector<const char*> output_names;

  for (size_t i = 0; i < num_inputs; ++i) {
    input_name_ptrs.emplace_back(
      session.GetInputNameAllocated(i, allocator));
    input_names.push_back(input_name_ptrs.back().get());

    std::cout << "Input " << i << ": "
          << input_names.back() << std::endl;
  }

  for (size_t i = 0; i < num_outputs; ++i) {
    output_name_ptrs.emplace_back(
      session.GetOutputNameAllocated(i, allocator));
    output_names.push_back(output_name_ptrs.back().get());

    std::cout << "Output " << i << ": "
          << output_names.back() << std::endl;
  }

  // ============================================================
  // 3. read and preprocess image
  // ============================================================

  const int H = 480;
  const int W = 640;

  cv::Mat img_u8 =
  cv::imread("/century/data/data/test.jpg",
         cv::IMREAD_GRAYSCALE);

if (img_u8.empty()) {
  std::cerr << "Failed to load image\n";
  return -1;
}

cv::resize(img_u8, img_u8, cv::Size(W, H));

// HF-Net float 
cv::Mat img_float;
img_u8.convertTo(img_float, CV_32F, 1.0 / 255.0);



  std::vector<float> input_data(H * W);
std::memcpy(input_data.data(),
      img_float.data,
      H * W * sizeof(float));

// NHWC
std::vector<int64_t> input_shape = {1, H, W, 1};

Ort::MemoryInfo memory_info =
  Ort::MemoryInfo::CreateCpu(
    OrtDeviceAllocator, OrtMemTypeCPU);

Ort::Value input_tensor =
  Ort::Value::CreateTensor<float>(
    memory_info,
    input_data.data(),
    input_data.size(),
    input_shape.data(),
    input_shape.size());


  // ============================================================
  // 4. inference
  // ============================================================
  auto output_tensors = session.Run(
    Ort::RunOptions{nullptr},
    input_names.data(),
    &input_tensor,
    1,
    output_names.data(),
    output_names.size());

  // ============================================================
  // 5. output parse
  // ============================================================
  for (size_t i = 0; i < output_tensors.size(); ++i) {
    auto& t = output_tensors[i];
    auto info = t.GetTensorTypeAndShapeInfo();
    auto shape = info.GetShape();

    std::cout << "Output[" << i << "] shape: ";
    for (auto s : shape) std::cout << s << " ";
    std::cout << std::endl;
  }

  
  // order：keypoints, local_desc, scores, global_desc

  auto& global_desc_tensor = output_tensors.back();
  float* global_desc = global_desc_tensor.GetTensorMutableData<float>();


  auto gd_shape = global_desc_tensor
            .GetTensorTypeAndShapeInfo()
            .GetShape();

  int gd_dim = gd_shape[1];


  std::cout << "Global desc first 5 dims: ";
  for (int i = 0; i < 5 && i < gd_dim; ++i) {
    std::cout << global_desc[i] << " ";
  }

  // --------------------------------------------------
// scores_dense
// --------------------------------------------------
  auto& scores_tensor = output_tensors[0];
  float* scores =
  scores_tensor.GetTensorMutableData<float>();

  float min_s = 1e9, max_s = -1e9;
  for (int i = 0; i < H * W; ++i) {
    min_s = std::min(min_s, scores[i]);
    max_s = std::max(max_s, scores[i]);
  }
  std::cout << "scores_dense min/max: "
        << min_s << " / " << max_s << std::endl;
  // extract keypoints
  auto hf_kps = ExtractKeypoints(scores, H, W);

  std::cout << "Extracted keypoints: "
      << hf_kps.size() << std::endl;

// --------------------------------------------------
// OpenCV visualizer
// --------------------------------------------------
  cv::Mat vis;
  cv::cvtColor(img_u8, vis, cv::COLOR_GRAY2BGR);

  std::vector<cv::KeyPoint> cv_kps;
  for (const auto& kp : hf_kps) {
    cv_kps.emplace_back(
      cv::Point2f(kp.x, kp.y),
      3.0f,   // size
      -1,
      kp.score
    );
  }

  for (const auto& kp : hf_kps) {
    cv::circle(
      vis,
      cv::Point(kp.x, kp.y),
      2,
      cv::Scalar(0, 255, 0),
      -1
    );
  }


  cv::imwrite("hfnet_keypoints.jpg", vis);

  std::cout << std::endl;

  return 0;
}
