/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/pipeline/vector_net.h"

namespace TensorRT {
class Engine;
}

namespace century {
namespace prediction {

/**
 * VectorNet evaluator using TensorRT for inference.
 *
 * Same model architecture and input format as VectornetEvaluator (PyTorch),
 * but uses a pre-built TRT engine (.engine) for faster inference.
 *
 * Input tensors (matching the original PyTorch model):
 *   obs_pos:       [1, 20, 2]    target obstacle position history
 *   obs_pos_step:  [1, 20, 2]    target obstacle position deltas
 *   vector_data:   [1, 450, 50, 9]  all polyline data (obs + map, padded)
 *   vector_mask:   [1, 450, 50]  bool, padding mask for vectors
 *   polyline_mask: [1, 450]      bool, padding mask for polylines
 *   rand_mask:     [1, 450]      bool, placeholder (all zeros)
 *   polyline_id:   [1, 450, 2]   polyline center coordinates
 *
 * Output tensor:
 *   trajectory:    [1, 30, 2]    predicted trajectory offsets (dx, dy)
 *
 * Engine is built offline via:
 *   python3 vectornet_export_onnx.py → trtexec → .engine
 */
class VectornetTrtEvaluator : public Evaluator {
 public:
  VectornetTrtEvaluator();
  ~VectornetTrtEvaluator() override;

  void Clear();

  bool Evaluate(Obstacle* obstacle_ptr,
                ObstaclesContainer* obstacles_container) override;

  std::string GetName() override { return "VECTORNET_TRT_EVALUATOR"; }

 private:
  void LoadEngine();

  // Extract obstacle history (reuses same logic as VectornetEvaluator)
  bool ExtractObstaclesHistory(
      Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container,
      std::vector<std::pair<double, double>>* curr_pos_history,
      std::vector<std::pair<double, double>>* all_obs_length,
      std::vector<std::vector<std::pair<double, double>>>* all_obs_pos_history,
      std::vector<float>* vector_mask);

  // Process obstacle positions into flat arrays
  bool ProcessObstaclePosition(
      Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container,
      std::vector<float>* target_obs_pos,
      std::vector<float>* target_obs_pos_step,
      std::vector<float>* vector_mask,
      std::vector<float>* obstacle_data,
      std::vector<float>* all_obs_p_id,
      int* obs_num);

  // Process map data into flat arrays
  bool ProcessMapData(FeatureVector* map_feature, PidVector* map_p_id,
                      int obs_num,
                      std::vector<float>* map_data,
                      std::vector<float>* all_map_p_id,
                      std::vector<float>* vector_mask);

  void AssembleInferenceInputs(
      int obs_num, int map_polyline_num,
      const std::vector<float>& obstacle_data,
      const std::vector<float>& map_data,
      const std::vector<float>& all_obs_p_id,
      const std::vector<float>& all_map_p_id,
      const std::vector<float>& vector_mask,
      std::vector<float>* vector_data,
      std::vector<float>* polyline_id,
      std::vector<uint8_t>* bool_vector_mask,
      std::vector<uint8_t>* polyline_mask,
      std::vector<uint8_t>* rand_mask);

  bool RunInference(const std::vector<float>& target_obs_pos,
                    const std::vector<float>& target_obs_pos_step,
                    const std::vector<float>& vector_data,
                    const std::vector<float>& polyline_id,
                    const std::vector<uint8_t>& bool_vector_mask,
                    const std::vector<uint8_t>& polyline_mask,
                    const std::vector<uint8_t>& rand_mask,
                    std::vector<float>* output);

  void FillTrajectory(Feature* latest_feature_ptr, double pos_x, double pos_y,
                      double heading, const std::vector<float>& output);

 private:
  std::shared_ptr<TensorRT::Engine> engine_;
  // Opaque CUDA stream handle. Keep CUDA headers out of this public header so
  // non-CUDA targets that include the evaluator can still compile.
  void* stream_ = nullptr;

  // GPU buffers (pre-allocated)
  void* d_obs_pos_ = nullptr;         // [1, 20, 2]
  void* d_obs_pos_step_ = nullptr;    // [1, 20, 2]
  void* d_vector_data_ = nullptr;     // [1, 450, 50, 9]
  void* d_vector_mask_ = nullptr;     // [1, 450, 50]  bool
  void* d_polyline_mask_ = nullptr;   // [1, 450]      bool
  void* d_rand_mask_ = nullptr;       // [1, 450]      bool
  void* d_polyline_id_ = nullptr;     // [1, 450, 2]
  void* d_output_ = nullptr;          // [1, 30, 2]

  // TensorRT execution contexts and these device buffers are reused by this
  // evaluator, while EvaluatorManager may call Evaluate() from multiple
  // threads.
  std::mutex trt_mutex_;

  VectorNet vector_net_;
};

}  // namespace prediction
}  // namespace century
