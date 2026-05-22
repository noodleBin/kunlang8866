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

#include "modules/prediction/evaluator/vehicle/vectornet_trt_evaluator.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <unordered_map>

#include "cyber/common/log.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"

#include "modules/perception/lidar/lib/detector/dnn_common/common/tensorrt.hpp"

namespace century {
namespace prediction {

using century::common::TrajectoryPoint;
using century::common::math::Vec2d;

namespace {

cudaStream_t ToCudaStream(void* stream) {
  return reinterpret_cast<cudaStream_t>(stream);
}

bool FileExists(const std::string& path) {
  std::ifstream fin(path);
  return fin.good();
}

std::string ResolveVectornetEnginePath() {
  const std::string legacy_data_path =
      "/century/modules/prediction/data/vectornet_vehicle.engine";
#if defined(__aarch64__) || defined(__arm__)
  const std::string preferred =
      FLAGS_trt_vehicle_vectornet_arm_engine_file;
#elif defined(__x86_64__) || defined(__i386__) || defined(_M_X64) || \
    defined(_M_IX86)
  const std::string preferred =
      FLAGS_trt_vehicle_vectornet_x86_engine_file;
#else
  const std::string preferred = FLAGS_trt_vehicle_vectornet_engine_file;
#endif

  if (FileExists(preferred)) {
    return preferred;
  }
  if (preferred != FLAGS_trt_vehicle_vectornet_engine_file &&
      FileExists(FLAGS_trt_vehicle_vectornet_engine_file)) {
    AINFO << "Platform VectorNet TRT engine not found: " << preferred
          << ", fallback to " << FLAGS_trt_vehicle_vectornet_engine_file;
    return FLAGS_trt_vehicle_vectornet_engine_file;
  }
  if (preferred != legacy_data_path && FileExists(legacy_data_path)) {
    AINFO << "Platform VectorNet TRT engine not found: " << preferred
          << ", fallback to " << legacy_data_path;
    return legacy_data_path;
  }
  return preferred;
}

}  // namespace

VectornetTrtEvaluator::VectornetTrtEvaluator() {
  evaluator_type_ = ObstacleConf::VECTORNET_TRT_EVALUATOR;
  LoadEngine();
}

VectornetTrtEvaluator::~VectornetTrtEvaluator() {
  if (stream_) cudaStreamDestroy(ToCudaStream(stream_));
  if (d_obs_pos_) cudaFree(d_obs_pos_);
  if (d_obs_pos_step_) cudaFree(d_obs_pos_step_);
  if (d_vector_data_) cudaFree(d_vector_data_);
  if (d_vector_mask_) cudaFree(d_vector_mask_);
  if (d_polyline_mask_) cudaFree(d_polyline_mask_);
  if (d_rand_mask_) cudaFree(d_rand_mask_);
  if (d_polyline_id_) cudaFree(d_polyline_id_);
  if (d_output_) cudaFree(d_output_);
}

void VectornetTrtEvaluator::Clear() {}

void VectornetTrtEvaluator::LoadEngine() {
  const std::string engine_path = ResolveVectornetEnginePath();
  AINFO << "Loading VectorNet TRT engine from: " << engine_path;
  engine_ = TensorRT::load(engine_path);
  if (!engine_) {
    AERROR << "Failed to load VectorNet TRT engine: "
           << engine_path;
    return;
  }
  engine_->print("VectorNet-TRT");

  // Log engine tensor info for diagnostics
  AINFO << "VectorNet TRT engine has " << engine_->num_bindings()
        << " bindings, dynamic=" << engine_->has_dynamic_dim();
  for (int i = 0; i < engine_->num_bindings(); ++i) {
    auto dims = engine_->static_dims(i);
    std::string dim_str;
    for (int d : dims) dim_str += std::to_string(d) + " ";
    AINFO << "  Binding " << i << ": input=" << engine_->is_input(i)
          << " dtype=" << static_cast<int>(engine_->dtype(i))
          << " dims=[" << dim_str << "]";
  }

  // If dynamic shapes, set them for batch=1
  if (engine_->has_dynamic_dim()) {
    AINFO << "Setting dynamic input shapes for batch=1";
    engine_->set_run_dims("target_obs_pos", {1, 20, 2});
    engine_->set_run_dims("target_obs_pos_step", {1, 20, 2});
    engine_->set_run_dims("vector_data", {1, 450, 50, 9});
    engine_->set_run_dims("vector_mask", {1, 450, 50});
    engine_->set_run_dims("polyline_mask", {1, 450});
    engine_->set_run_dims("rand_mask", {1, 450});
    engine_->set_run_dims("polyline_id", {1, 450, 2});
  }

  cudaStream_t stream = nullptr;
  cudaStreamCreate(&stream);
  stream_ = stream;

  // Pre-allocate GPU buffers matching the model's fixed input/output shapes
  cudaMalloc(&d_obs_pos_, 1 * 20 * 2 * sizeof(float));
  cudaMalloc(&d_obs_pos_step_, 1 * 20 * 2 * sizeof(float));
  cudaMalloc(&d_vector_data_, 1 * 450 * 50 * 9 * sizeof(float));
  cudaMalloc(&d_vector_mask_, 1 * 450 * 50 * sizeof(bool));
  cudaMalloc(&d_polyline_mask_, 1 * 450 * sizeof(bool));
  cudaMalloc(&d_rand_mask_, 1 * 450 * sizeof(bool));
  cudaMalloc(&d_polyline_id_, 1 * 450 * 2 * sizeof(float));
  cudaMalloc(&d_output_, 1 * 30 * 2 * sizeof(float));

  // Zero out rand_mask (it's always zeros)
  cudaMemset(d_rand_mask_, 0, 1 * 450 * sizeof(bool));

  AINFO << "VectorNet TRT engine loaded successfully.";
}

bool VectornetTrtEvaluator::Evaluate(
    Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container) {
  obstacle_ptr->SetEvaluatorType(evaluator_type_);
  Clear();

  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return false;
  }
  if (!engine_) {
    AERROR << "VectorNet TRT engine not loaded.";
    return false;
  }

  int obs_num = 0;
  auto t_start = std::chrono::system_clock::now();

  // -----------------------------------------------------------------------
  // Step 1: Extract obstacle features (same logic as VectornetEvaluator)
  // -----------------------------------------------------------------------
  std::vector<float> target_obs_pos(20 * 2, 0.0f);
  std::vector<float> target_obs_pos_step(20 * 2, 0.0f);
  std::vector<float> vector_mask(450 * 50, 0.0f);
  std::vector<float> obstacle_data;
  std::vector<float> all_obs_p_id;

  if (!ProcessObstaclePosition(obstacle_ptr, obstacles_container,
                               &target_obs_pos, &target_obs_pos_step,
                               &vector_mask, &obstacle_data, &all_obs_p_id,
                               &obs_num)) {
    AERROR << "Obstacle [" << id << "] processing obstacle position fails.";
    return false;
  }

  auto t_obs = std::chrono::system_clock::now();
  AINFO << "TRT obstacle vectors: "
        << std::chrono::duration<double>(t_obs - t_start).count() * 1000
        << " ms";

  // -----------------------------------------------------------------------
  // Step 2: Query map data
  // -----------------------------------------------------------------------
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  const double pos_x = latest_feature_ptr->position().x();
  const double pos_y = latest_feature_ptr->position().y();
  common::PointENU center_point =
      common::util::PointFactory::ToPointENU(pos_x, pos_y);
  const double heading = latest_feature_ptr->velocity_heading();

  FeatureVector map_feature;
  PidVector map_p_id;
  if (!vector_net_.query(center_point, heading, &map_feature, &map_p_id)) {
    return false;
  }

  auto t_map = std::chrono::system_clock::now();
  AINFO << "TRT map query: "
        << std::chrono::duration<double>(t_map - t_obs).count() * 1000
        << " ms";

  // -----------------------------------------------------------------------
  // Step 3: Process map data
  // -----------------------------------------------------------------------
  int map_polyline_num = static_cast<int>(map_feature.size());
  std::vector<float> map_data(map_polyline_num * 50 * 9, 0.0f);
  std::vector<float> all_map_p_id(map_polyline_num * 2, 0.0f);

  if (!ProcessMapData(&map_feature, &map_p_id, obs_num, &map_data,
                      &all_map_p_id, &vector_mask)) {
    AERROR << "Obstacle [" << id << "] processing map data fails.";
    return false;
  }

  std::vector<float> vector_data;
  std::vector<float> polyline_id;
  std::vector<uint8_t> bool_vector_mask;
  std::vector<uint8_t> polyline_mask;
  std::vector<uint8_t> rand_mask;
  AssembleInferenceInputs(obs_num, map_polyline_num, obstacle_data, map_data,
                          all_obs_p_id, all_map_p_id, vector_mask,
                          &vector_data, &polyline_id, &bool_vector_mask,
                          &polyline_mask, &rand_mask);

  auto t_prep = std::chrono::system_clock::now();
  AINFO << "TRT data preparation: "
        << std::chrono::duration<double>(t_prep - t_map).count() * 1000
        << " ms";

  std::vector<float> output(30 * 2);
  if (!RunInference(target_obs_pos, target_obs_pos_step, vector_data,
                    polyline_id, bool_vector_mask, polyline_mask, rand_mask,
                    &output)) {
    return false;
  }

  auto t_infer = std::chrono::system_clock::now();
  AINFO << "TRT inference: "
        << std::chrono::duration<double>(t_infer - t_prep).count() * 1000
        << " ms";

  FillTrajectory(latest_feature_ptr, pos_x, pos_y, heading, output);

  auto t_end = std::chrono::system_clock::now();
  AINFO << "TRT total: "
        << std::chrono::duration<double>(t_end - t_start).count() * 1000
        << " ms";

  return true;
}

void VectornetTrtEvaluator::AssembleInferenceInputs(
    int obs_num, int map_polyline_num, const std::vector<float>& obstacle_data,
    const std::vector<float>& map_data,
    const std::vector<float>& all_obs_p_id,
    const std::vector<float>& all_map_p_id,
    const std::vector<float>& vector_mask,
    std::vector<float>* vector_data,
    std::vector<float>* polyline_id,
    std::vector<uint8_t>* bool_vector_mask,
    std::vector<uint8_t>* polyline_mask,
    std::vector<uint8_t>* rand_mask) {
  int data_length = std::min(obs_num + map_polyline_num, 450);
  int map_copy = std::min(map_polyline_num, 450 - obs_num);

  vector_data->assign(450 * 50 * 9, 0.0f);
  std::memcpy(vector_data->data(), obstacle_data.data(),
              obs_num * 50 * 9 * sizeof(float));
  if (map_copy > 0) {
    std::memcpy(vector_data->data() + obs_num * 50 * 9, map_data.data(),
                map_copy * 50 * 9 * sizeof(float));
  }

  polyline_id->assign(450 * 2, 0.0f);
  std::memcpy(polyline_id->data(), all_obs_p_id.data(),
              obs_num * 2 * sizeof(float));
  if (map_copy > 0) {
    std::memcpy(polyline_id->data() + obs_num * 2, all_map_p_id.data(),
                map_copy * 2 * sizeof(float));
  }

  polyline_mask->assign(450, 0);
  for (int i = data_length; i < 450; ++i) {
    polyline_mask->at(i) = 1;
  }

  bool_vector_mask->assign(450 * 50, 0);
  for (int i = 0; i < 450 * 50; ++i) {
    bool_vector_mask->at(i) = (vector_mask[i] > 0.5f) ? 1 : 0;
  }

  rand_mask->assign(450, 0);
}

bool VectornetTrtEvaluator::RunInference(
    const std::vector<float>& target_obs_pos,
    const std::vector<float>& target_obs_pos_step,
    const std::vector<float>& vector_data,
    const std::vector<float>& polyline_id,
    const std::vector<uint8_t>& bool_vector_mask,
    const std::vector<uint8_t>& polyline_mask,
    const std::vector<uint8_t>& rand_mask,
    std::vector<float>* output) {
  std::lock_guard<std::mutex> lock(trt_mutex_);

  cudaMemcpyAsync(d_obs_pos_, target_obs_pos.data(), 20 * 2 * sizeof(float),
                  cudaMemcpyHostToDevice, ToCudaStream(stream_));
  cudaMemcpyAsync(d_obs_pos_step_, target_obs_pos_step.data(),
                  20 * 2 * sizeof(float), cudaMemcpyHostToDevice,
                  ToCudaStream(stream_));
  cudaMemcpyAsync(d_vector_data_, vector_data.data(),
                  450 * 50 * 9 * sizeof(float), cudaMemcpyHostToDevice,
                  ToCudaStream(stream_));
  cudaMemcpyAsync(d_vector_mask_, bool_vector_mask.data(),
                  450 * 50 * sizeof(bool), cudaMemcpyHostToDevice,
                  ToCudaStream(stream_));
  cudaMemcpyAsync(d_polyline_mask_, polyline_mask.data(), 450 * sizeof(bool),
                  cudaMemcpyHostToDevice, ToCudaStream(stream_));
  cudaMemcpyAsync(d_rand_mask_, rand_mask.data(), 450 * sizeof(bool),
                  cudaMemcpyHostToDevice, ToCudaStream(stream_));
  cudaMemcpyAsync(d_polyline_id_, polyline_id.data(),
                  450 * 2 * sizeof(float), cudaMemcpyHostToDevice,
                  ToCudaStream(stream_));

  std::unordered_map<std::string, const void*> bindings;
  bindings["target_obs_pos"] = d_obs_pos_;
  bindings["target_obs_pos_step"] = d_obs_pos_step_;
  bindings["vector_data"] = d_vector_data_;
  bindings["vector_mask"] = d_vector_mask_;
  bindings["polyline_mask"] = d_polyline_mask_;
  bindings["rand_mask"] = d_rand_mask_;
  bindings["polyline_id"] = d_polyline_id_;
  bindings["trajectory"] = d_output_;

  if (!engine_->forward(bindings, stream_)) {
    AERROR << "VectorNet TRT forward failed. Bindings provided: "
           << bindings.size() << ", engine expects: " << engine_->num_bindings();
    for (const auto& kv : bindings) {
      AERROR << "  binding: " << kv.first << " ptr=" << kv.second;
    }
    return false;
  }

  cudaMemcpyAsync(output->data(), d_output_, 30 * 2 * sizeof(float),
                  cudaMemcpyDeviceToHost, ToCudaStream(stream_));
  cudaStreamSynchronize(ToCudaStream(stream_));
  return true;
}

void VectornetTrtEvaluator::FillTrajectory(
    Feature* latest_feature_ptr, double pos_x, double pos_y, double heading,
    const std::vector<float>& output) {
  Trajectory* trajectory = latest_feature_ptr->add_predicted_trajectory();
  trajectory->set_probability(1.0);

  for (int i = 0; i < 30; ++i) {
    double prev_x = pos_x;
    double prev_y = pos_y;
    if (i > 0) {
      const auto& last_point =
          trajectory->trajectory_point(i - 1).path_point();
      prev_x = last_point.x();
      prev_y = last_point.y();
    }
    TrajectoryPoint* point = trajectory->add_trajectory_point();
    double dx = static_cast<double>(output[i * 2]);
    double dy = static_cast<double>(output[i * 2 + 1]);

    Vec2d offset(dx, dy);
    Vec2d rotated_offset = offset.rotate(heading - (M_PI / 2));
    double point_x = pos_x + rotated_offset.x();
    double point_y = pos_y + rotated_offset.y();
    point->mutable_path_point()->set_x(point_x);
    point->mutable_path_point()->set_y(point_y);

    if (i < 10) {
      point->mutable_path_point()->set_theta(heading);
    } else {
      point->mutable_path_point()->set_theta(std::atan2(
          trajectory->trajectory_point(i).path_point().y() -
              trajectory->trajectory_point(i - 1).path_point().y(),
          trajectory->trajectory_point(i).path_point().x() -
              trajectory->trajectory_point(i - 1).path_point().x()));
    }
    point->set_relative_time(static_cast<double>(i) *
                             FLAGS_prediction_trajectory_time_resolution);
    if (i == 0) {
      point->set_v(latest_feature_ptr->speed());
    } else {
      double diff_x = point_x - prev_x;
      double diff_y = point_y - prev_y;
      point->set_v(std::hypot(diff_x, diff_y) /
                   FLAGS_prediction_trajectory_time_resolution);
    }
  }
}

// ---------------------------------------------------------------------------
// Feature extraction — same logic as VectornetEvaluator but using float arrays
// ---------------------------------------------------------------------------

bool VectornetTrtEvaluator::ExtractObstaclesHistory(
    Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container,
    std::vector<std::pair<double, double>>* target_pos_history,
    std::vector<std::pair<double, double>>* all_obs_length,
    std::vector<std::vector<std::pair<double, double>>>* all_obs_pos_history,
    std::vector<float>* vector_mask) {
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos = std::make_pair(
      obs_curr_feature.position().x(), obs_curr_feature.position().y());

  // Extract target obstacle history
  for (std::size_t i = 0; i < obstacle_ptr->history_size() && i < 20; ++i) {
    const Feature& target_feature = obstacle_ptr->feature(i);
    if (!target_feature.IsInitialized()) break;
    target_pos_history->at(i) = WorldCoordToObjCoordNorth(
        std::make_pair(target_feature.position().x(),
                       target_feature.position().y()),
        obs_curr_pos, obs_curr_heading);
  }
  all_obs_length->emplace_back(
      std::make_pair(obs_curr_feature.length(), obs_curr_feature.width()));
  all_obs_pos_history->emplace_back(*target_pos_history);

  // Extract other obstacles
  std::vector<std::pair<double, double>> pos_history(20, {0.0, 0.0});
  for (int obs_id :
       obstacles_container->curr_frame_considered_obstacle_ids()) {
    Obstacle* obstacle = obstacles_container->GetObstacle(obs_id);
    int target_id = obstacle_ptr->id();
    if (obs_id == target_id) continue;

    const Feature& other_obs_curr_feature = obstacle->latest_feature();
    all_obs_length->emplace_back(std::make_pair(
        other_obs_curr_feature.length(), other_obs_curr_feature.width()));

    size_t obs_his_size = obstacle->history_size();
    obs_his_size = obs_his_size <= 20 ? obs_his_size : 20;
    int cur_idx = static_cast<int>(all_obs_pos_history->size());

    // Set vector mask for padding positions
    int mask_end = (obs_his_size > 1) ? (50 - (obs_his_size - 1)) : 49;
    for (int m = 0; m < mask_end && cur_idx < 450; ++m) {
      vector_mask->at(cur_idx * 50 + m) = 1.0f;
    }

    std::fill(pos_history.begin(), pos_history.end(),
              std::make_pair(0.0, 0.0));
    for (size_t i = 0; i < obs_his_size; ++i) {
      const Feature& feature = obstacle->feature(i);
      if (!feature.IsInitialized()) break;
      pos_history[i] = WorldCoordToObjCoordNorth(
          std::make_pair(feature.position().x(), feature.position().y()),
          obs_curr_pos, obs_curr_heading);
    }
    all_obs_pos_history->emplace_back(pos_history);
  }
  return true;
}

bool VectornetTrtEvaluator::ProcessObstaclePosition(
    Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container,
    std::vector<float>* target_obs_pos,
    std::vector<float>* target_obs_pos_step,
    std::vector<float>* vector_mask,
    std::vector<float>* obstacle_data,
    std::vector<float>* all_obs_p_id, int* obs_num) {
  std::vector<std::pair<double, double>> target_pos_history(20, {0.0, 0.0});
  std::vector<std::pair<double, double>> all_obs_length;
  std::vector<std::vector<std::pair<double, double>>> all_obs_pos_history;

  if (!ExtractObstaclesHistory(obstacle_ptr, obstacles_container,
                               &target_pos_history, &all_obs_length,
                               &all_obs_pos_history, vector_mask)) {
    return false;
  }

  // Fill target_obs_pos [20, 2] and target_obs_pos_step [20, 2]
  for (int j = 0; j < 20; ++j) {
    int idx = 19 - j;
    (*target_obs_pos)[idx * 2] =
        static_cast<float>(target_pos_history[j].first);
    (*target_obs_pos)[idx * 2 + 1] =
        static_cast<float>(target_pos_history[j].second);
    if (j == 19 ||
        (j > 0 && target_pos_history[j + 1].first == 0.0)) {
      break;
    }
    (*target_obs_pos_step)[idx * 2] = static_cast<float>(
        target_pos_history[j].first - target_pos_history[j + 1].first);
    (*target_obs_pos_step)[idx * 2 + 1] = static_cast<float>(
        target_pos_history[j].second - target_pos_history[j + 1].second);
  }

  *obs_num = static_cast<int>(all_obs_pos_history.size());

  // Build obstacle_data [obs_num, 50, 9] and all_obs_p_id [obs_num, 2]
  obstacle_data->resize(*obs_num * 50 * 9, 0.0f);
  all_obs_p_id->resize(*obs_num * 2, 0.0f);

  for (int i = 0; i < *obs_num; ++i) {
    // Compute obs_p_id (min x, min y)
    double min_x = std::numeric_limits<float>::max();
    double min_y = std::numeric_limits<float>::max();
    for (int j = 0; j < 20; ++j) {
      if (min_x > all_obs_pos_history[i][j].first)
        min_x = all_obs_pos_history[i][j].first;
      if (min_y > all_obs_pos_history[i][j].second)
        min_y = all_obs_pos_history[i][j].second;
    }
    (*all_obs_p_id)[i * 2] = static_cast<float>(min_x);
    (*all_obs_p_id)[i * 2 + 1] = static_cast<float>(min_y);

    // Build obs position pairs [start_x, start_y, end_x, end_y]
    // stored in last 19 rows of the 50-row block (rows 31..49)
    // Attribute: 11.0 (target) or 10.0 (other), 4.0 (vehicle)
    double attr_type = (i == 0) ? 11.0 : 10.0;
    double attr_boundary = 4.0;

    for (int j = 0; j < 19; ++j) {
      int row = 50 - 19 + j;  // rows 31..49
      int base = (i * 50 + row) * 9;

      // start point
      (*obstacle_data)[base + 0] =
          static_cast<float>(all_obs_pos_history[i][j + 1].first);
      (*obstacle_data)[base + 1] =
          static_cast<float>(all_obs_pos_history[i][j + 1].second);
      // end point
      (*obstacle_data)[base + 2] =
          static_cast<float>(all_obs_pos_history[i][j].first);
      (*obstacle_data)[base + 3] =
          static_cast<float>(all_obs_pos_history[i][j].second);
      // length
      (*obstacle_data)[base + 4] =
          static_cast<float>(all_obs_length[i].first);
      (*obstacle_data)[base + 5] =
          static_cast<float>(all_obs_length[i].second);
      // attribute
      (*obstacle_data)[base + 6] = static_cast<float>(attr_type);
      (*obstacle_data)[base + 7] = static_cast<float>(attr_boundary);
      // polyline id (filled from p_id, but here it's the cluster index)
      (*obstacle_data)[base + 8] = static_cast<float>(500 + i);
    }
  }

  return true;
}

bool VectornetTrtEvaluator::ProcessMapData(
    FeatureVector* map_feature, PidVector* map_p_id, int obs_num,
    std::vector<float>* map_data, std::vector<float>* all_map_p_id,
    std::vector<float>* vector_mask) {
  int map_polyline_num = static_cast<int>(map_feature->size());

  for (int i = 0; i < map_polyline_num && obs_num + i < 450; ++i) {
    size_t one_polyline_vector_size = map_feature->at(i).size();
    // Set vector mask for padding
    if (one_polyline_vector_size < 50) {
      for (size_t m = one_polyline_vector_size; m < 50; ++m) {
        (*vector_mask)[(obs_num + i) * 50 + m] = 1.0f;
      }
    }
  }

  for (int i = 0; i < map_polyline_num && i + obs_num < 450; ++i) {
    // Copy p_id
    if (i < static_cast<int>(map_p_id->size()) &&
        map_p_id->at(i).size() >= 2) {
      (*all_map_p_id)[i * 2] = static_cast<float>(map_p_id->at(i)[0]);
      (*all_map_p_id)[i * 2 + 1] = static_cast<float>(map_p_id->at(i)[1]);
    }

    // Copy polyline features [50, 9]
    int one_polyline_vector_size =
        static_cast<int>(map_feature->at(i).size());
    for (int j = 0; j < one_polyline_vector_size && j < 50; ++j) {
      const auto& feat = map_feature->at(i)[j];
      for (int d = 0; d < 9 && d < static_cast<int>(feat.size()); ++d) {
        (*map_data)[(i * 50 + j) * 9 + d] = static_cast<float>(feat[d]);
      }
    }
  }

  return true;
}

}  // namespace prediction
}  // namespace century
