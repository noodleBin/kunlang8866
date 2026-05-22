/******************************************************************************
 * Copyright 2026 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *****************************************************************************/

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/perception/common/perception_vis_bridge/raw_vis_bridge.h"
#include "modules/perception/common/perception_vis_bridge/vis_sink.h"
#include "modules/perception/common/perception_vis_bridge/vis_topic_registry.h"

namespace century {
namespace perception {
namespace {

constexpr char kNodeName[] = "perception_vis_bridge";
constexpr char kDisableShmEnv[] = "PERCEPTION_VIS_DISABLE_SHM";

REGISTER_VIS_TOPIC(debug, "/century/debug/perception/obstacles",
                   "century.perception.PerceptionObstacleDebugMsg");
REGISTER_VIS_TOPIC(prediction, "/century/prediction",
                   "century.prediction.PredictionObstacles");
REGISTER_VIS_TOPIC(localization, "/century/localization/pose",
                   "century.localization.LocalizationEstimate");

std::unique_ptr<VisSink> MakeSink(const std::string& topic) {
  const bool disable_shm = (nullptr != std::getenv(kDisableShmEnv));
  if (disable_shm) {
    AINFO << "PERCEPTION_VIS_DISABLE_SHM is set, use null sink for topic: "
          << topic;
    return std::make_unique<NullVisSink>();
  }
  return std::make_unique<ShmVisSink>(topic);
}

int Run(int /*argc*/, char** argv) {
  if (!cyber::Init(argv[0])) {
    AERROR << "Failed to init cyber.";
    return 1;
  }

  auto node = cyber::CreateNode(kNodeName);
  if (nullptr == node) {
    AERROR << "Failed to create node: " << kNodeName;
    return 1;
  }

  const auto enabled = VisTopicRegistry::Instance().EnabledTopics();
  if (enabled.empty()) {
    AWARN << "No perception vis topics enabled; check PERCEPTION_VIS_TOPICS.";
  }

  const size_t bridge_count = enabled.size();
  std::vector<std::unique_ptr<RawVisBridge>> bridges;
  bridges.reserve(bridge_count);
  for (size_t i = 0; i < bridge_count; ++i) {
    const auto& spec = enabled[i];
    bridges.emplace_back(
        std::make_unique<RawVisBridge>(node.get(), spec, MakeSink(spec.topic)));
  }

  cyber::WaitForShutdown();
  return 0;
}

}  // namespace
}  // namespace perception
}  // namespace century

int main(int argc, char** argv) { return century::perception::Run(argc, argv); }
