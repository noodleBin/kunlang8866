#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/storytelling/proto/story.pb.h"

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/time.h"

using century::cyber::Time;
using century::localization::LocalizationEstimate;

using LocalizationPtr = std::shared_ptr<LocalizationEstimate>;
using LocalizationReaderPtr =
    std::shared_ptr<century::cyber::Reader<LocalizationEstimate>>;

using century::prediction::PredictionObstacles;
using PredictionObstaclePtr = std::shared_ptr<PredictionObstacles>;
using PredictionObstacleReaderPtr =
    std::shared_ptr<century::cyber::Reader<PredictionObstacles>>;

using century::perception::PerceptionObstacles;
using PerceptionObstaclePtr = std::shared_ptr<PerceptionObstacles>;
using PerceptionObstacleReaderPtr =
    std::shared_ptr<century::cyber::Reader<PerceptionObstacles>>;

using NodeType = century::cyber::Node;
using NodePtr = std::unique_ptr<NodeType>;

template <typename MsgTypePtr>
void ReceiveMessageCallback(const MsgTypePtr& msg) {
  AINFO << std::fixed << std::setprecision(10) << msg->header().module_name()
        << " (msg tm | system tm) : " << msg->header().timestamp_sec() << " | "
        << Time::Now().ToSecond();
}

void ReadLoaclization(const NodePtr& node) noexcept {
  const std::string& localization_pose_channel_name =
      "/century/localization/pose";
  LocalizationReaderPtr localization_estimate_reader_ =
      node->CreateReader<LocalizationEstimate>(
          localization_pose_channel_name,
          ReceiveMessageCallback<LocalizationPtr>);
}

void ReadPrediction(const NodePtr& node) noexcept {
  const std::string& prediction_channel_name = "/century/prediction";
  PredictionObstacleReaderPtr prediction_reader_ =
      node->CreateReader<PredictionObstacles>(
          prediction_channel_name,
          ReceiveMessageCallback<PredictionObstaclePtr>);
}

void ReadPerception(const NodePtr& node) noexcept {
  const std::string& perception_channel_name = "/century/perception";
  PerceptionObstacleReaderPtr perception_reader_ =
      node->CreateReader<PerceptionObstacles>(
          perception_channel_name,
          ReceiveMessageCallback<PerceptionObstaclePtr>);
}

int main(int argc, char* argv[]) {
  century::cyber::Init(argv[0]);
  auto listener_node = century::cyber::CreateNode("MsgChecker");
  AINFO << "Start to Read location";
  ReadLoaclization(listener_node);
  ReadPrediction(listener_node);
  ReadPerception(listener_node);
  if (listener_node) {
    AINFO << "End to Read location: " << listener_node->Name();
  }
  century::cyber::WaitForShutdown();
  return 0;
}
