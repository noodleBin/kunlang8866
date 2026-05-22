#pragma once

#include <stddef.h>
#include <stdint.h>

namespace century {
namespace perception {

constexpr size_t kPerceptionVisTopicMaxLength = 128;
constexpr size_t kPerceptionVisTypeMaxLength = 128;
constexpr size_t kPerceptionVisPayloadMaxBytes = 16 * 1024 * 1024;

#pragma pack(push, 1)
struct PerceptionVisRawShm {
  char topic[kPerceptionVisTopicMaxLength];
  char msg_type[kPerceptionVisTypeMaxLength];
  uint32_t sequence_num;
  double header_timestamp_sec;
  double measurement_time_sec;
  uint32_t payload_size;
  uint8_t payload[kPerceptionVisPayloadMaxBytes];
};
#pragma pack(pop)

}  // namespace perception
}  // namespace century
