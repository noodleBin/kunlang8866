/******************************************************************************
 * Copyright 2025 The Century Authors. All Rights Reserved.
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

#include "tztek_camera.h"

namespace century {
namespace drivers {
namespace camera {

TztekCamera::TztekCamera() : node_(century::cyber::CreateNode("camera")) {}

TztekCamera::~TztekCamera() {}

century::cyber::proto::QosProfile TztekCamera::CreateQosProfile() {
  century::cyber::proto::QosProfile qos;
  qos.set_history(century::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos.set_reliability(
      century::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos.set_durability(
      century::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  return qos;
}

std::shared_ptr<Writer<Image>> TztekCamera::CreateImageWriter(
    const std::string channel) {
  century::cyber::proto::RoleAttributes writer_attr;
  writer_attr.set_channel_name(channel);
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  return node_->CreateWriter<Image>(writer_attr);
}

std::shared_ptr<Writer<CompressedImage>>
TztekCamera::CreateCompressedImageWriter(const std::string channel) {
  century::cyber::proto::RoleAttributes writer_attr;
  writer_attr.set_channel_name(channel);
  *writer_attr.mutable_qos_profile() = CreateQosProfile();
  return node_->CreateWriter<CompressedImage>(writer_attr);
}

void TztekCamera::ChangeOwn() {
  std::string cmd_prefix{"echo nvidia | sudo -S "};
  std::string change_video_own_cmd{"chown nvidia:nvidia /dev/video*"};
  std::string change_serial_own_cmd{"chown nvidia:nvidia /dev/ttyTHS1"};
  std::string change_gpio_own_cmd{
      "chmod 777 /sys/class/tz_gpio/camera_*/value"};

  auto cmd = cmd_prefix + change_video_own_cmd;
  std::system(cmd.data());

  cmd = cmd_prefix + change_serial_own_cmd;
  std::system(cmd.data());

  cmd = cmd_prefix + change_gpio_own_cmd;
  std::system(cmd.data());
}

bool TztekCamera::Init(std::shared_ptr<Config> camera_config) {
  int config_index = 63;
  AINFO << "TztekCamera init.";
  ChangeOwn();
  ResetPower();
  int ret = 0;

  std::vector<int> channum;
  int bit_position = 0;
  while (config_index > 0) {
    if (config_index & 1) {
      channum.push_back(bit_position);
    }
    config_index >>= 1;
    bit_position++;
  }

  // "/home/nvidia/cam_geac/cfg/hb_j5dev.json"
  ret = hb_vin_init(0, camera_config->hb_j5dev_path().data());
  if (ret < 0) {
    AERROR << "Failed to hb_cam_init.";
    return false;
  }

  output_raw_ = camera_config->output_raw();
  int cam_num = channum.size();
  for (int i = 0; i < cam_num; i++) {
    std::string raw_output_channel;
    std::string compressed_output_channel;
    std::string frame_id;
    bool find = false;
    int pipeid = channum[i];
    uint32_t video, weight, height, fps, format;
    ret = hb_port_mapping(pipeid, &video, &weight, &height, &fps, &format);
    if (ret != 0) {
      AERROR << "Failed to hb_port_mapping.";
      return false;
    }
    AINFO << "camera i: " << i << ", pipeid: " << pipeid << ", video: " << video
          << ", width: " << weight << ", height: " << height << ", fps: " << fps
          << ", format: " << DateTypeMap[format];
    for (auto topic_config : camera_config->topic_configs()) {
      if (topic_config.video_index() == i) {
        find = true;
        raw_output_channel = topic_config.raw_output_channel();
        compressed_output_channel = topic_config.compressed_output_channel();
        frame_id = topic_config.frame_id();
        break;
      }
    }
    if (!find) {
      char tmp[256] = {0};
      (void)snprintf(tmp, sizeof(tmp), "/century/sensor/camera/video%d/image",
                     i);
      raw_output_channel = std::string(tmp);
      (void)snprintf(tmp, sizeof(tmp),
                     "/century/sensor/camera/video%d/image/compressed", i);
      compressed_output_channel = std::string(tmp);
      (void)snprintf(tmp, sizeof(tmp), "camera_video%d", i);
      frame_id = std::string(tmp);
    }

    auto raw_image_writer = CreateImageWriter(raw_output_channel);
    auto compressed_image_writer =
        CreateCompressedImageWriter(compressed_output_channel);
    std::shared_ptr<CCameraMgr> camera = std::make_shared<CCameraMgr>(
        pipeid, video, weight, height, fps, format, raw_image_writer,
        compressed_image_writer, frame_id, output_raw_);
    if (!camera->Init()) {
      AERROR << "Failed to do camera: " << i << " camera->Init().";
      return false;
    }
    cameras_.push_back(camera);
  }
  AINFO << "All cameras finished camera->Init().";

  for (int i = 0; i < cam_num; i++) {
    cameras_[i]->Start();
  }

  return true;
}

int TztekCamera::SensorSystem(const char *pCmd) {
  pid_t pid;
  struct sigaction sa, intr, quit;
  int status = 0;

  if (pCmd == NULL) {
    return 1;
  }

  sa.sa_handler = SIG_IGN;
  sa.sa_flags = 0;
  sigemptyset(&sa.sa_mask);
  if ((sigaction(SIGINT, &sa, &intr) < 0) ||
      (sigaction(SIGQUIT, &sa, &quit) < 0)) {
    return -1;
  }

  pid = vfork();
  if (pid == 0) {
    char achCmd[2048] = {0};
    strncpy(achCmd, pCmd, sizeof(achCmd) - 1);
    char *achArgv[4] = {(char *)"/bin/sh", (char *)"-c", achCmd, NULL};
    for (int nfd = 3; nfd < 4096; nfd++) {
      if (close(nfd) != 0) {
        break;
      }
    }
    (void)sigaction(SIGINT, &intr, (struct sigaction *)NULL);
    (void)sigaction(SIGQUIT, &quit, (struct sigaction *)NULL);

    int childRet = execv("/bin/sh", achArgv);
    if (childRet < 0) {
      printf("%s failed %d errno =%d  %s!\n", pCmd, childRet, errno,
             strerror(errno));
      _exit(childRet);
    }
  } else if (pid < 0) {
    return -1;
  } else {
    int n;

    do {
      n = waitpid(pid, &status, 0);
    } while (n == -1 && errno == EINTR);

    if (n != pid) {
      status = -1;
    }
  }

  if (sigaction(SIGINT, &intr, (struct sigaction *)NULL) ||
      sigaction(SIGQUIT, &quit, (struct sigaction *)NULL) != 0) {
    return -1;
  }
  return status;
}

int TztekCamera::ResetPower() {
  AINFO << "start reset power.";
  std::string cmd_prefix{"echo nvidia | sudo -S "};
  std::string cmd;

  cmd = std::string("echo 0 > /sys/class/tz_gpio/camera_0_1-power/value");
  std::string full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host0
  cmd = std::string("echo 0 > /sys/class/tz_gpio/camera_2_3-power/value");
  full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host2
  cmd = std::string("echo 0 > /sys/class/tz_gpio/camera_4_5-power/value");
  full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host3
  cmd = std::string("echo 0 > /sys/class/tz_gpio/camera_6_7-power/value");
  full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host3
  sleep(1);
  AINFO << "power is off.";
  cmd = std::string("echo 1 > /sys/class/tz_gpio/camera_0_1-power/value");
  full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host0
  cmd = std::string("echo 1 > /sys/class/tz_gpio/camera_2_3-power/value");
  full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host2
  cmd = std::string("echo 1 > /sys/class/tz_gpio/camera_4_5-power/value");
  full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host3
  cmd = std::string("echo 1 > /sys/class/tz_gpio/camera_6_7-power/value");
  full_cmd = cmd_prefix + cmd;
  std::system(full_cmd.data());  // cam host3
  sleep(1);
  AINFO << "power on.";
  return 0;
}
}  // namespace camera
}  // namespace drivers
}  // namespace century