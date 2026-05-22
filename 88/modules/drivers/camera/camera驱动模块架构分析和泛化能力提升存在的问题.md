概述
本文档详细分析了相机驱动模块中的所有硬编码值，这些硬编码值限制了其在不同硬件平台和配置间的通用性和可移植性。
1. 平台特定硬编码
1.1 用户权限管理
**文件：`tztek_camera.cc`**
- 位置：第54行、第212行
- 硬编码内容：`"echo nvidia | sudo -S "`
- 问题：硬编码的"nvidia"用户名
- 影响：仅适用于NVIDIA Tegra平台
- 解决方案：通过protobuf或环境变量使用户名可配置

// 当前硬编码代码
char achCmd[2048];
sprintf(achCmd, "echo nvidia | sudo -S chown -R nvidia:nvidia %s", device_path);

// 建议改进
const char* user_name = GetConfig().user_name().c_str() || "nvidia";
char achCmd[2048];
sprintf(achCmd, "echo %s | sudo -S chown -R %s:%s %s",
        user_name, user_name, user_name, device_path);

1.2 GPIO路径硬编码

**文件：`tztek_camera.cc`**
- 位置：第58行、第215-239行
- 硬编码GPIO路径：
- "/sys/class/tz_gpio/camera_0_1-power/value"
- "/sys/class/tz_gpio/camera_2_3-power/value"
- "/sys/class/tz_gpio/camera_4_5-power/value"
- "/sys/class/tz_gpio/camera_6_7-power/value"
- 问题：特定硬件平台专用
- 解决方案：创建可配置GPIO路径的平台抽象层

// 建议的protobuf配置消息
message CameraPowerGPIO {
  repeated string power_path = 1;
  repeated string reset_path = 2;
}

message PlatformConfig {
  CameraPowerGPIO nvidia_gpio = 1;
  CameraPowerGPIO horizon_gpio = 2;
  CameraPowerGPIO generic_gpio = 3;
}

1.3 设备路径硬编码

**文件：`tztek_camera.cc`**
- 位置：第55-56行
- 硬编码：`"/dev/ttyTHS1"`
- 问题：串口设备特定于某些平台
- 解决方案：使设备路径可配置

1.4 用户和组名称

**文件：`tztek_camera.cc`**
- 位置：第180-183行
- 硬编码：组"video"和"render"
- 问题：假设Linux组存在
- 解决方案：使组名称可配置并提供回退机制

2. 配置硬编码

2.1 默认分辨率和帧率

**文件：`conf/camera_video.pb.txt`**
- 位置：第6-8行
- 硬编码：`width: 1920`，`height: 1536`，`frame_rate: 20`
- 问题：固定的分辨率和帧率
- 解决方案：使这些参数每个相机通道可配置

// 建议的配置
message CameraChannel {
  string device_path = 1;
  uint32 width = 2;
  uint32 height = 3;
  uint32 frame_rate = 4;
  string pixel_format = 5;  // 例如："YUYV", "UYVY", "RAW"
  uint32 buffer_count = 6;
  uint32 jpeg_quality = 7;
}

2.2 设备索引位掩码

**文件：`tztek_camera.cc`**
- 位置：第71行
- 硬编码：`config_index = 63`（二进制：111111）
- 问题：固定为6个相机
- 解决方案：根据实际相机数量使位掩码可配置

// 当前：固定6个相机
config_index = 63;  // 111111 二进制

// 建议：从配置获取
config_index = 0;
for (int i = 0; i < config.channel_size(); i++) {
  if (config.channel(i).enabled()) {
    config_index |= (1 << i);
  }
}

3. 主题命名硬编码

3.1 主题路径模板

**文件：`tztek_camera.cc`**
- 位置：第122-129行
- 硬编码：`"/century/sensor/camera/video%d/image"`
- 问题：固定的主题命名模式
- 解决方案：使主题前缀和命名模式可配置

// 建议的配置
message TopicConfig {
  string base_topic = 1;  // "/century/sensor/camera"
  string image_topic_template = 2;  // "/video%d/image"
  string compressed_topic_template = 3;  // "/video%d/image/compressed"
}

3.2 默认主题名称

**文件：`conf/camera_video.pb.txt`**
- 硬编码主题名称：
- "/century/sensor/camera/video0/image"
- "/century/sensor/camera/video0/image/compressed"
- "/century/sensor/camera/right_rear/image"
- "/century/sensor/camera/rear/image"
- "/century/sensor/camera/left_front/image"
- "/century/sensor/camera/left_rear/image"
- "/century/sensor/camera/right_front/image"
- "/century/sensor/camera/front/image"
- 问题：固定的映射关系
- 解决方案：允许每个相机通道自定义主题名称

// 建议的配置
message CameraChannel {
  string name = 1;  // 例如："front_12mm", "left_fisheye"
  string image_topic = 2;
  string compressed_topic = 3;
  string raw_topic = 4;  // 可选
}

4. V4L2实现硬编码

4.1 缓冲区配置

**文件：`v4l2cam.cc`**
- 位置：第59行
- 硬编码：`V4L2_BUFFER_LENGHT = 2`
- 问题：固定的缓冲区数量
- 解决方案：使缓冲区数量可配置

4.2 默认像素格式

**文件：`v4l2cam.cc`**
- 位置：第60行
- 硬编码：`V4L2_PIX_FMT_YUYV`
- 问题：固定格式
- 解决方案：从protobuf使格式可配置

4.3 超时值

**文件：`v4l2cam.cc`**
- 多个位置：
- 第124行：usleep(1000 * 1000) = 1秒延迟
- 第189行：tv.tv_usec = 20 * 1000 = 20ms选择超时
- 第364行：tv.tv_usec = 100 * 1000 = 100ms清除缓冲区超时
- 问题：硬编码的超时值不适用于所有场景
- 解决方案：使所有超时值可配置

// 建议的超时配置
message TimeoutConfig {
  uint32 device_init_ms = 1;  // 1000
  uint32 select_timeout_us = 2;  // 20000
  uint32 buffer_clear_timeout_us = 3;  // 100000
  uint32 thread_sleep_ms = 4;  // 5
  uint32 acquisition_timeout_ms = 5;  // 100
}

4.4 线程优先级和休眠

**文件：`v4l2cam.cc`**
- 位置：第392行
- 硬编码：`param.sched_priority = 90`
- 位置：第401行、第411行
- 硬编码：`usleep(5 * 1000)` = 5ms
- 问题：固定的线程配置
- 解决方案：使线程优先级和休眠时间可配置

5. 图像处理硬编码

5.1 JPEG质量

**文件：`mgr_camera.cc`**
- 位置：第81行
- 硬编码：JPEG质量`90`
- 问题：固定的质量设置
- 解决方案：使每个通道的质量可配置

// 建议的每通道配置
message CameraChannel {
  uint32 jpeg_quality = 7;  // 1-100
  bool enable_compression = 8;
  uint32 image_pool_size = 9;  // 100
}

5.2 缓冲区大小计算

**文件：`camera_type.h`**
- 位置：第33行
- 硬编码：`unsigned char image[2 * 4096 * 2160]`
- 问题：固定为8K分辨率，对于小相机浪费内存
- 解决方案：根据实际分辨率计算或可配置

// 建议的动态计算
class CameraBuffer {
public:
  CameraBuffer(uint32 width, uint32 height, const string& format) {
    // 根据格式和分辨率计算
    if (format == "YUYV") {
      size_ = width * height * 2;  // 每像素2字节
    } else if (format == "RAW") {
      size_ = width * height * 3;  // RGB每像素3字节
    }
    buffer_ = new uint8_t[size_];
  }

  ~CameraBuffer() { delete[] buffer_; }
  uint8_t* data() { return buffer_; }
  size_t size() { return size_; }
};

6. 魔数和常量

6.1 文件描述符限制

**文件：`tztek_camera.cc`**
- 位置：第175行
- 硬编码：`for (int nfd = 3; nfd < 4096; nfd++)`
- 问题：固定的文件描述符限制
- 解决方案：使用系统限制或可配置

6.2 命令缓冲区大小

**文件：`tztek_camera.cc`**
- 位置：第172行
- 硬编码：`char achCmd[2048]`
- 问题：固定的缓冲区大小
- 解决方案：使用动态分配或可配置大小

6.3 跳过数量

**文件：`mgr_camera.cc`**
- 位置：第19行
- 硬编码：`SKIP_NUM = 1`
- 问题：未使用的常量，应该删除

7. 硬件特定硬编码

7.1 Horizon J5dev路径

**文件：`conf/camera_video.pb.txt`**
- 位置：第15行
- 硬编码：`"hb_j5dev_path: "/century/modules/drivers/camera/conf/hb_j5dev.json"`
- 问题：Horizon配置的固定路径
- 解决方案：使路径可配置

// 建议的配置
message PlatformConfig {
  string horizon_j5dev_path = 1;
  string nvidia_config_path = 2;
  string generic_config_path = 3;
}

7.2 图像处理常量

**文件：`mgr_camera.cc`**
- 位置：第45行
- 硬编码：`2 * dwWidth`（每行字节数）
- 问题：假设YUYV格式
- 解决方案：根据实际像素格式计算

// 建议的格式感知计算
size_t CalculateStepSize(uint32 width, const string& format) {
  if (format == "YUYV" || format == "UYVY") {
    return width * 2;  // 每像素2字节
  } else if (format == "RGB" || format == "RAW") {
    return width * 3;  // 每像素3字节
  }
  return width * 2;  // 默认回退
}

8. 配置文件路径硬编码

8.1 硬编码JSON路径

**文件：`tztek_camera.cc`**
- 位置：多处加载JSON配置的地方
- 问题：配置文件的固定路径
- 解决方案：使用环境变量或可配置路径

// 建议的配置加载
string config_path = GetConfigPath();
if (config_path.empty()) {
  config_path = "/etc/camera/camera_config.json";
}

// 或使用环境变量
char* env_path = getenv("CAMERA_CONFIG_PATH");
if (env_path) {
  config_path = env_path;
}

重构优先级建议

高优先级（关键通用性）
1. 使用户名和组名可配置
2. 通过平台抽象层使GPIO路径可配置
3. 使主题命名模式可配置
4. 使分辨率、帧率和像素格式可配置
中优先级（重要灵活性）
1. 使超时值可配置
2. 使缓冲区数量可配置
3. 使JPEG质量可配置
4. 添加缺失配置文件的错误处理
低优先级（锦上添花）
1. 基于分辨率的动态缓冲区大小
2. 可配置的线程优先级
3. 平台特定的配置加载
4. 所有配置选项的文档
实施策略

1. 第一阶段：向protobuf模式添加可配置选项
2. 第二阶段：更新代码以使用可配置值
3. 第三阶段：添加平台检测和回退机制
4. 第四阶段：添加配置值验证
5. 第五阶段：更新文档和示例