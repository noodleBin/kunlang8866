# config.proto 配置精简裁定表

## 背景

本表用于推进 `proto/config.proto` 和 `conf/camera_video.pb.txt` 的配置精简。
当前目标平台为 J5/NVIDIA，其中 Tztek 对应 NVIDIA 侧实现；同时 x86 pipeline 需要保持
proto 生成代码和平台无关规则可编译。

本轮先做裁定，再按表从简单到复杂逐项推进。每项推进时先复核当前代码证据，再做最小变更。
涉及 proto schema、默认配置或运行行为变化时，状态先进入 `已修改待你验证`，等待目标环境验证。

## 状态模型

- `已修复`: 当前代码或配置已经满足要求，并且你已确认目标环境验证通过。
- `已修改待你验证`: 已完成代码/配置/文档修改，本地只能做静态或局部验证，等待目标环境验证。
- `迁移后待修`: 当前仍存在需要处理的配置项，后续按表推进。
- `迁移后可保留`: 当前字段仍有真实用途，或作为兼容路径暂时保留。
- `重复/合并`: 该字段语义已被其他配置覆盖，处理时并入对应项。

## 排序原则

1. 先处理没有任何运行读取路径、只存在于默认配置和 proto 的字段。
2. 再处理旧 USB/V4L2 通用相机控制字段。
3. 再处理 J5/NVIDIA 平台边界字段。
4. 最后处理仍在使用但语义需要重构的字段。

## 当前证据

- `TztekCamera::Init()` 当前读取 `platform_config`、`hb_j5dev_path`、`enabled_camera_mask`、
  `output_raw`、`compress_conf.jpeg_quality`、`v4l2_buffer_count`、`v4l2_timing_config`、
  `v4l2_thread_config`、`topic_configs`、`width`、`height`、`crop_mode`。
- 设备节点由 `hb_port_mapping()` 返回的 `video` 拼接为 `/dev/video%d`，不是来自
  `Config.camera_dev`。
- 采集 fps 和像素格式来自 `hb_port_mapping()`，不是来自 `Config.frame_rate` 或
  `Config.pixel_format`。
- x86 下 `camera_component_lib` 和 `tztek_camera_lib` 选择空源码/空依赖，但
  `proto/config.proto` 仍会生成并参与平台无关编译。

## 逐项裁定

| 顺序 | 配置项 | 当前发现 | 状态 | 推荐动作 |
|---:|---|---|---|---|
| 1 | `Config.camera_dev` | 当前设备节点由 VIN 映射的 `video` 决定，并在 V4L2 层拼成 `/dev/video%d`；代码不读取该字段；已从默认配置删除，proto 暂保留 | 已修改待你验证 | 后续统一 schema 迁移阶段废弃并 `reserved`；说明见 `docs/superpowers/specs/2026-04-17-proto-cleanup-camera_dev.md` |
| 2 | `Config.frame_id` 顶层 | 当前 frame id 来自 `topic_configs.frame_id`，顶层字段不读取；已从默认配置删除，proto 暂保留 | 已修改待你验证 | 后续统一 schema 迁移阶段废弃；说明见 `docs/superpowers/specs/2026-04-17-proto-cleanup-frame_id.md` |
| 3 | `Config.channel_name` | 当前 raw topic 来自 `topic_configs.raw_output_channel`，该字段不读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 4 | `CompressConfig.output_channel` | 当前压缩 topic 来自 `topic_configs.compressed_output_channel`，该字段不读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 5 | `CompressConfig.image_pool_size` | JPEG encoder 没有读取或使用该字段 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 6 | `Config.monochrome` | 当前没有灰度输出控制路径，代码不读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 7 | `Config.bytes_per_pixel` | 当前 raw `step` 和 JPEG input stride 已按协商像素格式计算，代码不读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 8 | `Config.output_type` / `OutputType` | 当前输出 encoding 来自协商 V4L2 pixel format，代码不读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 enum |
| 9 | `Config.io_method` / `IOMethod` | 当前实现固定走 V4L2 streaming + DMABUF，代码不读取 read/mmap/userptr 配置 | 迁移后待修 | 从默认配置删除；proto 后续废弃 enum |
| 10 | `Config.pixel_format` | 当前真实 pixel format 来自 `hb_port_mapping()` 返回值，代码不读取 string 字段 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 11 | `Config.frame_rate` | 当前 fps 来自 `hb_port_mapping()` 并传入 V4L2 `SetStreamFps()`，代码不读取该字段 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 12 | `Config.trigger_internal` | 当前触发模式不从 proto 读取；J5 触发配置在 VIN JSON 中 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 13 | `Config.trigger_fps` | 当前触发 fps 不从 proto 读取；J5 触发配置在 VIN JSON 中 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 14 | `Config.device_wait_ms` | 当前等待和超时参数已由 `v4l2_timing_config` 覆盖，该字段不读取 | 重复/合并 | 从默认配置删除；语义并入 `v4l2_timing_config` |
| 15 | `Config.spin_rate` | 当前不读取，旧 select/spin 语义已过时 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 16 | `Config.brightness` | 当前没有 V4L2 control 或 sensor control 读取路径 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 17 | `Config.contrast` | 当前没有 V4L2 control 或 sensor control 读取路径 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 18 | `Config.saturation` | 当前没有 V4L2 control 或 sensor control 读取路径 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 19 | `Config.sharpness` | 当前没有 V4L2 control 或 sensor control 读取路径 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 20 | `Config.gain` | 当前没有 V4L2 control 或 sensor control 读取路径；J5/NVIDIA 增益应走平台 sensor/VIN 配置 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 21 | `Config.auto_focus` | 当前多路车载相机链路没有读取该字段 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 22 | `Config.focus` | 当前多路车载相机链路没有读取该字段 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 23 | `Config.auto_exposure` | 当前曝光控制不从 proto 读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 24 | `Config.exposure` | 当前曝光控制不从 proto 读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 25 | `Config.auto_white_balance` | 当前白平衡控制不从 proto 读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 26 | `Config.white_balance` | 当前白平衡控制不从 proto 读取 | 迁移后待修 | 从默认配置删除；proto 后续废弃 |
| 27 | `PlatformConfig.generic_config_path` | 当前明确目标平台为 J5/NVIDIA，没有 generic backend 代码路径 | 迁移后待修 | proto 后续废弃；避免保留虚假扩展入口 |
| 28 | `PlatformConfig.nvidia_config_path` | Tztek 对应 NVIDIA 侧能力，字段应作为 NVIDIA backend 配置入口保留；当前尚无读取路径 | 迁移后待修 | 保留 proto；默认配置暂不写；后续补 NVIDIA 读取路径 |
| 29 | `PlatformConfig.horizon_j5dev_path` | 当前 J5 VIN JSON 主路径，代码真实读取 | 迁移后可保留 | 保留；后续可迁移为更准确的 `j5_vin_config_path`，旧字段 fallback |
| 30 | `Config.hb_j5dev_path` | legacy fallback，当前仍被兼容读取 | 迁移后可保留 | 先保留；确认部署全量迁移到 `platform_config` 后废弃 |
| 31 | `PlatformConfig.camera_power_gpio.power_value_paths` | 当前真实读取，用于 reset power | 迁移后可保留 | 保留；后续可按 J5/NVIDIA 平台拆分是否需要 |
| 32 | `Config.output_raw` | 当前真实读取，控制 raw writer 创建 | 迁移后可保留 | 保留；后续迁入 `OutputConfig` |
| 33 | `Config.width` | 当前真实读取，但语义是输出宽度，不是采集宽度 | 迁移后待修 | 保留兼容；后续迁为 `output_config.width` |
| 34 | `Config.height` | 当前真实读取，但语义是输出高度，不是采集高度 | 迁移后待修 | 保留兼容；后续迁为 `output_config.height` |
| 35 | `Config.crop_mode` / `CropMode` | 当前真实读取，只影响 1280x720 输出缩放裁剪 | 迁移后可保留 | 保留；后续迁入 `OutputConfig` |
| 36 | `CompressConfig.jpeg_quality` | 当前真实读取，控制 JPEG 质量 | 迁移后可保留 | 保留；后续迁入 `CompressionConfig` |
| 37 | `TopicConfig.video_index` | 当前用于匹配 VIN video、pipe id 或循环序号 | 迁移后可保留 | 保留；后续明确匹配规则，减少三种 index 混用 |
| 38 | `TopicConfig.raw_output_channel` | 当前真实读取 | 迁移后可保留 | 保留 |
| 39 | `TopicConfig.compressed_output_channel` | 当前真实读取，空值可禁用压缩 writer | 迁移后可保留 | 保留 |
| 40 | `TopicConfig.frame_id` | 当前真实读取 | 迁移后可保留 | 保留 |
| 41 | `Config.enabled_camera_mask` | 当前真实读取，决定启用通道 | 迁移后可保留 | 保留 |
| 42 | `Config.v4l2_buffer_count` | 当前真实读取，控制 V4L2 buffer 数量 | 迁移后可保留 | 保留 |
| 43 | `V4l2TimingConfig.*` | 当前真实读取，控制 init、clear、poll、retry timing | 迁移后可保留 | 保留 |
| 44 | `V4l2ThreadConfig.*` | 当前真实读取，控制实时调度和 CPU 亲和性 | 迁移后可保留 | 保留 |
| 45 | `ChannelCpuAffinity.channel/core` | 当前真实读取，用于 per-channel CPU 绑定 | 迁移后可保留 | 保留 |

## 推进规则

- 每次只推进当前表中最靠前的 `迁移后待修` 或 `重复/合并` 项。
- 本表后续输出的 specs/plans 均使用 `proto-cleanup` 前缀，避免与历史 camera 架构问题文档混淆：
  - specs: `docs/superpowers/specs/YYYY-MM-DD-proto-cleanup-<item>.md`
  - plans: `docs/superpowers/plans/YYYY-MM-DD-proto-cleanup-<item>-plan.md`
- 默认先清理 `conf/camera_video.pb.txt` 中的无效字段，降低误导；proto 字段废弃放到统一 schema 迁移阶段。
- 对 proto 字段做删除或 `reserved` 前，必须再次做代码和配置全局检索。
- x86 pipeline 至少需要验证 `proto/config.proto` 生成路径不被破坏；aarch64 目标环境验证仍由你在指定 Docker/远程环境完成。
