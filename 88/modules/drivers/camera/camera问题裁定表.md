# camera 问题裁定表

日期：2026-04-15

基准文档：`camera驱动模块架构分析和泛化能力提升存在的问题.md`

审计范围：
- 当前目录下的现版本代码
- 重点核查 `tztek/tztek_camera.cc`、`tztek/v4l2cam.cc`、`tztek/mgr_camera.cc`、`tztek/inc/camera_type.h`、`proto/config.proto`、`conf/camera_video.pb.txt`

判定状态说明：
- `已修复`：文档中的问题在当前代码中已消失
- `部分修复`：问题的主要方向已处理，但仍保留泛化或配置化缺口
- `迁移后待修`：旧行号已失效，但同类问题在新位置仍存在，建议后续修复
- `已修改待你验证`：代码已按方案修改，但尚未经过你的真实环境验证
- `迁移后可保留`：问题形态迁移后仍可见，但当前实现可接受，短期不建议改
- `重复/合并`：该问题在当前代码里与另一条问题本质相同，后续应合并处理

结论总览：
- 已修复：16 项
- 部分修复：0 项
- 迁移后待修：0 项
- 已修改待你验证：0 项
- 迁移后可保留：3 项
- 重复/合并：1 项

## 逐项裁定

| ID | 文档原问题 | 当前代码证据 | 当前判定 | 建议动作 |
| --- | --- | --- | --- | --- |
| 1.1 | `nvidia` 用户名硬编码，`sudo -S` 权限处理固定 | 当前代码中检索 `nvidia`、`sudo -S` 无匹配；`tztek/tztek_camera.cc` 已改为直接写 sysfs 节点 | 已修复 | 不再按旧方案修复；后续只需关注现有 `ResetPower()` 的平台抽象 |
| 1.2 | GPIO 路径硬编码 | 已在 `proto/config.proto` 的 `PlatformConfig` 中新增 `camera_power_gpio.power_value_paths`；`conf/camera_video.pb.txt` 显式写入默认 4 个路径；`tztek/platform_config.cc` 新增默认路径和配置解析 helper；`tztek/tztek_camera.cc` 已改为解析配置路径后传入 `ResetPower(...)`，不再直接依赖固定 `kPowerNodes`；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-16-gpio-power-path-config-fix.md` |
| 1.3 | `/dev/ttyTHS1` 设备路径硬编码 | 当前代码中检索 `ttyTHS1` 无匹配；设备打开逻辑为 `tztek/v4l2cam.cc:131-133` 通过 `video_index` 生成 `/dev/video%d` | 已修复 | 不按旧问题继续修；后续若要提升泛化，单独评估 `/dev/video%d` 是否需要进一步抽象 |
| 1.4 | Linux 用户组 `video`、`render` 硬编码 | 当前代码中检索 `video`/`render` 的旧权限处理逻辑无匹配；不存在旧版 chown/chgrp 路径 | 已修复 | 不继续追旧问题 |
| 2.1 | 默认分辨率、帧率固定，且应支持每通道配置 | `proto/config.proto` 已支持 `width`/`height`/`frame_rate` 配置，`conf/camera_video.pb.txt` 中的 `1920x1536 @ 20fps` 属于默认部署参数；当前 `tztek/mgr_camera.cc` 仍限制输出模式为 `1920x1536` 或 `1280x720`，并要求采集分辨率为 `1920x1536`，这是当前处理链路能力约束 | 迁移后可保留 | 已确认采用方案 A：不改代码。该项已通过配置文件参数化，不视为传统意义的硬编码；采集/输出模式泛化如有新需求，后续单独立项。说明见 `docs/superpowers/specs/2026-04-17-resolution-frame-rate-config-keep.md` |
| 2.2 | 相机使能位掩码固定为 6 路 | 已新增 `proto/config.proto` 的 `enabled_camera_mask = 36 [default = 63]` 和 `conf/camera_video.pb.txt` 的显式默认值；`tztek/enabled_camera_mask.cc` 已提供 mask 校验和通道展开 helper；`tztek/tztek_camera.cc` 已改为从配置读取 mask 后展开 `channel_indices`，不再使用固定 `kDefaultEnableMask`；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-17-enabled-camera-mask-fix.md` |
| 3.1 | 主题路径模板硬编码 | `proto/config.proto` 已提供 `topic_configs`，默认配置也已显式配置 6 路 topic；`tztek/tztek_camera.cc` 中缺失 topic 或部分字段为空时仍保留 `"/century/sensor/camera/video%d/..."`、`"camera_video%d"` 回退模板 | 迁移后可保留 | 已确认采用方案 A：不改代码。当前显式配置能力满足使用要求，回退模板作为兼容兜底保留；说明见 `docs/superpowers/specs/2026-04-16-topic-fallback-template-keep.md` |
| 3.2 | 默认主题映射关系固定 | `conf/camera_video.pb.txt:34-74` 的 6 路 topic 仍是固定部署映射；但 schema 已支持逐项自定义，见 `proto/config.proto:62-66` | 迁移后可保留 | 已确认这是默认部署样例配置，不视为代码层硬编码缺陷；说明见 `docs/superpowers/specs/2026-04-15-topic-default-mapping-keep.md`，后续不作为代码修复项推进 |
| 4.1 | V4L2 缓冲区数量固定 | 已新增 `proto/config.proto` 的 `v4l2_buffer_count = 33 [default = 2]`、`conf/camera_video.pb.txt` 的显式默认值、`tztek/tztek_camera.cc` 的解析与非法值回退、`tztek/mgr_camera.cc`/`tztek/v4l2cam.cc` 的运行时透传；`V4l2Camera` 已改为用 `buffer_count_` 驱动 buffer 申请、QBUF、DQBUF、清理与释放；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-16-v4l2-buffer-count-config-fix.md` |
| 4.2 | 默认像素格式固定为 `YUYV` | 当前 `tztek` 链路已支持 `RAW / YUYV / UYVY` 三种格式映射；格式来源于 `hb_port_mapping()` 返回的 `format`，并经 `VIDIOC_S_FMT/VIDIOC_G_FMT` 完成设置与协商。`YUYV` 默认值仅作为未知格式的兜底，不再构成“只支持 YUYV”的功能限制 | 已修复 | 当前问题按“已支持不同像素格式切换”收口；后续若需要继续治理未知格式兜底策略，可作为新议题单独讨论 |
| 4.3 | V4L2 超时值硬编码 | 已新增 `proto/config.proto` 的 `V4l2TimingConfig` 和 `conf/camera_video.pb.txt` 的默认配置；`tztek/tztek_camera.cc` 已解析并透传 `V4l2TimingOptions`；`tztek/v4l2cam.cc` 中 `1000ms` 初始化等待、`100ms` 清空超时、首次采集前 `5ms`、`StartCapture()` 失败 `20ms`、抓帧失败 `5ms`、`poll` 上下限 `50/500ms` 均已改为配置项；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-16-v4l2-timing-config-fix.md` |
| 4.4 | 线程优先级和 CPU 亲和性固定 | 已新增 `proto/config.proto` 的 `V4l2ThreadConfig` 和 `conf/camera_video.pb.txt` 的默认配置；`tztek/tztek_camera.cc` 已解析并透传 `V4l2ThreadOptions`；`tztek/v4l2cam.cc` 中实时调度开关、目标优先级、CPU 亲和性开关、默认 core 数和 per-channel core 映射均已配置化；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-16-v4l2-thread-config-fix.md` |
| 5.1 | JPEG 质量固定为 90 | 已新增 `proto/config.proto:60` 的 `jpeg_quality`、`conf/camera_video.pb.txt:32` 的默认值、`tztek/tztek_camera.cc:158-166` 的解析与非法值回退、`tztek/mgr_camera.cc:247` 的运行时接入；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-15-jpeg-quality-config-fix.md` |
| 5.2 | 静态图像缓冲区按 `2 * 4096 * 2160` 固定分配 | 已从 `tztek/inc/camera_type.h` 删除 `TImageWrap` 及其固定大数组定义；你已确认测试机上采用同等方案后编译通过且程序可运行 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-15-camera-type-buffer-removal-fix.md` |
| 6.1 | 文件描述符上限 `4096` 魔数 | 当前代码中检索 `4096; nfd` 无匹配，旧逻辑已不存在 | 已修复 | 不继续追旧问题 |
| 6.2 | 命令缓冲区 `char achCmd[2048]` 魔数 | 当前代码中检索 `achCmd[2048]` 无匹配，旧命令拼接逻辑已不存在 | 已修复 | 不继续追旧问题 |
| 6.3 | `SKIP_NUM = 1` 未使用常量 | 当前代码中检索 `SKIP_NUM` 无匹配 | 已修复 | 不继续追旧问题 |
| 7.1 | `hb_j5dev_path` 为固定 Horizon 路径 | 已新增 `proto/config.proto` 的 `PlatformConfig`、`conf/camera_video.pb.txt` 的 `platform_config.horizon_j5dev_path`，以及 `tztek/tztek_camera.cc` 中“新字段优先、旧字段回退”的兼容解析逻辑；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-15-platform-config-j5dev-fix.md` |
| 7.2 | 图像处理按 `2 * width` 假设 YUYV 步长 | 已新增 `pixel_format_utils` 统一根据 V4L2 像素格式计算 bytes-per-pixel 和 packed stride；`tztek/mgr_camera.cc` 的 raw `step` 和 JPEG/缩放 `input_stride` 均已改为使用 `CalculatePackedImageStride(...)`，并保留 `data_len / height` 的 padding 修正逻辑；你已确认真实环境验证通过 | 已修复 | 修改说明见 `docs/superpowers/specs/2026-04-16-pixel-stride-helper-fix.md` |
| 8.1 | JSON 配置路径硬编码 | 当前代码未在 `tztek/tztek_camera.cc` 内部写死 JSON 路径，而是通过 `camera_config->hb_j5dev_path()` 读取，见 `tztek/tztek_camera.cc:127-151`；其本质与 7.1 相同 | 重复/合并 | 与 7.1 合并处理，不单独立项修复 |

## 审计备注

### 1. 当前最值得优先进入修复列表的事项

当前裁定表内已无 `迁移后待修` 项。

### 2. 当前代码里额外暴露出的相关限制

这些点不一定与原文一一对应，但在后续逐项修复时应一并关注：

- `tztek/mgr_camera.cc:204-209` 只允许输出 `1920x1536` 或 `1280x720`
- `tztek/mgr_camera.cc:225-229` 要求实际采集分辨率固定为 `1920x1536`

### 3. 本轮审计原则

- 只根据当前代码证据下结论，不沿用旧文档行号
- 已从运行路径消失的问题，直接判定为已修复，不为了“对齐旧文档”而重复立项
- 配置文件中的部署默认值，不等同于代码层硬编码；只有当 schema 本身不足或代码存在固定回退逻辑时，才判为仍需修复
- 初始裁定基于静态审计；逐项修复后的真实环境验证结果以对应问题说明和你的验证结论为准
