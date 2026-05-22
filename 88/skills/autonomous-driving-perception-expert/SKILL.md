---
name: autonomous-driving-perception-expert
description: 当用户提到 Century/Apollo 的 perception 感知模块开发、联调、排障、性能优化、配置/DAG/topic 对齐、模型切换、线上维护时，必须使用本技能。适用于“感知没输出”“topic 不通”“如何启动 lidar/camera/trafficlight 感知”“如何做日常维护”等场景，即使用户没有明确说“使用技能”。
license: MIT
compatibility: opencode
metadata:
  audience: autonomous-driving-engineers
  language: zh-CN
  framework: century
---

# Autonomous Driving Perception Expert

用于 Century 项目 `modules/perception` 的日常开发、维护、联调和故障定位。

## 目标

- 快速定位感知链路故障（输入 -> 处理 -> 输出）。
- 用统一流程做构建、启动、验证、回归，减少“能跑但不可复现”。
- 在不破坏生产配置的前提下完成改动与验证。

## Source-of-Truth 路径

- `modules/perception/`
- `scripts/perception.sh`
- `scripts/topics.txt`
- `modules/perception/production/launch/perception_all.launch`
- `modules/perception/production/launch/perception_lidar.launch`
- `modules/perception/production/launch/perception_camera.launch`
- `modules/perception/production/launch/perception_trafficlight.launch`
- `modules/perception/production/dag/dag_streaming_perception_lidar.dag`
- `modules/perception/production/dag/dag_streaming_perception_camera.dag`
- `modules/perception/production/dag/dag_streaming_perception_trafficlights.dag`
- `modules/perception/production/conf/perception/lidar/config_manager.config`
- `modules/perception/production/conf/perception/lidar/*.pb.txt`
- `modules/perception/production/conf/perception/camera/*.pb.txt`
- `docs/howto/how_to_run_perception_module_on_your_local_computer.md`

## 标准工作流（必须按顺序执行）

### 1) 明确范围与运行模式

- 判定任务是 `lidar` / `camera` / `trafficlight` / 全感知链路。
- 判定是否需要 GPU 路径：
  - 需要 CUDA 推理：`bash century.sh build_opt_gpu`
  - 常规开发：`bash century.sh build`

### 2) 先读配置再启动

- 从 active 配置读取输入输出 topic，不允许凭记忆写死。
- 确认 launch、dag、component config 三者的 channel 一致。

### 3) 启动最小可复现链路

- 优先使用官方脚本：`bash scripts/perception.sh`
- 或按子链路启动：
  - lidar: `cyber_launch start /century/modules/perception/production/launch/perception_lidar.launch`
  - camera: `cyber_launch start /century/modules/perception/production/launch/perception_camera.launch`
  - trafficlight: `cyber_launch start /century/modules/perception/production/launch/perception_trafficlight.launch`

### 4) 分阶段验证

- 输入阶段：传感器 topic 是否存在且持续更新。
- 处理阶段：DAG 组件是否起起停停、是否有配置加载异常。
- 输出阶段：
  - `/century/perception/obstacles`
  - `/century/perception/traffic_light`
- 下游阶段：planning/control 是否订阅到并消费输出。

### 5) 故障分层定位

- 无输入：驱动/topic 名称不一致。
- 有输入无输出：config 路径或 DAG wiring 断裂。
- 有输出下游无消费：topic 契约不一致。
- 间歇性掉帧：资源瓶颈、时序、进程守护。

### 6) 修改与回归

- 仅改与任务直接相关的文件。
- 改完必须做：
  - 相关模块 build
  - 相关测试/回放验证
  - 输出 topic 存活与正确性验证

## 日常维护清单（Quick Checklist）

每次日常维护至少覆盖：

1. 启动命令可复现（记录命令）。
2. 关键配置文件路径可定位（记录路径）。
3. 输入/输出 topic 可观测（记录证据）。
4. 首个失败阶段已定位（不是“感觉是这里”）。
5. 给出下一步可执行动作（命令或文件级改动）。

## 代码改动守则

- 先匹配项目现有模式，再改代码。
- 不扩大改动范围，不顺手改无关模块。
- 不删除测试来“消灭失败”。
- 不用类型抑制逃避问题（如 `as any`/`@ts-ignore` 等）。
- C++ 改动优先保证：可读、可测、可回滚。

## 常用诊断动作

- 对照 `scripts/topics.txt` 核对 topic 命名。
- 交叉比对 `launch`、`dag`、`component pb.txt` 的 reader/writer。
- 需要时使用：
  - `modules/tools/perception/print_perception.py`
  - `modules/tools/perception/replay_perception.py`

## 输出格式（对用户汇报）

每次输出固定包含：

1. 本次目标与范围（lidar/camera/trafficlight/全链路）
2. 实际使用的启动命令与关键文件路径
3. 观测到的链路状态（输入 -> 处理 -> 输出 -> 下游）
4. 首个失败阶段 + 证据
5. 已实施修复与验证结果
6. 下一步最小动作（1 条命令或 1 个文件）

## 何时联动其他技能

- 仅 lidar 问题：联动 `century-perception-lidar-only`
- 仅 camera 问题：联动 `century-perception-camera-only`
- 仅 trafficlight 问题：联动 `century-perception-trafficlight-only`
- 感知全链路启动/联调：联动 `century-perception-pipeline-runner`
- 项目构建异常：联动 `century-build-workflow`

## 禁止事项

- 未经要求修改生产 launch/dag 全局行为。
- 凭经验猜 topic 名称后直接下结论。
- 只给“建议”不落地执行与验证。
