---
name: century-perception-lidar-only
description: Use when user asks to "run lidar perception", "start perception_lidar", "debug lidar dag/config/channel", or verify lidar perception output topics in Century.
---

# Century Perception Lidar-Only

Use this skill for lidar-only perception startup, validation, and staged troubleshooting.

## Source-of-Truth Paths

- `modules/perception/production/launch/perception_lidar.launch`
- `modules/perception/production/dag/dag_streaming_perception_lidar.dag`
- `modules/perception/production/conf/perception/lidar/config_manager.config`
- `modules/perception/production/conf/perception/lidar/*.pb.txt`
- `scripts/perception.sh`
- `scripts/topics.txt`

## Standard Workflow

1. Build with required mode:
   - `bash century.sh build_opt_gpu` (preferred for CUDA path)
   - `bash century.sh build` (dry-run or CPU path)
2. Start lidar-only perception:
   - `cyber_launch start /century/modules/perception/production/launch/perception_lidar.launch`
3. Verify output topic:
   - `/century/perception/obstacles`
4. Cross-check channel wiring:
   - input/output channel names in lidar config vs DAG readers/writers
5. If no output, isolate failure stage:
   - sensor input missing
   - config manager path mismatch
   - DAG component starts but writer topic mismatch

## Guardrails

- Do not assume sensor topic names; read active lidar config files.
- Do not modify launch or dag files unless explicitly requested.
- Prefer canonical launch path over ad-hoc mainboard command.

## Output Format For User

1. Launch command and dag path
2. Effective lidar config file(s)
3. Observed input/output channels
4. First failing stage and evidence
5. Exact next command or file to fix
