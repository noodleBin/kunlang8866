---
name: century-perception-pipeline-runner
description: Use when user asks to "start perception", "run lidar pipeline", "check perception topic", or debug dag/config/channel issues under modules/perception. Also use for perception-to-planning output validation.
---

# Century Perception Pipeline Runner

Use this skill to run and diagnose Century perception quickly with reproducible command paths.

## Source-of-Truth Files

- `scripts/perception.sh`
- `modules/perception/production/launch/perception_all.launch`
- `modules/perception/production/launch/perception_lidar.launch`
- `modules/perception/production/dag/dag_streaming_perception_lidar.dag`
- `modules/perception/production/conf/perception/lidar/config_manager.config`
- `scripts/topics.txt`
- `docs/howto/how_to_run_perception_module_on_your_local_computer.md`

## Primary Outputs to Validate

- `/century/perception/obstacles`
- `/century/perception/traffic_light`

## Standard Workflow

### 1) Confirm runtime prerequisites

- If GPU path is required, use `./century.sh build_opt_gpu`.
- If CPU path is enough for dry run, use `./century.sh build`.
- Source Cyber env where needed: `source /century/cyber/setup.bash`.

### 2) Start perception safely

Prefer the canonical script first:

- `bash scripts/perception.sh`

If debugging lidar-only, use explicit launch:

- `cyber_launch start /century/modules/perception/production/launch/perception_lidar.launch`

### 3) Trace config and DAG wiring

Read selected lidar config and capture:

- detector/component names
- `input_channel_name`
- `output_channel_name`
- map or ROI switches (`enable_hdmap`, map manager toggles)

Cross-check those names against readers/writers in:

- `modules/perception/production/dag/dag_streaming_perception_lidar.dag`

### 4) Validate end-to-end channels

- Verify perception topics exist and update while replaying data.
- Verify planning-facing output topic `/century/perception/obstacles` is alive.
- When needed, use helper tools:
  - `modules/tools/perception/print_perception.py`
  - `modules/tools/perception/replay_perception.py`

### 5) Diagnose by failure stage

- **No input cloud**: check lidar driver and sensor topic names.
- **DAG starts but no output**: check config-manager file paths and channel mismatch.
- **Output exists but planning misses it**: check topic contract and consumer subscription.
- **Intermittent drop**: inspect process guard logic in `node_monitor.sh`.

## Guardrails

- Do not invent topic names; read from `scripts/topics.txt` and active config.
- Do not modify production launch scripts unless user asks for code changes.
- Prefer reproducible script paths over ad-hoc commands.

## Related Skills

- `century-perception-lidar-only` for lidar-specific bring-up.
- `century-perception-camera-only` for camera obstacle pipeline only.
- `century-perception-trafficlight-only` for traffic-light-only pipeline.

## Output Format For User

Always return:

1. Launch command and DAG path used
2. Effective lidar config files and key channels
3. Observed flow: input -> intermediate -> output
4. First failing stage and concrete evidence
5. Exact next command or file to change
