---
name: century-perception-trafficlight-only
description: Use when user asks to "run traffic light perception", "start perception_trafficlight", "check traffic light topic", or debug traffic-light dag/config/topic in Century.
---

# Century Perception TrafficLight-Only

Use this skill for traffic-light-only pipeline startup, validation, and troubleshooting.

## Source-of-Truth Paths

- `modules/perception/production/launch/perception_trafficlight.launch`
- `modules/perception/production/dag/dag_streaming_perception_trafficlights.dag`
- `modules/perception/production/conf/perception/camera/trafficlights_perception_component.config`
- `modules/perception/production/conf/perception/perception_common.flag`
- `docs/howto/how_to_run_perception_module_on_your_local_computer.md`

## Standard Workflow

1. Start traffic-light-only pipeline:
   - `cyber_launch start /century/modules/perception/production/launch/perception_trafficlight.launch`
2. Replay bag for runtime validation:
   - `cyber_recorder play -f /century/data/bag/anybag -r 0.2`
3. Verify configured output topic:
   - `/century/perception/traffic_light`
4. Verify camera input channels from component config:
   - `/century/sensor/camera/front_6mm/image`
   - `/century/sensor/camera/front_12mm/image`
5. Optional visualizer:
   - set `--start_visualizer=true` in `perception_common.flag`

## Common Failure Points

- Camera input topic mismatch or no image stream.
- TF frame or timestamp sync issues.
- Output topic alive but no valid detections due to map/time alignment.

## Guardrails

- Use component config topic names as source-of-truth.
- Do not hardcode alternate topics without checking active config.
- Avoid editing launch/dag files unless explicitly requested.

## Output Format For User

1. Launch command and dag path
2. Effective traffic-light config and channels
3. Output topic status
4. First failing stage and evidence
5. Next concrete fix command/file
