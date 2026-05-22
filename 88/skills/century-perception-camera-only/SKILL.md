---
name: century-perception-camera-only
description: Use when user asks to "run camera perception", "start perception_camera", "check camera obstacle output", or debug camera perception dag/config/topic in Century.
---

# Century Perception Camera-Only

Use this skill for camera-only obstacle perception bring-up and diagnosis.

## Source-of-Truth Paths

- `modules/perception/production/launch/perception_camera.launch`
- `modules/perception/production/dag/dag_streaming_perception_camera.dag`
- `modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`
- `modules/perception/production/conf/perception/perception_common.flag`
- `docs/howto/how_to_run_perception_module_on_your_local_computer.md`

## Standard Workflow

1. Start camera-only perception:
   - `cyber_launch start /century/modules/perception/production/launch/perception_camera.launch`
2. Replay bag for validation when needed:
   - `cyber_recorder play -f /century/data/bag/anybag -r 0.2`
3. Verify camera input channels from config:
   - `/century/sensor/camera/front_6mm/image`
   - `/century/sensor/camera/front_12mm/image`
4. Verify obstacle output channel behavior:
   - check configured obstacle output topic in camera component config
   - confirm planning-consumed obstacle topic is alive
5. Optional visualization:
   - set `enable_visualization: true` in `fusion_camera_detection_component.pb.txt`

## Common Failure Points

- Camera channels not present or naming mismatch.
- Config points to wrong model/config directory.
- Topic contract mismatch between camera output and downstream consumer.

## Guardrails

- Always read topic names from active config before declaring channel issues.
- Keep launch path canonical; avoid editing production flags unless requested.

## Output Format For User

1. Launch command and dag path
2. Effective camera config and input channels
3. Output topic check result
4. First failing stage with evidence
5. Exact next fix step
