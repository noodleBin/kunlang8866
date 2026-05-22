---
name: century-drivable-area-integration
description: Build and validate Century drivable-area delivery from perception to planning/control. Use this whenever the user mentions 可行驶区域, drivable area, ROI polygon output, perception-to-planning interface, or asks to publish free-space/driveable polygons to规控, even if they only ask for "加个proto".
---

# Century Drivable Area Integration

Use this skill to implement or debug explicit drivable-area output in Century.

## What This Skill Delivers

- A concrete implementation path from perception ROI to a publishable proto message.
- A safe rollout plan: V1 current-time drivable polygon, V2 time-expanded drivable area.
- Verification checklist for topic, schema, and planning/control consumption.

## Required Repo Touchpoints

- Perception ROI pipeline
  - `modules/perception/lidar/lib/map_manager/map_manager.cc`
  - `modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc`
  - `modules/perception/lidar/lib/scene_manager/roi_service/roi_service.cc`
- Perception output component
  - `modules/perception/onboard/component/lidar_detector_component.cc`
- Existing perception obstacles proto
  - `modules/perception/proto/perception_obstacle.proto`
  - `modules/perception/proto/perception_obstacle_debug.proto`
- Planning intake
  - `modules/planning/planning_component.cc`

## Workflow

### 1) Confirm schema gap

Verify no existing dedicated drivable-area message in perception obstacle proto.

Search cues: `drivable`, `driveable`, `roi`, `polygon`, `freespace`.

### 2) Define proto contract

Create a dedicated message (do not overload obstacle polygon fields):

- `header`
- `frame_id` / coordinate frame enum
- `repeated Polygon outer_polygons`
- `repeated Polygon holes`
- `valid_duration_sec`
- optional confidence / source

Keep polygons 2D on ground plane for planning/control.

### 3) Build drivable area geometry

Compute at least:

- `P_base`: road+junction ROI polygon from map manager/ROI filter path
- `O_now`: current obstacle occupied polygons
- `P_drive = P_base - Union(O_now)`

V2 (optional): time-indexed `P_drive(t)` using predicted obstacle motion.

### 4) Publish and wire

- Add a writer in perception pipeline (component-level output).
- Publish to a dedicated topic (example: `/century/perception/drivable_area`).
- Add planning/control reader and map into local view/status structure.

### 5) Verify end-to-end

- Topic appears and updates each frame.
- Polygon winding/order and frame convention are consistent.
- Planning reads the message without blocking and has fallback on empty data.

## Design Guardrails

- Do not mix obstacle corner polygons and drivable-area polygons in one field.
- Explicitly define coordinate frame in message.
- Include validity horizon; control must know staleness.
- Preserve backward compatibility for existing perception obstacle topics.

## Output Format For User

Always return:

1. Changed files
2. Message schema summary
3. Publish/read topics
4. Validation results
5. Risks and next step
