# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a camera driver module for the Century autonomous driving platform. It captures frames from multiple V4L2/USB cameras via Tztek hardware, optionally JPEG-compresses them, and publishes raw/compressed images over the Century Cyber middleware (a pub/sub framework similar to ROS).

The module runs exclusively on **aarch64** (ARM64) targets — all BUILD rules conditionally compile to empty on x86_64.

## Build System

Bazel with `rules_cc`. The module lives at `//modules/drivers/camera` in the larger Century monorepo.

```bash
# Build the shared library (must target aarch64)
bazel build //modules/drivers/camera:libcamera_component.so

# Run cpplint
bazel test //modules/drivers/camera:camera_cpplint
bazel test //modules/drivers/camera/tztek:tztek_cpplint

# Install (copies .so + runtime config/dag/launch files)
bazel run //modules/drivers/camera:install
```

## Architecture

**CameraComponent** (`camera_component.cc`) — A `TimerComponent` registered with Cyber. On `Init()`, it reads a protobuf config file, creates a `TztekCamera` instance, and calls its `Init()`. The `Proc()` timer callback is currently a no-op; frame capture runs in threads owned by `CameraManager`.

**TztekCamera** (`tztek/tztek_camera.cc`) — Orchestrator. Powers on cameras via sysfs GPIO nodes, calls `hb_vin_init` with the J5 device config JSON, then creates a `CameraManager` per enabled camera pipeline. Each camera gets its own Cyber `Writer<Image>` (raw) and/or `Writer<CompressedImage>` (compressed) channels. Camera channels are enabled via a 6-bit bitmask (`kDefaultEnableMask = 0x3F`).

**CameraManager** (`tztek/mgr_camera.cc`) — Per-camera capture loop. Calls into V4L2 to dequeue frames, performs optional cropping (TOP/CENTER/BOTTOM modes), and publishes via Cyber writers.

**JPEG Encoder** (`tztek/mgr_camera_jpegenc.cc`) — Hardware-accelerated JPEG encoding path using `libjpegenc`.

**V4L2 Layer** (`tztek/v4l2cam.cc`) — Low-level V4L2 device open/close, buffer management (mmap/DMA), stream on/off.

**NvBufSurface** (`tztek/NvBufSurface.cc`) — NVIDIA buffer surface allocation/mapping for DMA frame transfer.

## Key Configuration Files

- `proto/config.proto` — Protobuf schema for all camera settings (resolution, format, I/O method, exposure, topic mappings, crop mode, etc.)
- `conf/camera_video.pb.txt` — Default protobuf text config. Defines 6 camera pipelines (right_rear, rear, left_front, left_rear, right_front, front) at 1920x1536 YUYV 20fps.
- `conf/hb_j5dev.json` — Hobot J5 device configuration for the VIN (Video Input) subsystem.
- `dag/camera.dag` — Cyber DAG definition; loads `libcamera_component.so` as a timer component with 100ms interval.
- `launch/camera.launch` — Cyber launch file for the camera module alone.
- `launch/camera_and_video.launch` — Launches both camera and video (H.265 encoder) modules.

## External Dependencies

The tztek library links against `-lgeaccam -ljpegenc -lyuv` (vendor-provided shared libraries for the Tztek camera hardware, JPEG encoding, and YUV conversion). Header-only dependencies are in `tztek/inc/` including Hobot VIN APIs (`hb_vin.h`), NVIDIA buffer surfaces, and libyuv.

## Namespace

All code is under `century::drivers::camera`. Protobuf config is `century.drivers.camera.config`.
