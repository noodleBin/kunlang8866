#!/usr/bin/env python3

"""Print raw camera and lidar arrival timestamps from cyber topics."""

import argparse
import os
import pathlib
import re
import shutil
import sys
import tempfile
import threading
import time

import yaml

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

sys.path.append("/century/bazel-bin/cyber/python/internal/")
sys.path.append("/century/bazel-bin")
sys.path.append(".")

from cyber.python.cyber_py3 import cyber


def load_proto_classes():
    bazel_bin_root = pathlib.Path("/century/bazel-bin")
    if not bazel_bin_root.exists():
        raise RuntimeError("/century/bazel-bin does not exist in current environment")

    temp_dir = pathlib.Path(tempfile.mkdtemp(prefix="arrival_monitor_proto_"))
    modules_src = bazel_bin_root / "modules"
    modules_dst = temp_dir / "modules"

    for subdir in [modules_src / "common" / "proto", modules_src / "drivers" / "proto"]:
        if subdir.exists():
            relative = subdir.relative_to(modules_src)
            shutil.copytree(subdir, modules_dst / relative, dirs_exist_ok=True)

    pattern = re.compile(r",\s*create_key=_descriptor\._internal_create_key")
    for pb2_file in temp_dir.glob("modules/**/*_pb2.py"):
        os.chmod(pb2_file, 0o644)
        content = pb2_file.read_text(encoding="utf-8")
        content = pattern.sub("", content)
        pb2_file.write_text(content, encoding="utf-8")

    sys.path.insert(0, str(temp_dir))
    from modules.drivers.proto.sensor_image_pb2 import Image  # type: ignore
    from modules.drivers.proto.pointcloud_pb2 import PointCloudPacked  # type: ignore
    from modules.drivers.proto.pointcloud_pb2 import PointXYZIRTCloud  # type: ignore

    return Image, PointCloudPacked, PointXYZIRTCloud


Image, PointCloudPacked, PointXYZIRTCloud = load_proto_classes()


def get_repo_root() -> pathlib.Path:
    current = pathlib.Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "modules" / "perception").exists():
            return parent
    raise RuntimeError("Failed to locate repo root")


def load_camera_configs(config_path: str):
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    cameras = config.get("cameras", [])
    if not cameras:
        raise ValueError(f"No cameras found in {config_path}")
    return cameras


def load_lidar_configs_from_dag(dag_path: str):
    text = pathlib.Path(dag_path).read_text(encoding="utf-8")
    pattern = re.compile(
        r'name:\s*"([^"]+)".*?readers\s*\{\s*channel:\s*"([^"]+)"',
        re.S,
    )
    return [
        {"name": name, "channel": channel} for name, channel in pattern.findall(text)
    ]


def get_proto_timestamp(msg) -> float:
    if hasattr(msg, "header") and msg.header and msg.header.timestamp_sec:
        return msg.header.timestamp_sec
    if hasattr(msg, "measuretime") and msg.measuretime:
        return msg.measuretime
    if hasattr(msg, "measurement_time") and msg.measurement_time:
        return msg.measurement_time
    return 0.0


class ArrivalMonitor(object):
    def __init__(self, camera_config_path: str, dag_path: str):
        self.camera_configs = load_camera_configs(camera_config_path)
        self.lidar_configs = load_lidar_configs_from_dag(dag_path)
        self.lock = threading.Lock()
        self.last_wall_by_key = {}

    def _print_message(
        self, kind: str, name: str, channel: str, msg_ts: float, wall: float
    ):
        key = f"{kind}:{name}"
        with self.lock:
            last_wall = self.last_wall_by_key.get(key)
            inter_arrival_ms = -1.0
            if last_wall is not None:
                inter_arrival_ms = (wall - last_wall) * 1000.0
            self.last_wall_by_key[key] = wall

        print(
            f"[PY-{kind}] name={name} channel={channel} "
            f"recv_wall={wall:.6f} msg_ts={msg_ts:.6f} "
            f"inter_arrival={inter_arrival_ms:.3f}ms"
        )

    def on_camera(self, msg, camera_cfg):
        self._print_message(
            "CAMERA",
            camera_cfg["name"],
            camera_cfg["channel"],
            get_proto_timestamp(msg),
            time.time(),
        )

    def on_lidar_raw(self, raw_msg, lidar_cfg):
        wall = time.time()
        msg_ts = 0.0

        packed = PointCloudPacked()
        try:
            packed.ParseFromString(raw_msg)
            msg_ts = get_proto_timestamp(packed)
        except Exception:
            msg_ts = 0.0

        if 0.0 == msg_ts:
            cloud = PointXYZIRTCloud()
            try:
                cloud.ParseFromString(raw_msg)
                msg_ts = get_proto_timestamp(cloud)
            except Exception:
                msg_ts = 0.0

        self._print_message(
            "LIDAR",
            lidar_cfg["name"],
            lidar_cfg["channel"],
            msg_ts,
            wall,
        )

    def run(self):
        cyber.init("camera_arrival_monitor")
        node = cyber.Node("camera_arrival_monitor")

        print(f"[PY-MONITOR] subscribe {len(self.camera_configs)} camera topics")
        for camera_cfg in self.camera_configs:
            node.create_reader(camera_cfg["channel"], Image, self.on_camera, camera_cfg)
            print(
                f"[PY-MONITOR] camera reader {camera_cfg['name']} -> {camera_cfg['channel']}"
            )

        print(f"[PY-MONITOR] subscribe {len(self.lidar_configs)} lidar topics")
        for lidar_cfg in self.lidar_configs:
            node.create_rawdata_reader(
                lidar_cfg["channel"], self.on_lidar_raw, lidar_cfg
            )
            print(
                f"[PY-MONITOR] lidar reader {lidar_cfg['name']} -> {lidar_cfg['channel']}"
            )

        node.spin()
        cyber.shutdown()


def main():
    repo_root = get_repo_root()
    parser = argparse.ArgumentParser(
        description="Print camera/lidar arrival timestamps"
    )
    parser.add_argument(
        "--camera_config",
        default="/century/modules/perception/data/params/camera_sensor.yaml",
        help="camera sensor yaml path",
    )
    parser.add_argument(
        "--dag",
        default=str(
            repo_root
            / "modules/perception/production/dag/dag_streaming_kl_perception_lidar_fusion.dag"
        ),
        help="dag file used to discover lidar channels",
    )
    args = parser.parse_args()

    monitor = ArrivalMonitor(args.camera_config, args.dag)
    monitor.run()


if __name__ == "__main__":
    main()
