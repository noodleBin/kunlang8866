#!/usr/bin/env python3

"""Print detailed raw camera and lidar arrival timestamps from cyber topics."""

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

    for subdir in [
        modules_src / "common" / "proto",
        modules_src / "drivers" / "proto",
    ]:
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
        {"name": name, "channel": channel}
        for name, channel in pattern.findall(text)
    ]


def safe_has_field(msg, field_name: str) -> bool:
    try:
        return msg.HasField(field_name)
    except Exception:
        return False


def get_header_timestamp(msg) -> float:
    if safe_has_field(msg, "header"):
        return getattr(msg.header, "timestamp_sec", 0.0)
    return 0.0


def get_measurement_time(msg) -> float:
    if safe_has_field(msg, "measurement_time"):
        return getattr(msg, "measurement_time", 0.0)
    return 0.0


def get_measuretime(msg) -> float:
    if safe_has_field(msg, "measuretime"):
        return getattr(msg, "measuretime", 0.0)
    return 0.0


def format_delay_ms(recv_wall: float, ts: float) -> float:
    if ts <= 0.0:
        return -1.0
    return (recv_wall - ts) * 1000.0


class ArrivalMonitor(object):
    def __init__(self, camera_config_path: str, dag_path: str):
        self.camera_configs = load_camera_configs(camera_config_path)
        self.lidar_configs = load_lidar_configs_from_dag(dag_path)

        self.lock = threading.Lock()
        self.last_recv_wall_by_key = {}

    def _print_message(
        self,
        kind: str,
        name: str,
        channel: str,
        header_ts: float,
        measurement_time: float,
        measuretime: float,
        recv_wall: float,
    ):
        key = f"{kind}:{name}"

        with self.lock:
            last_recv_wall = self.last_recv_wall_by_key.get(key, 0.0)

            if last_recv_wall > 0.0:
                inter_arrival_ms = (recv_wall - last_recv_wall) * 1000.0
            else:
                inter_arrival_ms = -1.0

            self.last_recv_wall_by_key[key] = recv_wall

        delay_header_ms = format_delay_ms(recv_wall, header_ts)
        delay_measurement_ms = format_delay_ms(recv_wall, measurement_time)
        delay_measuretime_ms = format_delay_ms(recv_wall, measuretime)

        if header_ts > 0.0 and measurement_time > 0.0:
            header_minus_measurement_ms = (header_ts - measurement_time) * 1000.0
        else:
            header_minus_measurement_ms = -1.0

        print(
            f"[PY-{kind}] "
            f"name={name} "
            f"channel={channel} "
            f"header_ts={header_ts:.6f} "
            f"measurement_time={measurement_time:.6f} "
            f"measuretime={measuretime:.6f} "
            f"recv_wall={recv_wall:.6f} "
            f"last_recv_wall={last_recv_wall:.6f} "
            f"inter_arrival={inter_arrival_ms:.3f}ms "
            f"delay_header={delay_header_ms:.3f}ms "
            f"delay_measurement={delay_measurement_ms:.3f}ms "
            f"delay_measuretime={delay_measuretime_ms:.3f}ms "
            f"header_minus_measurement={header_minus_measurement_ms:.3f}ms"
        )

    def on_camera(self, msg, camera_cfg):
        recv_wall = time.time()

        header_ts = get_header_timestamp(msg)
        measurement_time = get_measurement_time(msg)

        # Image proto 里没有 measuretime，这里固定为 0
        measuretime = 0.0

        self._print_message(
            "CAMERA",
            camera_cfg["name"],
            camera_cfg["channel"],
            header_ts,
            measurement_time,
            measuretime,
            recv_wall,
        )

    def on_lidar_raw(self, raw_msg, lidar_cfg):
        recv_wall = time.time()

        header_ts = 0.0
        measurement_time = 0.0
        measuretime = 0.0

        packed = PointCloudPacked()
        parsed = False

        try:
            packed.ParseFromString(raw_msg)
            header_ts = get_header_timestamp(packed)
            measurement_time = get_measurement_time(packed)
            measuretime = get_measuretime(packed)
            parsed = True
        except Exception:
            parsed = False

        if not parsed or (
            header_ts == 0.0
            and measurement_time == 0.0
            and measuretime == 0.0
        ):
            cloud = PointXYZIRTCloud()
            try:
                cloud.ParseFromString(raw_msg)
                header_ts = get_header_timestamp(cloud)
                measurement_time = get_measurement_time(cloud)
                measuretime = get_measuretime(cloud)
            except Exception:
                pass

        self._print_message(
            "LIDAR",
            lidar_cfg["name"],
            lidar_cfg["channel"],
            header_ts,
            measurement_time,
            measuretime,
            recv_wall,
        )

    def run(self):
        cyber.init("camera_lidar_arrival_monitor")
        node = cyber.Node("camera_lidar_arrival_monitor")

        print(f"[PY-MONITOR] subscribe {len(self.camera_configs)} camera topics")

        for camera_cfg in self.camera_configs:
            node.create_reader(
                camera_cfg["channel"],
                Image,
                self.on_camera,
                camera_cfg,
            )

            print(
                f"[PY-MONITOR] camera reader "
                f"{camera_cfg['name']} -> {camera_cfg['channel']}"
            )

        print(f"[PY-MONITOR] subscribe {len(self.lidar_configs)} lidar topics")

        for lidar_cfg in self.lidar_configs:
            node.create_rawdata_reader(
                lidar_cfg["channel"],
                self.on_lidar_raw,
                lidar_cfg,
            )

            print(
                f"[PY-MONITOR] lidar reader "
                f"{lidar_cfg['name']} -> {lidar_cfg['channel']}"
            )

        node.spin()
        cyber.shutdown()


def main():
    repo_root = get_repo_root()

    parser = argparse.ArgumentParser(
        description="Print detailed camera/lidar arrival timestamps"
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
            / "modules/perception/production/dag/"
            / "dag_streaming_kl_perception_lidar_fusion.dag"
        ),
        help="dag file used to discover lidar channels",
    )

    args = parser.parse_args()

    monitor = ArrivalMonitor(args.camera_config, args.dag)
    monitor.run()


if __name__ == "__main__":
    main()