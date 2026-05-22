#!/usr/bin/env python3
import argparse
import json
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


START_FLAG = b"##"
DEFAULT_UNIQUE_ID = "LK1200009PS000000"  # 17 bytes

CMD_TASK_RECV = 0xC5
CMD_CONTROL_REQUEST = 0xCA
CMD_TASK_CONTROL = 0xC7
CMD_PARAMETER_QUERY = 0xC8
CMD_SET_QUERY = 0xC9
CMD_INFORM_BROADCAST = 0xD5
CMD_REPLY_MESSAGE = 0xD6
CMD_INFORM_NOTIFY = 0xD7

CTRL_REBOOT = 0x01
CTRL_FINE_TUNING = 0x02
CTRL_MOVE = 0x06
CTRL_IMMEDIATELY_ARRIVE = 0x09
CTRL_TASK_STATUS_CONTROL = 0x0A
CTRL_INFORM_BORROW = 0x0C
CTRL_STOP = 0x0D
CTRL_MULTI_SELECT = 0x0E
CTRL_MULTI_BEGIN = 0x0F

MOVE_IN = 0x01
MOVE_OUT = 0x02


@dataclass
class VehiclePacket:
    command_id: int
    response_flag: int
    unique_id: str
    encryption_method: int
    data_unit: bytes

    def pack(self) -> bytes:
        unique = self.unique_id.encode("ascii")
        if len(unique) not in (17, 21):
            raise ValueError("unique_id length must be 17 or 21")
        data_len = len(self.data_unit)
        header = (
            START_FLAG
            + bytes([self.command_id, self.response_flag])
            + unique
            + bytes([self.encryption_method])
            + struct.pack(">H", data_len)
        )
        bcc = 0
        for b in header[2:] + self.data_unit:
            bcc ^= b
        return header + self.data_unit + bytes([bcc])


class Protocol:
    @staticmethod
    def now6() -> bytes:
        t = time.localtime()
        return struct.pack("6B", t.tm_year % 100, t.tm_mon, t.tm_mday,
                           t.tm_hour, t.tm_min, t.tm_sec)

    @staticmethod
    def seq4(seq: int) -> bytes:
        return struct.pack(">I", seq & 0xFFFFFFFF)

    @staticmethod
    def be_u16(v: int) -> bytes:
        return struct.pack(">H", v & 0xFFFF)

    @staticmethod
    def be_u32(v: int) -> bytes:
        return struct.pack(">I", v & 0xFFFFFFFF)

    @staticmethod
    def be_u64(v: int) -> bytes:
        return struct.pack(">Q", v & 0xFFFFFFFFFFFFFFFF)

    @staticmethod
    def lonlat_to_u64(lon: float, lat: float) -> Tuple[int, int]:
        return int(lon * 1e8), int(lat * 1e8)


class MCloudCommandFactory:
    def __init__(self, unique_id: str = DEFAULT_UNIQUE_ID,
                 encryption_method: int = 0x01):
        self.unique_id = unique_id
        self.encryption_method = encryption_method

    def _packet(self, cmd: int, data_unit: bytes,
                response_flag: int = 0xFE) -> bytes:
        return VehiclePacket(
            command_id=cmd,
            response_flag=response_flag,
            unique_id=self.unique_id,
            encryption_method=self.encryption_method,
            data_unit=data_unit,
        ).pack()

    def task_recv_new(
        self,
        seq: int,
        status: int,
        is_loading: int,
        container_mode: int,
        container_position: int,
        lon: float,
        lat: float,
        heading_deg: int,
        pose_type: int,
        station_type: int,
        crane_id: str = "",
        pose_size: int = 1,
    ) -> bytes:
        task_info = 0
        task_info |= (is_loading & 0x01)
        task_info |= (container_mode & 0x03) << 1
        task_info |= (container_position & 0x03) << 3
        lon_u64, lat_u64 = Protocol.lonlat_to_u64(lon, lat)
        crane = crane_id.encode("utf-8")

        data = (
            Protocol.now6()
            + Protocol.seq4(seq)
            + bytes([0x01, status])
            + Protocol.be_u32(task_info)
            + bytes([len(crane)])
            + crane
            + Protocol.be_u16(pose_size)
            + Protocol.be_u64(lon_u64)
            + Protocol.be_u64(lat_u64)
            + Protocol.be_u16(heading_deg)
            + bytes([pose_type, station_type])
        )
        return self._packet(CMD_TASK_RECV, data)

    def task_recv_sub2(self, seq: int, status: int = 0,
                       sub_seq: int = 1, pose_size: int = 1) -> bytes:
        data = (
            Protocol.now6()
            + Protocol.seq4(seq)
            + bytes([0x02, status])
            + Protocol.be_u16(sub_seq)
            + Protocol.be_u16(pose_size)
        )
        return self._packet(CMD_TASK_RECV, data)

    def task_recv_sub3(self, seq: int) -> bytes:
        data = Protocol.now6() + Protocol.seq4(seq) + bytes([0x03])
        return self._packet(CMD_TASK_RECV, data)

    def control_reboot(self, seq: int) -> bytes:
        data = Protocol.now6() + Protocol.seq4(seq) + bytes([CTRL_REBOOT])
        return self._packet(CMD_CONTROL_REQUEST, data)

    def control_fine(
        self,
        seq: int,
        status: int,
        distance_cm: int,
        operator_type: int,
        lon: float = 0.0,
        lat: float = 0.0,
        heading_deg: int = 0,
        crane_id: str = "",
    ) -> bytes:
        data = (
            Protocol.now6()
            + Protocol.seq4(seq)
            + bytes([CTRL_FINE_TUNING, status])
            + Protocol.be_u16(distance_cm)
            + bytes([operator_type])
        )
        if operator_type == 0x02:
            lon_u64, lat_u64 = Protocol.lonlat_to_u64(lon, lat)
            crane = crane_id.encode("utf-8")
            data += (
                Protocol.be_u64(lon_u64)
                + Protocol.be_u64(lat_u64)
                + Protocol.be_u16(heading_deg)
                + bytes([len(crane)])
                + crane
            )
        return self._packet(CMD_CONTROL_REQUEST, data)

    def control_move(self, seq: int, move_type: int, lon: float,
                     lat: float, heading_deg: int) -> bytes:
        lon_u64, lat_u64 = Protocol.lonlat_to_u64(lon, lat)
        data = (
            Protocol.now6()
            + Protocol.seq4(seq)
            + bytes([CTRL_MOVE, move_type])
            + Protocol.be_u64(lon_u64)
            + Protocol.be_u64(lat_u64)
            + Protocol.be_u16(heading_deg)
        )
        return self._packet(CMD_CONTROL_REQUEST, data)

    def control_stop(self, seq: int) -> bytes:
        data = Protocol.now6() + Protocol.seq4(seq) + bytes([CTRL_STOP])
        return self._packet(CMD_CONTROL_REQUEST, data)

    def control_immediately_arrive(self, seq: int) -> bytes:
        data = Protocol.now6() + Protocol.seq4(seq) + bytes([CTRL_IMMEDIATELY_ARRIVE])
        return self._packet(CMD_CONTROL_REQUEST, data)

    def control_multi_begin(self, seq: int) -> bytes:
        data = Protocol.now6() + Protocol.seq4(seq) + bytes([CTRL_MULTI_BEGIN])
        return self._packet(CMD_CONTROL_REQUEST, data)

    def control_multi_select(self, seq: int, path_id: int) -> bytes:
        data = (
            Protocol.now6()
            + Protocol.seq4(seq)
            + bytes([CTRL_MULTI_SELECT])
            + Protocol.be_u32(path_id)
        )
        return self._packet(CMD_CONTROL_REQUEST, data)

    def control_task_status(self, seq: int, event_type: int,
                            area_code: int) -> bytes:
        # cloud.cc reads data[11] for event_type and data[23] for area.
        payload = bytearray()
        payload.extend(Protocol.now6())
        payload.extend(Protocol.seq4(seq))
        payload.append(CTRL_TASK_STATUS_CONTROL)
        payload.append(event_type)       # index 11
        payload.extend(b"\x00" * 11)    # index 12..22
        payload.append(area_code)        # index 23
        return self._packet(CMD_CONTROL_REQUEST, bytes(payload))

    def control_inform_borrow(self, seq: int, status: int) -> bytes:
        data = (
            Protocol.now6()
            + Protocol.seq4(seq)
            + bytes([CTRL_INFORM_BORROW, status])
        )
        return self._packet(CMD_CONTROL_REQUEST, data)

    def task_control(self, seq: int, task_control_code: int) -> bytes:
        data = (
            Protocol.now6()
            + Protocol.seq4(seq)
            + bytes([task_control_code])
        )
        return self._packet(CMD_TASK_CONTROL, data)

    def parameter_query(self, seq: int, item_ids: List[int]) -> bytes:
        data = Protocol.now6() + Protocol.seq4(seq) + bytes([len(item_ids)]) + bytes(item_ids)
        return self._packet(CMD_PARAMETER_QUERY, data)

    def set_query(self, seq: int, item_id: int, value: int) -> bytes:
        # cloud.cc currently reads value mostly from data[12], or [12:14] for 0x04
        data = bytearray()
        data.extend(Protocol.now6())
        data.extend(Protocol.seq4(seq))
        data.append(1)         # count
        data.append(item_id)   # data[11]
        if item_id == 0x04:
            data.extend(Protocol.be_u16(value))
        else:
            data.append(value & 0xFF)
        return self._packet(CMD_SET_QUERY, bytes(data))

    def inform_broadcast(self, vehicles: List[Dict]) -> bytes:
        return self._packet(CMD_INFORM_BROADCAST,
                            json.dumps(vehicles, ensure_ascii=False).encode("utf-8"))

    def inform_notify(self, payload: Dict) -> bytes:
        return self._packet(CMD_INFORM_NOTIFY,
                            json.dumps(payload, ensure_ascii=False).encode("utf-8"))

    def reply_message(self, payload: Dict) -> bytes:
        return self._packet(CMD_REPLY_MESSAGE,
                            json.dumps(payload, ensure_ascii=False).encode("utf-8"))


class MCloudTestServer:
    def __init__(self, host: str, port: int, factory: MCloudCommandFactory,
                 verbose_rx: bool = False):
        self.host = host
        self.port = port
        self.factory = factory
        self.verbose_rx = verbose_rx
        self.server: Optional[socket.socket] = None
        self.client: Optional[socket.socket] = None
        self.client_addr = None
        self.running = True
        self.rx_buffer = bytearray()
        self.lock = threading.Lock()
        self.print_lock = threading.Lock()
        self.seq = 1000

    def _log(self, *parts) -> None:
        with self.print_lock:
            print(*parts, flush=True)

    def start(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self._log(f"[server] listening on {self.host}:{self.port}")
        self.client, self.client_addr = self.server.accept()
        self._log(f"[server] client connected: {self.client_addr}")

        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()

        self._interactive_loop()

    def _recv_loop(self) -> None:
        assert self.client is not None
        while self.running:
            try:
                data = self.client.recv(4096)
                if not data:
                    # self._log("[recv] client disconnected")
                    self.running = False
                    break
                # if self.verbose_rx:
                    # self._log(f"[recv-raw] bytes={len(data)} hex={data.hex()}")
                self.rx_buffer.extend(data)
                self._drain_packets()
            except OSError as ex:
                self._log(f"[recv] socket error: {ex}")
                self.running = False
                break

    def _drain_packets(self) -> None:
        while True:
            if len(self.rx_buffer) < 24:
                return
            start = self.rx_buffer.find(START_FLAG)
            if start < 0:
                if self.rx_buffer:
                    self._log(
                        f"[recv] drop {len(self.rx_buffer)} bytes(no start flag): "
                        f"{self.rx_buffer.hex()}"
                    )
                self.rx_buffer.clear()
                return
            if start > 0:
                self._log(
                    f"[recv] skip {start} bytes before start flag: "
                    f"{self.rx_buffer[:start].hex()}"
                )
                del self.rx_buffer[:start]
            if len(self.rx_buffer) < 24:
                return
            data_len = (self.rx_buffer[22] << 8) | self.rx_buffer[23]
            total = 24 + data_len + 1
            if len(self.rx_buffer) < total:
                return
            pkt = bytes(self.rx_buffer[:total])
            del self.rx_buffer[:total]
            self._print_packet(pkt)

    def _print_packet(self, pkt: bytes) -> None:
        cmd = pkt[2]
        rsp = pkt[3]
        unique = pkt[4:21].decode("ascii", errors="replace")
        enc = pkt[21]
        data_len = (pkt[22] << 8) | pkt[23]
        data = pkt[24:24 + data_len]
        bcc = pkt[-1]
        # self._log(
        #     f"[recv] cmd=0x{cmd:02X} rsp=0x{rsp:02X} enc=0x{enc:02X} "
        #     f"len={data_len} uid={unique} bcc=0x{bcc:02X}"
        # )
        # if cmd in (CMD_INFORM_BROADCAST, CMD_INFORM_NOTIFY, CMD_REPLY_MESSAGE,
        #            0xD3, 0xD4):
        #     try:
        #         self._log("[recv-json]", json.loads(data.decode("utf-8")))
        #     except Exception:
        #         self._log("[recv-hex]", data.hex())
        # else:
        #     self._log("[recv-hex]", data.hex())

    def _send(self, pkt: bytes, name: str) -> None:
        if not self.client:
            self._log("[send] no client")
            return
        with self.lock:
            self.client.sendall(pkt)
        self._log(f"[send] {name} bytes={len(pkt)}")

    def _interactive_loop(self) -> None:
        help_text = (
            "\nCommands:\n"
            "  help\n"
            "  exit\n"
            "  task_new\n"
            "  task_sub2\n"
            "  task_sub3\n"
            "  ctrl_reboot\n"
            "  ctrl_fine\n"
            "  ctrl_move_in\n"
            "  ctrl_move_out\n"
            "  ctrl_stop\n"
            "  ctrl_arrive\n"
            "  ctrl_multi_begin\n"
            "  ctrl_multi_select\n"
            "  ctrl_task_status\n"
            "  ctrl_inform_borrow\n"
            "  task_ctrl\n"
            "  param_query\n"
            "  set_query_speed\n"
            "  set_query_fas_aeb\n"
            "  set_query_temp_parking\n"
            "  set_query_immediately_stop\n"
            "  set_query_background_music\n"
            "  inform_broadcast_sample\n"
            "  inform_notify_sample\n"
            "  reply_message_sample\n"
        )
        self._log(help_text)

        while self.running:
            try:
                line = input("mcloud-test> ").strip()
            except (EOFError, KeyboardInterrupt):
                line = "exit"
            if not line:
                continue
            parts = line.split()
            cmd = parts[0]
            seq = self._next_seq()

            try:
                if cmd == "help":
                    self._log(help_text)
                elif cmd == "exit":
                    self.running = False
                    break
                elif cmd == "task_new":
                    pkt = self.factory.task_recv_new(
                        seq=seq,
                        status=0,
                        is_loading=1,
                        container_mode=1,
                        container_position=1,
                        lon=116.397000,
                        lat=39.908000,
                        heading_deg=90,
                        pose_type=1,
                        station_type=2,
                        crane_id="craneA",
                    )
                    self._send(pkt, "task_new")
                elif cmd == "task_sub2":
                    pkt = self.factory.task_recv_sub2(seq=seq, status=0, sub_seq=1, pose_size=1)
                    self._send(pkt, "task_sub2")
                elif cmd == "task_sub3":
                    self._send(self.factory.task_recv_sub3(seq), "task_sub3")
                elif cmd == "ctrl_reboot":
                    self._send(self.factory.control_reboot(seq), "ctrl_reboot")
                elif cmd == "ctrl_fine":
                    pkt = self.factory.control_fine(
                        seq=seq,
                        status=1,
                        distance_cm=120,
                        operator_type=0x02,
                        lon=116.397100,
                        lat=39.908100,
                        heading_deg=90,
                        crane_id="craneA",
                    )
                    self._send(pkt, "ctrl_fine")
                elif cmd == "ctrl_move_in":
                    pkt = self.factory.control_move(
                        seq=seq,
                        move_type=MOVE_IN,
                        lon=116.397200,
                        lat=39.908200,
                        heading_deg=90,
                    )
                    self._send(pkt, "ctrl_move_in")
                elif cmd == "ctrl_move_out":
                    pkt = self.factory.control_move(
                        seq=seq,
                        move_type=MOVE_OUT,
                        lon=116.397300,
                        lat=39.908300,
                        heading_deg=90,
                    )
                    self._send(pkt, "ctrl_move_out")
                elif cmd == "ctrl_stop":
                    self._send(self.factory.control_stop(seq), "ctrl_stop")
                elif cmd == "ctrl_arrive":
                    self._send(self.factory.control_immediately_arrive(seq), "ctrl_arrive")
                elif cmd == "ctrl_multi_begin":
                    self._send(self.factory.control_multi_begin(seq), "ctrl_multi_begin")
                elif cmd == "ctrl_multi_select":
                    pkt = self.factory.control_multi_select(seq, 1)
                    self._send(pkt, "ctrl_multi_select")
                elif cmd == "ctrl_task_status":
                    pkt = self.factory.control_task_status(seq, 1, 0x31)
                    self._send(pkt, "ctrl_task_status")
                elif cmd == "ctrl_inform_borrow":
                    pkt = self.factory.control_inform_borrow(seq, 1)
                    self._send(pkt, "ctrl_inform_borrow")
                elif cmd == "task_ctrl":
                    pkt = self.factory.task_control(seq, 1)
                    self._send(pkt, "task_ctrl")
                elif cmd == "param_query":
                    pkt = self.factory.parameter_query(seq, [0x01, 0x04, 0x0A])
                    self._send(pkt, "param_query")
                elif cmd == "set_query_speed":
                    pkt = self.factory.set_query(seq, 0x04, 360)  # 36.0km/h*10
                    self._send(pkt, "set_query_speed")
                elif cmd == "set_query_fas_aeb":
                    pkt = self.factory.set_query(seq, 0x06, 0x02)
                    self._send(pkt, "set_query_fas_aeb")
                elif cmd == "set_query_temp_parking":
                    pkt = self.factory.set_query(seq, 0x07, 0x02)
                    self._send(pkt, "set_query_temp_parking")
                elif cmd == "set_query_immediately_stop":
                    pkt = self.factory.set_query(seq, 0x08, 0x02)
                    self._send(pkt, "set_query_immediately_stop")
                elif cmd == "set_query_background_music":
                    pkt = self.factory.set_query(seq, 0x0A, 0x02)
                    self._send(pkt, "set_query_background_music")
                elif cmd == "inform_broadcast_sample":
                    now_ms = int(time.time() * 1000)
                    payload = [
                        {
                            "vehicleId": "KL001",
                            "vin": "LSVCC6AB1BN123456",
                            "vehicleType": 1,
                            "speed": 12.3,
                            "driveMode": 0,
                            "lng": 116.397,
                            "lat": 39.908,
                            "heading": 90,
                            "gear": 1,
                            "taskType": 1,
                            "timestamp": now_ms,
                        },
                        {
                            "vehicleId": "STK01",
                            "vin": "LSVCC6AB1BN654321",
                            "vehicleType": 2,
                            "speed": 3.2,
                            "lng": 116.398,
                            "lat": 39.909,
                            "heading": 180,
                            "taskType": 2,
                            "timestamp": now_ms,
                        },
                    ]
                    self._send(self.factory.inform_broadcast(payload), "inform_broadcast_sample")
                elif cmd == "inform_notify_sample":
                    payload = {
                        "targetNo": "obs_001",
                        "operate": 1,
                        "timestamp": int(time.time()),
                    }
                    self._send(self.factory.inform_notify(payload), "inform_notify_sample")
                elif cmd == "reply_message_sample":
                    payload = {
                        "msgId": "stacker01_P_1700000000",
                        "status": 2,
                        "msg": "ok",
                        "timestamp": int(time.time()),
                    }
                    self._send(self.factory.reply_message(payload), "reply_message_sample")
                else:
                    self._log("unknown command, input 'help'")
            except Exception as ex:
                self._log(f"[error] {ex}")

        self.close()

    def close(self) -> None:
        self.running = False
        if self.client:
            try:
                self.client.close()
            except OSError:
                pass
            self.client = None
        if self.server:
            try:
                self.server.close()
            except OSError:
                pass
            self.server = None

    def _next_seq(self) -> int:
        self.seq += 1
        return self.seq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MCloud command test server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50018)
    parser.add_argument("--unique-id", default=DEFAULT_UNIQUE_ID,
                        help="must be 17 or 21 chars")
    parser.add_argument("--encryption", type=lambda x: int(x, 0), default=0x01)
    parser.add_argument("--verbose-rx", action="store_true",
                        help="print raw received bytes before packet parsing")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    factory = MCloudCommandFactory(unique_id=args.unique_id,
                                   encryption_method=args.encryption)
    server = MCloudTestServer(args.host, args.port, factory,
                              verbose_rx=args.verbose_rx)
    server.start()


if __name__ == "__main__":
    main()
