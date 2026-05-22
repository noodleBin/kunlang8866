import time

from cyber.python.cyber_py3 import cyber
from modules.common.proto.error_code_pb2 import OK, PLANNING_ERROR
from modules.monitor.proto.system_status_pb2 import MonitoredData
from modules.planning.proto.decision_pb2 import STOP_REASON_OBSTACLE
from modules.planning.proto.planning_pb2 import ADCTrajectory

ERROR_PUBLISH_COUNT = 10
RECOVERY_PUBLISH_COUNT = 10
PUBLISH_INTERVAL_SEC = 0.2


def build_error_monitor_msg():
    msg = MonitoredData()
    fault = msg.fault_data.add()
    fault.code_msg = "LOW_FREQ_LOCALIZATION"
    return msg


def build_error_planning_msg():
    msg = ADCTrajectory()
    msg.header.status.error_code = PLANNING_ERROR
    msg.decision.main_decision.stop.reason_code = STOP_REASON_OBSTACLE
    return msg


def build_recovery_monitor_msg():
    return MonitoredData()


def build_recovery_planning_msg():
    msg = ADCTrajectory()
    msg.header.status.error_code = OK
    return msg


def publish_for(writer_pairs, count, interval_sec):
    for _ in range(count):
        for writer, msg in writer_pairs:
            writer.write(msg)
        time.sleep(interval_sec)


def main():
    cyber.init("test_road_event_pub")
    node = cyber.Node("test_road_event_pub")

    monitor_writer = node.create_writer(
        "/century/monitor/monitor_data_x86", MonitoredData, 1)
    planning_writer = node.create_writer(
        "/century/planning", ADCTrajectory, 1)

    print("Publishing road-event errors...")
    publish_for([
        (monitor_writer, build_error_monitor_msg()),
        (planning_writer, build_error_planning_msg()),
    ], ERROR_PUBLISH_COUNT, PUBLISH_INTERVAL_SEC)

    print("Publishing recovery messages...")
    publish_for([
        (monitor_writer, build_recovery_monitor_msg()),
        (planning_writer, build_recovery_planning_msg()),
    ], RECOVERY_PUBLISH_COUNT, PUBLISH_INTERVAL_SEC)

    cyber.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
