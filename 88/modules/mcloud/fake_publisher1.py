import time

from cyber.python.cyber_py3 import cyber
from modules.monitor.proto.system_status_pb2 import MonitoredData
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.common.proto.error_code_pb2 import PLANNING_ERROR
from modules.planning.proto.decision_pb2 import STOP_REASON_OBSTACLE

cyber.init("test_road_event_pub")
node = cyber.Node("test_road_event_pub")

monitor_writer = node.create_writer(
    "/century/monitor/monitor_data_x86", MonitoredData, 1)
planning_writer = node.create_writer(
    "/century/planning", ADCTrajectory, 1)

monitor_msg = MonitoredData()
fault = monitor_msg.fault_data.add()
fault.code_msg = "LOW_FREQ_LOCALIZATION"

planning_msg = ADCTrajectory()
planning_msg.header.status.error_code = PLANNING_ERROR
planning_msg.decision.main_decision.stop.reason_code = STOP_REASON_OBSTACLE

for _ in range(10):
    monitor_writer.write(monitor_msg)
    planning_writer.write(planning_msg)
    time.sleep(0.2)

cyber.shutdown()
