#!/usr/bin/env python3
from queue import Queue
from threading import Lock

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from std_msgs.msg import Float64


class Sensor:
    def __init__(self, sensor_registry, sensor_name, ros_topic, ros_type):
        self.sensor_registry = sensor_registry
        self.is_done = False
        self.name = sensor_name
        self.ros_topic = ros_topic
        self.TopicType = ros_type
        self.lock = Lock()
        self.sub = None
    
    def callback(self, data):
        with self.lock:
            if not self.is_done and data is not None:
                rospy.loginfo(f"received {self.name} data")
                self.sensor_registry.accept(self.name)
                self.is_done = True
        
    def start(self):
        with self.lock:
            self.sub = rospy.Subscriber(self.ros_topic, self.TopicType, self.callback)
            rospy.loginfo(f"awaiting {self.name} data")


class SensorRegistry:
    def __init__(self, expected_sensors, done_func):
        self.done = done_func
        self.data = {}
        self.lock = Lock()
        for sensor_name in expected_sensors:
            self.data[sensor_name] = False
    
    def accept(self, sensor_name):
        with self.lock:
            self.data[sensor_name] = True
            if all(self.data.values()):
                self.done()


def main():
    sensors = [
        ("position", "mavros/global_position/global", NavSatFix),
        ("relative_altitude", "mavros/global_position/rel_alt", Float64),
        ("battery", "mavros/battery", BatteryState),
        ("state", "mavros/state", State),
        ("extended_state", "mavros/extended_state", ExtendedState),
        ("diagnostics", "/diagnostics", DiagnosticArray),
        ("estimator_status", "mavros/estimator_status", EstimatorStatus),
        ("imu", "mavros/imu/data", Imu)
    ]

    is_done_queue = Queue()

    done_func = lambda: is_done_queue.put(1)
    sensor_names = [tup[0] for tup in sensors]
    registry = SensorRegistry(sensor_names, done_func)

    inputs = []
    for sensor in sensors:
        name, topic, TopicType = sensor
        inputs.append(Sensor(registry, name, topic, TopicType))

    rospy.init_node("test_sensors")

    for i in inputs:
        i.start()
    
    is_done_queue.get(timeout=60.0)
    rospy.loginfo("SUCCESS! All sensor data received.")
    rospy.signal_shutdown("success")

if __name__ == "__main__":
    main()