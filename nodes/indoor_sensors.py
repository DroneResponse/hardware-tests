#!/usr/bin/env python3

import dataclasses

import rospy

from dr_hardware_tests import SensorSynchronizer, SensorData, SensorMeta, MAVROS_SENSORS


def is_outdoor_sensor(sensor_meta: SensorMeta):
    return sensor_meta.name != "position" and sensor_meta.name != "relative_altitude"

INDOOR_SENSORS = set(filter(is_outdoor_sensor, MAVROS_SENSORS))

class _SensorDetectionEvents:
    def __init__(self):
        self.detected_types = set()

    def next_name(self, name):
        if name not in self.detected_types:
            rospy.loginfo(f"sensor test: detected {name}")
            self.detected_types.add(name)

_detected = _SensorDetectionEvents()

_sensor_names = set(map(lambda x: x.name, INDOOR_SENSORS))


def test_recv_all_types(data: SensorData) -> bool:
    a = dataclasses.asdict(data)
    missing_messages = set()
    for name in _sensor_names:
        msg = a[name]
        if msg == None:
            missing_messages.add(name)
        else:
            _detected.next_name(name)

    if not missing_messages:
        return True
    else:
        rospy.logdebug(f"missing sensors: {missing_messages}")
        return False


def main():
    synchronizer = SensorSynchronizer()
    synchronizer.start(sensors=INDOOR_SENSORS)

    rospy.loginfo(f"sensor test: testing if these sensors are available: {_sensor_names}")
    try:
        synchronizer.await_condition(test_recv_all_types, 60)
        rospy.loginfo("sensor test: SUCCESS")
    except Exception as e:
        rospy.logfatal(f"{type(e).__name__} {e}")


if __name__ == "__main__":
    rospy.init_node("test_indoor_sensors")
    main()
    rospy.signal_shutdown("sensor test: finished")