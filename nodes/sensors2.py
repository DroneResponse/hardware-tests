#!/usr/bin/env python3

import dataclasses

import rospy

from dr_hardware_tests import SensorSynchronizer, SensorData, SensorTest, MAVROS_SENSORS


def test_recv_all_types(data: SensorData) -> bool:
    a = dataclasses.asdict(data)
    missing_messages = []
    for name, msg in a.items():
        if msg == None:
            missing_messages.append(name)
    
    if not missing_messages:
        return True
    else:
        print(f"missing_messages = {missing_messages}")
        return False

def make_callback(sensor_name: str, synchronizer: SensorSynchronizer):
    def callback_func(message):
        synchronizer.update_sensor_data(sensor_name, message)
    return callback_func

def t():
    synchronizer = SensorSynchronizer()
    synchronizer.start()
    subs = []
    
    for sensor in MAVROS_SENSORS:
        cb = make_callback(sensor.name, synchronizer)
        subs.append(sensor.make_sub(cb))

    print("waiting for messages to populate")
    synchronizer.await_condition(test_recv_all_types)
    print("SUCCESS")
    rospy.signal_shutdown("SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_arm")
    t()