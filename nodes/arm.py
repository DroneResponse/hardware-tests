#!/usr/bin/env python3
from threading import Condition
import rospy

from dr_hardware_tests import Drone, SensorSynchronizer, SensorData, flight_helpers, sleep


def is_armed(data: SensorData):
    if not data.state:
        return False
    return data.state.armed


def is_disarmed(data: SensorData):
    if not data.state:
        return False
    return not data.state.armed


def main():
    drone, sensors = flight_helpers.start_drone_io()

    rospy.loginfo("arming test: sending arm command")
    drone.arm()
    rospy.loginfo(
        "arming test: waiting for sensors to indicate that we're armed")
    sensors.await_condition(is_armed, 30)

    # Stay armed for some time
    sleep(1.0)

    rospy.loginfo("arming test: sending disarm command")
    drone.disarm()
    sensors.await_condition(is_disarmed, 30)
    rospy.loginfo(
        "arming test: waiting for sensors to indicate that we're disarmed")
    rospy.loginfo("arming test: SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_arm")
    main()
    rospy.signal_shutdown("arming test: finished")
