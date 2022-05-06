#!/usr/bin/env python3
from threading import Condition
import rospy

from dr_hardware_tests import Drone, SensorSynchronizer, SensorData, flight_helpers, sleep
from dr_hardware_tests import is_user_ready_to_start, start_RC_failsafe


def log(msg):
    rospy.loginfo(f"arming test: {msg}")

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

    log("waiting for user to start the test with the RC transmitter")
    sensors.await_condition(is_user_ready_to_start)

    log("starting RC failsafe trigger")
    start_RC_failsafe(sensors)

    log("sending arm command")
    drone.arm()
    log("waiting for sensors to indicate that we're armed")
    sensors.await_condition(is_armed, 30)

    # Stay armed for some time
    sleep(9.5)

    log("sending disarm command")
    drone.disarm()
    sensors.await_condition(is_disarmed, 30)
    log("waiting for sensors to indicate that we're disarmed")
    log("SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_arm")
    main()
    rospy.signal_shutdown("arming test: finished")
