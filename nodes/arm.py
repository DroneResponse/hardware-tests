#!/usr/bin/env python3
from threading import Condition
import time
from dr_hardware_tests.flight_predicate import is_offboard_mode
import rospy

from dr_hardware_tests import Drone, SensorSynchronizer, SensorData, flight_helpers, sleep
from dr_hardware_tests import is_user_ready_to_start, start_RC_failsafe
from dr_hardware_tests.Drone import Drone
from dr_hardware_tests import SetpointSender

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

def send_velocity_zero_setpoints(drone: Drone):
    log("creating SetpointSender")
    setpoint_sender: SetpointSender = SetpointSender(drone=drone)
    log("starting SetpointSender")
    setpoint_sender.start()
    log("done starting SetpointSender")
    setpoint_sender.velocity = 0.0, 0.0, 0.0

def main():
    drone, sensors = flight_helpers.start_drone_io()
    send_velocity_zero_setpoints(drone)
    sleep(1.75)

    log("waiting for user to start the test with the RC transmitter")
    sensors.await_condition(is_user_ready_to_start)

    log("sending arm command")
    drone.arm()
    log("waiting for sensors to indicate that we're armed")
    sensors.await_condition(is_armed, 30)

    arm_time = time.monotonic()
    log("waiting for user to enter Offboard mode")
    sensors.await_condition(is_offboard_mode, 7)


    log("starting RC failsafe trigger")
    start_RC_failsafe(sensors)

    # Stay armed for some time
    up_time = time.monotonic() - arm_time
    sleep_time = max(9.5 - up_time, 0)
    sleep(sleep_time)

    log("sending disarm command")
    drone.disarm()
    sensors.await_condition(is_disarmed, 30)
    log("waiting for sensors to indicate that we're disarmed")
    log("SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_arm")
    main()
    rospy.signal_shutdown("arming test: finished")
