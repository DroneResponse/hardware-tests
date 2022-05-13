#!/usr/bin/env python3
from threading import Condition
import time
from dr_hardware_tests.flight_predicate import is_offboard_mode
import rospy

from dr_hardware_tests import Drone, SensorSynchronizer, SensorData, flight_helpers, sleep
from dr_hardware_tests import FlightMode
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

def enter_offboard_mode(drone: Drone, sensors: SensorSynchronizer) -> float:
    """Command PX4 to enter offboard mode. Wait until we sense that we're in Offboard mode.
    Return how much time it took to switch to offboard mode in seconds.
    """
    log("switching to offboard mode")
    t0 = time.monotonic()
    drone.set_mode(FlightMode.OFFBOARD)

    log("waiting for PX4 to enter offboard mode")
    sensors.await_condition(is_offboard_mode, 5)
    t = time.monotonic() - t0
    log(f"detected offboard mode after {t} seconds")
    return t

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

    t = enter_offboard_mode(drone, sensors)

    log("starting RC failsafe trigger")
    start_RC_failsafe(sensors)

    sleep(max(9.5 - t, 5))

    log("sending disarm command")
    drone.disarm()
    log("waiting for sensors to indicate that we're disarmed")
    sensors.await_condition(is_disarmed, 30)
    log("SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_arm")
    main()
    rospy.signal_shutdown("arming test: finished")
