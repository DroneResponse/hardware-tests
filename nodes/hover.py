#!/usr/bin/env python3
# Arm, takeoff, hover, land.


import dataclasses
from threading import Condition
import time
import rospy

from dr_hardware_tests.flight_helpers import enter_offboard_mode
from dr_hardware_tests import Drone, FlightMode, SensorSynchronizer
from dr_hardware_tests import is_data_available, is_armed, make_func_is_alt_reached, is_loiter_mode
from dr_hardware_tests import is_disarmed, is_on_ground
from dr_hardware_tests import start_RC_failsafe, sleep
from dr_hardware_tests import is_user_ready_to_start
from dr_hardware_tests import SetpointSender


def log(msg):
    rospy.loginfo(f"hover test: {msg}")

def send_velocity_zero_setpoints(setpoint_sender: SetpointSender):
    log("starting SetpointSender")
    setpoint_sender.start()
    log("done starting SetpointSender")
    setpoint_sender.velocity = 0.0, 0.0, 0.0


def main():
    drone = Drone()
    drone.start()
    sensors = SensorSynchronizer()
    sensors.start()

    log("creating SetpointSender")
    setpoint_sender: SetpointSender = SetpointSender(drone=drone)
    send_velocity_zero_setpoints(setpoint_sender)
    sleep(1.75)

    log("waiting for user to start the test with the RC transmitter")
    sensors.await_condition(is_user_ready_to_start)    

    log("waiting for sensor data to come online")
    sensors.await_condition(is_data_available, 30)

    log("checking preflight parameters")
    ret = drone.check_preflight_params()
    if ret != 0:
        log("takeoff altitude and geo-fence not set as expected...exiting")
        return

    log("please arm the drone")

    log("waiting for drone to arm")
    sensors.await_condition(is_armed, 30)

    t = enter_offboard_mode(drone, sensors)

    log("starting RC failsafe trigger")
    start_RC_failsafe(sensors)

    pause_time = max(9.5 - t, 5)
    sleep(pause_time)  #remain armed for about 10 sec (time since the drone armed)

    targ_alt = drone.read_takeoff_alt()
    log("sending takeoff command")
    drone.set_mode(FlightMode.TAKEOFF)
    is_takeoff_alt_reached = make_func_is_alt_reached(targ_alt)
    sensors.await_condition(is_takeoff_alt_reached, 30)

    log("hover in loiter mode")
    drone.set_mode(FlightMode.LOITER)
    sensors.await_condition(is_loiter_mode, 5)

    log("hover for 30 seconds")
    sleep(30)

    log("send land command")
    drone.set_mode(FlightMode.LAND)

    log("waiting to reach the ground")
    sensors.await_condition(is_on_ground)

    log("sending disarm command")
    drone.disarm()
    log("waiting for drone to disarm")
    sensors.await_condition(is_disarmed, 30)

    log("SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_hover")
    main()
    rospy.signal_shutdown("finished")
