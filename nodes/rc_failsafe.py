#!/usr/bin/env python3
import rospy
from dr_hardware_tests import Drone, SensorSynchronizer, SensorData, flight_helpers, FlightMode
from dr_hardware_tests import is_user_ready_to_start, is_user_taking_control, is_posctl_mode


def log(msg):
    rospy.loginfo(f"rc failsafe test: {msg}")


def mk_mode_check_func(mode):
    def check_mode(data: SensorData):
        if data.mode is None:
            return False
        return data.state.mode == mode
    
    return check_mode


def is_rc_landing_with_channel5(data: SensorData):
    if data.rcin is None:
        return False
    
    chan5_raw = data.rcin.channels[5] 
    is_user_landing = 1160 <= chan5_raw and chan5_raw <= 1320 
    return is_user_landing


def main():
    drone, sensors = flight_helpers.start_drone_io()

    log("waiting for user to start the test with the RC transmitter")
    sensors.await_condition(is_user_ready_to_start)

    flight_modes = [
         (FlightMode.STABILIZED.value, "STABILIZED"),
         (FlightMode.ALTCTL.value, "Altitude"),
         (FlightMode.POSCTL.value, "POSITION"),
         (FlightMode.RTL.value, "RTL"),
    ]

    for mode, mode_name in flight_modes:
        log(f"please enter {mode_name} mode")
        is_mode_active = mk_mode_check_func(mode)
        sensors.await_condition(is_mode_active)
        log(f"testing the failsafe in {mode_name} mode")
        sensors.await_condition(is_user_taking_control)
    
    log("Please start the test with the RC transmitter")
    sensors.await_condition(is_user_ready_to_start)

    log("please trigger a landing with RC channel 5")
    sensors.await_condition(is_rc_landing_with_channel5)
    log("testing failsafe with channel 5 input")
    sensors.await_condition(is_user_taking_control)

    log("done")

if __name__ == "__main__":
    rospy.init_node("test_rc_failsafe")
    main()
    rospy.signal_shutdown("rc failsafe: test finished")
