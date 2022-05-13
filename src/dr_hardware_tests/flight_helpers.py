from threading import Event, Lock, Thread
import time
import sys
from typing import Tuple
from dr_hardware_tests.Drone import Drone

import rospy

from .sensor import RospyShutdownException, SensorSynchronizer
from .flight_predicate import is_data_available, is_off_ground, is_posctl_mode
from .flight_predicate import is_user_taking_control
from .sleepy import sleep


def _quit_on_rc_trigger(sensors: SensorSynchronizer, failsafe_engaged_event: Event):
    try:
        rospy.loginfo("RC failsafe waiting for data")
        sensors.await_condition(is_data_available)

        rospy.loginfo("RC failsafe now watching for RC override")
        failsafe_engaged_event.set()
        sensors.await_condition(is_user_taking_control)
    except RospyShutdownException:
        return
    rospy.logfatal("Humans have taken control of the drone")
    rospy.signal_shutdown("Humans control the drone")


def start_drone_io() -> Tuple[Drone, SensorSynchronizer]:
    """Create the objects needed to interact with the drone.

    This creates and starts a Drone object that you can use to control the drone.
    This also creates a sensor synchronizer that lets you access sensor data.

    Returns a tuple holding the drone and sensor synchronizer (in that order)
    """
    drone: Drone = Drone()
    drone.start()
    sensors: SensorSynchronizer = SensorSynchronizer()
    sensors.start()
    return drone, sensors


def start_RC_failsafe(sensors: SensorSynchronizer):
    failsafe_engaged_event = Event()
    thread = Thread(target=_quit_on_rc_trigger, args=[sensors, failsafe_engaged_event])
    thread.start()
    failsafe_engaged_event.wait()






