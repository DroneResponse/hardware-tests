from threading import Thread
import sys

import rospy

from .sensor import RospyShutdownException, SensorSynchronizer
from .flight_predicate import is_data_available, is_off_ground, is_posctl_mode

def _quit_on_pos_ctl(sensors: SensorSynchronizer):
    try:
        rospy.loginfo("RC watchdog waiting for data")
        sensors.await_condition(is_data_available)
        rospy.loginfo("RC watchdog waiting for drone to lift off the ground")
        sensors.await_condition(is_off_ground)
        rospy.loginfo("RC watchdog engaged")
        sensors.await_condition(is_posctl_mode)
    except RospyShutdownException:
        return
    rospy.logfatal("Humans have taken control of the drone")
    rospy.signal_shutdown("Humans control the drone")

def start_RC_failsafe_watchdog(sensors: SensorSynchronizer):
    thread = Thread(target=_quit_on_pos_ctl, args=[sensors])
    thread.start()