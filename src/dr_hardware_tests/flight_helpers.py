from threading import Event, Lock, Thread
import time
import sys
from typing import Tuple
from dr_hardware_tests.Drone import Drone

import rospy
import math
import tf
import tf.transformations
from droneresponse_mathtools import Lla, geoid_height

from .sensor import RospyShutdownException, SensorSynchronizer
from .flight_predicate import is_data_available, is_off_ground, is_posctl_mode
from .flight_predicate import is_user_taking_control
from .flight_predicate import is_offboard_mode
from dr_hardware_tests import FlightMode, SensorData
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


def enter_offboard_mode(drone: Drone, sensors: SensorSynchronizer) -> float:
    """Command PX4 to enter offboard mode. Wait until we sense that we're in Offboard mode.
    Return how much time it took to switch to offboard mode in seconds.
    """
    rospy.loginfo("switching to offboard mode")
    t0 = time.monotonic()
    drone.set_mode(FlightMode.OFFBOARD)

    rospy.loginfo("waiting for PX4 to enter offboard mode")
    sensors.await_condition(is_offboard_mode, 5)
    t = time.monotonic() - t0
    rospy.loginfo(f"detected offboard mode after {round(t, 3)} seconds")
    return t


def read_lla(sensor_data: SensorData):
    """
    Return the drone's position with altitude specified as meters above the ellipsoid
    """
    pos = sensor_data.position
    return Lla(pos.latitude, pos.longitude, pos.altitude)


def enu_up_angle_to_compass(euler_angles_deg: float) -> float:
        x = 90 - euler_angles_deg
        return x % 360

def compass_to_enu_up(compass_heading_deg: float) -> float:
    return 90 - compass_heading_deg

def read_heading(sensor_data: SensorData):
    q = sensor_data.imu.orientation
    quaternion_angles = [q.x, q.y, q.z, q.w]
    euler_angles = tf.transformations.euler_from_quaternion(quaternion_angles)
    up_angle = math.degrees(euler_angles[2])
    return enu_up_angle_to_compass(up_angle)