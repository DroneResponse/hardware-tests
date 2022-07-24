#!/usr/bin/env python3

# Here is what the gimbal test is supposed to do:
# 1) turn 90 degrees around the front axis (roll like you're titlting your head to the right)
# 2) turn 90 degrees around the right axis (look up)
# 3) turn 90 degrees around the down axis (look right)
# 4) look right 90 degrees and look down 45 degrees
# 5) look left 90 degrees then look up 45 degrees
# between each 
#
# How long should this test wait for the gimbal to move?
# After we send a command to maneuver the gimbal we need to wait for the gimbal to finish moving.
# This next variable controls how long will you wait. 
GIMBAL_WAITING_TIME = 5.0 # seconds

import os

from dr_hardware_tests.gimbal import MavlinkNode
os.environ["MAVLINK20"] = "1"

import math
from queue import Queue
from typing import Union

import rospy
from mavros import mavlink
from mavros_msgs.msg import Mavlink
from pymavlink import mavutil

from dr_hardware_tests import Drone, SensorSynchronizer, SensorMeta
from dr_hardware_tests import sleep, start_drone_io
from tf.transformations import quaternion_from_euler, quaternion_about_axis, quaternion_multiply, euler_from_quaternion
from pymavlink.dialects.v20 import common as mavlink2


def log(msg):
    rospy.loginfo(f"gimbal test: {msg}")


def _quaternion_about_axis_d(angle_degrees: float, axis):
    return quaternion_about_axis(math.radians(angle_degrees), axis)


def main():
    log("initializing")
    drone, sensors = start_drone_io()
    log("finding gimbals")
    gimbal_managers = drone.gimbal.find_gimbals(wait_time=1.0)
    log("done finding gimbals")
    for gm in gimbal_managers:
        log(f"found gimbal manager: {gm.mavlink_node}")
    
    ## 1) turn 90 degrees around the front axis
    axis = (1, 0, 0)
    angle = 40
    q1 = quaternion_about_axis(math.radians(angle), axis)

    ## 2) turn 90 degrees around the right axis
    angle = 90
    axis = (0, 1, 0)
    q2 = quaternion_about_axis(math.radians(angle), axis)

    ## 3) turn 90 degrees around the down axis
    angle = 90
    axis = (0, 0, 1)
    q3 = quaternion_about_axis(math.radians(angle), axis)

    ## 4) look right 90 degrees and look down 45 degrees
    a = _quaternion_about_axis_d(90, (0, 0, 1))
    b = _quaternion_about_axis_d(45, (1, 0, 0))
    # this rotates by a then b
    q4 = quaternion_multiply(b, a)

    ## 5) look left 90 degrees then look up 45 degrees
    a = _quaternion_about_axis_d(-90, (0, 0, 1))
    b = _quaternion_about_axis_d(45, (1, 0, 0))
    q5 = quaternion_multiply(b, a)

    all_quaternions = [q1, q2, q3, q4, q5]
    log_messages = [
        "roll gimbal right 40 degrees (rotate around the front axis)",
        "tilt gimbal up 90 degrees (rotate around the right axis)",
        "turn gimbal right 90 degrees (rotate around the down axis)",
        "turn gimbal right 90 degrees and look down 45 degrees",
        "turn gimbal left 90 degrees and look up 45 degrees",
    ]


    log("taking control of gimbal manager sysid=1, compid=1")
    gimbal_manager = MavlinkNode(1, 1)
    drone.gimbal.take_control(gimbal_manager)
    
    log("move the gimbal")
    for index, (q, log_msg) in enumerate(zip(all_quaternions, log_messages)):
        # The next 3 lines fix the quaternion so it works with the SD-HX10 gimbal. Remove this fix to use with gazebo
        euler_values = euler_from_quaternion(q)
        euler_values = [-1.0 * angle for angle in euler_values]
        q = quaternion_from_euler(*euler_values)
        
        log(f"{log_msg} (maneuver {index + 1} of {len(all_quaternions)})")
        drone.gimbal.set_attitude(q, gimbal_manager)
        sleep(GIMBAL_WAITING_TIME)
        log("resetting gimbal to neutral position")
        drone.gimbal.reset_attitude(gimbal_manager)
        sleep(GIMBAL_WAITING_TIME)

    log("test done")


if __name__ == "__main__":
    rospy.init_node("test_indoor_sensors")
    main()
    rospy.signal_shutdown("sensor test: finished")
