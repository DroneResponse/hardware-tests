#!/usr/bin/env python3

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
from tf.transformations import quaternion_from_euler, quaternion_about_axis, quaternion_multiply
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
    
    ## 1) turn 90 degrees around the first axis
    axis = (1, 0, 0)
    angle = 90
    q1 = quaternion_about_axis(math.radians(angle), axis)

    ## 2) turn 90 degrees around the second axis
    angle = 90
    axis = (0, 1, 0)
    q2 = quaternion_about_axis(math.radians(angle), axis)

    ## 3) turn 90 degrees around the third axis
    angle = 90
    axis = (0, 0, 1)
    q3 = quaternion_about_axis(math.radians(angle), axis)

    ## 4) look right 90 degrees and look down 45 degrees
    a = _quaternion_about_axis_d(90, (0, 0, 1))
    b = _quaternion_about_axis_d(45, (1, 0, 0))
    # this rotates a then b
    q4 = quaternion_multiply(b, a)

    ## 5) look left 90 degrees then look up 45 degrees
    a = _quaternion_about_axis_d(-90, (0, 0, 1))
    b = _quaternion_about_axis_d(45, (1, 0, 0))
    q5 = quaternion_multiply(b, a)

    all_quaternions = [q1, q2, q3, q4, q5]

    log("taking control of gimbal manager sysid=1, compid=1")
    gimbal_manager = MavlinkNode(1, 1)
    drone.gimbal.take_control(gimbal_manager)
    
    log("move the gimbal")
    for q in all_quaternions:
        drone.gimbal.set_attitude(q, gimbal_manager)
        sleep(5)
        drone.gimbal.reset_attitude(gimbal_manager)
        sleep(1)

    
    log("reset gimbal to neutral position")
    drone.gimbal.reset_attitude(gimbal_manager)
    sleep(1)
    



if __name__ == "__main__":
    rospy.init_node("test_indoor_sensors")
    main()
    rospy.signal_shutdown("sensor test: finished")