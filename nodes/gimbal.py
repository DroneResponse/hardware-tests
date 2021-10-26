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
from tf.transformations import quaternion_from_euler
from pymavlink.dialects.v20 import common as mavlink2



def log(msg):
    rospy.loginfo(f"gimbal test: {msg}")



def main():
    log("initializing")
    drone, sensors = start_drone_io()
    log("finding gimbals")
    gimbal_managers = drone.gimbal.find_gimbals(wait_time=1.0)
    log("done finding gimbals")
    for gm in gimbal_managers:
        log(f"found gimbal manager: {gm.mavlink_node}")
    q = quaternion_from_euler(0, 0, math.radians(90))

    log("taking control of gimbal manager sysid=1, compid=1")
    gimbal_manager = MavlinkNode(1, 1)
    drone.gimbal.take_control(gimbal_manager)
    drone.gimbal.set_attitude(q, gimbal_manager)
    sleep(1)
    



if __name__ == "__main__":
    rospy.init_node("test_indoor_sensors")
    main()
    rospy.signal_shutdown("sensor test: finished")