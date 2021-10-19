#!/usr/bin/env python3

from queue import Queue

import rospy
from mavros import mavlink
from mavros_msgs.msg import Mavlink

from dr_hardware_tests import Drone, SensorSynchronizer, SensorMeta
from dr_hardware_tests import decode_mavlink2, sleep, start_drone_io



def log(msg):
    rospy.loginfo(f"gimbal test: {msg}")


def read_gimbal_cid():
    return_channel = Queue()
    def cb(message: Mavlink):
        b: bytearray = mavlink.convert_to_bytes(message)
        mavmsg = decode_mavlink2(b)
        if mavmsg.get_msgId() == 0:
            print(mavmsg)
            return_channel.put(message)


    meta = SensorMeta('mavlink_from', 'mavlink/from', Mavlink)
    sub = meta.make_sub(cb)

    for _ in range(10):
        return_channel.get()
    sub.unregister()


def main():
    log("initializing")
    drone, sensors = start_drone_io()
    read_gimbal_cid()
    



if __name__ == "__main__":
    rospy.init_node("test_indoor_sensors")
    main()
    rospy.signal_shutdown("sensor test: finished")