#!/usr/bin/env python3
# Arm, takeoff, hover, land.


import dataclasses
from threading import Condition
import rospy

from dr_hardware_tests import Drone, FlightMode, SensorSynchronizer, SensorData


def is_data_available(data: SensorData) -> bool:
    a = dataclasses.asdict(data)
    return all(a.items())


def is_armed(data: SensorData):
    return data.state.armed


def is_disarmed(data: SensorData):
    return not data.state.armed

def is_takeoff_alt_reached(data: SensorData):
    delta_alt = 2.5 - data.relative_altitude.data
    delta_alt = abs(delta_alt)
    return delta_alt < 0.25

def is_loiter_mode(data: SensorData):
    return data.state.mode == FlightMode.LOITER.value

# this constant comes from 
# http://docs.ros.org/en/api/mavros_msgs/html/msg/ExtendedState.html
_LANDED_STATE_ON_GROUND = 1
def is_on_ground(data: SensorData):
    return data.extended_state.landed_state == _LANDED_STATE_ON_GROUND


def main():
    drone = Drone()
    drone.start()
    sensors = SensorSynchronizer()
    sensors.start()

    rospy.loginfo("hover test: waiting for sensor data to come online")
    sensors.await_condition(is_data_available, 30)

    rospy.loginfo("hover test: sending arm command")
    drone.arm()

    rospy.loginfo("hover test: waiting for drone to arm")
    sensors.await_condition(is_armed, 30)

    # rospy.loginfo("hover test: setting takeoff altitude to 3.0 meters")
    # drone.set_param('MIS_TAKEOFF_ALT', real_value=3.0)

    rospy.sleep(1)
    rospy.loginfo("hover test: sending takeoff command")
    drone.set_mode(FlightMode.TAKEOFF)
    sensors.await_condition(is_takeoff_alt_reached, 30)

    rospy.loginfo("hover test: hover in loiter mode")
    drone.set_mode(FlightMode.LOITER)
    sensors.await_condition(is_loiter_mode, 5)

    rospy.loginfo("hover test: hover for 5 seconds")
    rospy.sleep(5)

    rospy.loginfo("hover test: send land command")
    drone.set_mode(FlightMode.LAND)

    rospy.loginfo("hover test: waiting to reach the ground")
    sensors.await_condition(is_on_ground)

    rospy.loginfo("hover test: sending disarm command")
    drone.disarm()
    rospy.loginfo("hover test: waiting for drone to disarm")
    sensors.await_condition(is_disarmed, 30)

    rospy.loginfo("hover test: SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_arm")
    main()
    rospy.signal_shutdown("hover test: finished")
