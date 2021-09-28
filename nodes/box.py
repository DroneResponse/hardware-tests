import dataclasses
from threading import Condition
from typing import List
import rospy

from droneresponse_mathtools import Lla, geoid_height

from dr_hardware_tests import Drone, SensorSynchronizer, SetpointSender
from dr_hardware_tests import FlightMode, SensorData
from dr_hardware_tests import is_data_available, is_armed, is_takeoff_alt_reached, is_loiter_mode
from dr_hardware_tests import is_disarmed, make_is_drone_at_target_func, is_on_ground


def arm(drone: Drone, sensors: SensorSynchronizer):
    drone.arm()
    sensors.await_condition(is_armed, 30)


def takeoff(drone: Drone, sensors: SensorSynchronizer):
    rospy.sleep(1)
    drone.set_mode(FlightMode.TAKEOFF)
    sensors.await_condition(is_takeoff_alt_reached, 30)


def read_lla(sensor_data: SensorData):
    pos = sensor_data.position
    return Lla(pos.latitude, pos.longitude, pos.altitude)


def find_waypoints_pure(current_pos: Lla, rel_alt: float, alt: float):
    delta_alt = alt - rel_alt
    delta_down = -1.0 * delta_alt

    start = current_pos.move_ned(0.0, 0.0, delta_down)
    end = start

    north = current_pos.move_ned(5.0, 0.0, delta_down)
    south = current_pos.move_ned(-5.0, 0.0, delta_down)
    east = current_pos.move_ned(0.0, 5.0, delta_down)
    west = current_pos.move_ned(0.0, -5.0, delta_down)

    return [start, north, east, south, west, north, end]


def find_waypoints(drone: Drone, sensors: SensorSynchronizer, alt: float):
    sensor_data = sensors.sensor_data()

    rel_alt = sensor_data.relative_altitude.data
    current_pos = read_lla(sensor_data)

    return find_waypoints_pure(current_pos, rel_alt, alt)


def ellipsoid_to_amsl(pos: Lla) -> Lla:
    wgs84_alt = pos.altitude
    geoid_separation = geoid_height(pos.latitude, pos.longitude)
    amsl_alt = wgs84_alt - geoid_separation
    return Lla(pos.latitude, pos.longitude, amsl_alt)


def fly_waypoints(drone: Drone, sensors: SensorSynchronizer,
                  waypoints_wgs84: List[Lla]):
    setpoint_sender: SetpointSender = SetpointSender(drone=drone)
    setpoint_sender.start()

    # We need to send PX4 some setpoints before we enable offboard mode
    setpoint_sender.setpoint = ellipsoid_to_amsl(waypoints_wgs84[0])
    rospy.sleep(5)
    drone.set_mode(FlightMode.OFFBOARD)

    for i in range(waypoints_wgs84):
        target_lla = waypoints_wgs84[i]
        is_arrived = make_is_drone_at_target_func(target_lla)

        setpoint_lla = ellipsoid_to_amsl(target_lla)
        setpoint_sender.setpoint = setpoint_lla

        sensors.await_condition(is_arrived)


def land(drone: Drone, sensors: SensorSynchronizer):
    drone.set_mode(FlightMode.LAND)
    sensors.await_condition(is_on_ground)


def main():
    drone: Drone = Drone()
    drone.start()
    sensors: SensorSynchronizer = SensorSynchronizer()
    sensors.start()

    sensors.await_condition(is_data_available, 30)

    arm(drone, sensors)
    takeoff(drone, sensors)

    waypoints_wgs84 = find_waypoints(drone, sensors)
    fly_waypoints(drone, sensors, waypoints_wgs84)

    land(drone, sensors)
    drone.disarm()
    sensors.await_condition(is_disarmed, 30)

    rospy.loginfo("fly box waypoints: SUCCESS")
