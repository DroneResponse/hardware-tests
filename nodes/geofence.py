#!/usr/bin/env python3

import rospy

from droneresponse_mathtools import Lla
from dr_hardware_tests import Drone, SensorSynchronizer
from dr_hardware_tests import FlightMode, SensorData
from dr_hardware_tests import is_data_available, is_inside_geofence

INVALID_COORDINATES = [
    [42.709686, -67.9603958],
    [42.661231, -67.9586792],
    [42.660979, -67.8972244],
    [42.709686, -67.9010010],
    [42.709686, -67.9603958],
]

CIRCULAR_RADIUS = 100.0


def log(msg):
    rospy.loginfo(f"geofence test: {msg}")


def read_lla(sensor_data: SensorData):
    pos = sensor_data.position
    return Lla(pos.latitude, pos.longitude, pos.altitude)


def get_geofence(sensors: SensorSynchronizer):
    sensor_data = sensors.sensor_data()
    geofence = sensor_data.geofence
    return [(w.x_lat, w.y_long) for w in geofence.waypoints]

def calculate_geofence(sensors: SensorSynchronizer):
    sensor_data = sensors.sensor_data()
    current_pos = read_lla(sensor_data)
    latitude = current_pos.get_latitude()
    longitude = current_pos.get_longitude()

    point1 = [latitude + 0.001, longitude - 0.001]
    point2 = [latitude - 0.001, longitude - 0.001]
    point3 = [latitude - 0.001, longitude + 0.001]
    point4 = [latitude + 0.001, longitude + 0.001]

    return [point1, point2, point3, point4, point1]


def main():
    log("initializing")
    drone: Drone = Drone()
    drone.start()
    sensors: SensorSynchronizer = SensorSynchronizer()
    sensors.start()

    log("waiting for data")
    sensors.await_condition(is_data_available, 30)

    log("setting invalid geofence")
    drone.set_geofence(INVALID_COORDINATES)

    # Wait for geofence/waypoints topic to update
    rospy.sleep(1)

    if is_inside_geofence(sensors.sensor_data()):
        rospy.logfatal("Invalid geofence passes inclusion check")

    drone.clear_geofence()

    log("setting valid geofence")
    valid_coordinates = calculate_geofence(sensors)
    drone.set_geofence(valid_coordinates)

    # Wait for geofence/waypoints topic to update
    rospy.sleep(1)

    sensors.await_condition(is_inside_geofence, 30)

    drone.clear_geofence()

    log("setting circular geofence")
    drone.set_param("GF_MAX_HOR_DIST", CIRCULAR_RADIUS)

    # Wait for parameters to update
    rospy.sleep(1)

    circular_geofence = drone.get_param("GF_MAX_HOR_DIST")
    if not circular_geofence.success or circular_geofence.value.real != CIRCULAR_RADIUS:
        rospy.logfatal("Circular geofence was unable to be set")

    log("SUCCESS")

if __name__ == "__main__":
    rospy.init_node("test_geofence")
    main()
    rospy.signal_shutdown("testing finished")
