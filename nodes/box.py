#!/usr/bin/env python3
from typing import List
import rospy

from droneresponse_mathtools import Lla, geoid_height

from dr_hardware_tests import Drone, SensorSynchronizer, SetpointSender
from dr_hardware_tests import FlightMode, SensorData
from dr_hardware_tests import is_data_available, is_armed, make_func_is_alt_reached, is_loiter_mode
from dr_hardware_tests import is_disarmed, make_func_is_drone_at_target, is_on_ground
from dr_hardware_tests import is_user_ready_to_start, start_RC_failsafe
from dr_hardware_tests import sleep


def log(msg):
    rospy.loginfo(f"box test: {msg}")


def arm(drone: Drone, sensors: SensorSynchronizer):
    log("sending arm command")
    drone.arm()
    log("waiting for drone to arm")
    sensors.await_condition(is_armed, 30)
    log("drone is armed")


def takeoff(drone: Drone, sensors: SensorSynchronizer):
    # rospy.sleep(1)

    targ_alt = drone.get_param('MIS_TAKEOFF_ALT')
    log("switching to takeoff mode")
    drone.set_mode(FlightMode.TAKEOFF)
    log("waiting for drone to reach alt")

    is_takeoff_alt_reached = make_func_is_alt_reached(targ_alt)
    sensors.await_condition(is_takeoff_alt_reached, 30)
    log("takeoff complete")


def read_lla(sensor_data: SensorData):
    pos = sensor_data.position
    return Lla(pos.latitude, pos.longitude, pos.altitude)


def find_waypoints_pure(square_center_pos: Lla, rel_alt: float, alt: float):
    delta_alt = alt - rel_alt
    delta_down = -1.0 * delta_alt

    start = square_center_pos.move_ned(0.0, 0.0, delta_down)
    end = start

    north = square_center_pos.move_ned(5.0, 0.0, delta_down)
    south = square_center_pos.move_ned(-5.0, 0.0, delta_down)
    east = square_center_pos.move_ned(0.0, 5.0, delta_down)
    west = square_center_pos.move_ned(0.0, -5.0, delta_down)

    return [start, north, east, south, west, north, end]


def find_waypoints(drone: Drone, sensors: SensorSynchronizer, alt: float):
    log("calculating waypoints to trace out a square")
    sensor_data = sensors.sensor_data()

    rel_alt = sensor_data.relative_altitude.data
    current_pos = read_lla(sensor_data)
    center = current_pos.move_ned(0.0, -25.0, 0.0)

    waypoints = find_waypoints_pure(center, rel_alt, alt)
    waypoints.append(current_pos)

    log_message = [str(wp) for wp in waypoints]
    log_message = f"found waypoints: {str(log_message)}"
    log(log_message)

    return waypoints


def ellipsoid_to_amsl(pos: Lla) -> Lla:
    wgs84_alt = pos.altitude
    geoid_separation = geoid_height(pos.latitude, pos.longitude)
    amsl_alt = wgs84_alt - geoid_separation
    return Lla(pos.latitude, pos.longitude, amsl_alt)


def fly_waypoints(drone: Drone, sensors: SensorSynchronizer,
                  waypoints_wgs84: List[Lla]):
    log("fly_waypoints")
    log("creating SetpointSender")
    setpoint_sender: SetpointSender = SetpointSender(drone=drone)
    log("starting SetpointSender")
    setpoint_sender.start()
    log("done starting SetpointSender")

    # We need to send PX4 some setpoints before we enable offboard mode
    log("setting inital setpoint")
    setpoint_sender.setpoint = ellipsoid_to_amsl(waypoints_wgs84[0])
    log("sleeping for 5 seconds")
    sleep(5)
    log("switching to offboard mode")
    drone.set_mode(FlightMode.OFFBOARD)
    waypoint_names = [
        "square center", "north", "east", "south", "west", "north",
        "square center", "home"
    ]
    for i in range(len(waypoints_wgs84)):
        target_lla = waypoints_wgs84[i]
        is_arrived = make_func_is_drone_at_target(target_lla)

        setpoint_lla = ellipsoid_to_amsl(target_lla)
        log(f"flying to the {waypoint_names[i]} waypoint (#{i} of {len(waypoint_names)}) at {setpoint_lla}")
        setpoint_sender.setpoint = setpoint_lla

        sensors.await_condition(is_arrived)
    log("done flying waypoints")


def land(drone: Drone, sensors: SensorSynchronizer):
    log("switching to land mode")
    drone.set_mode(FlightMode.LAND)
    log("waiting for drone to touch the ground")
    sensors.await_condition(is_on_ground)
    log("the drone has landed")


def main():
    log("initializing")
    drone: Drone = Drone()
    drone.start()
    sensors: SensorSynchronizer = SensorSynchronizer()
    sensors.start()

    log("waiting for user to start the test with the RC transmitter")
    sensors.await_condition(is_user_ready_to_start)

    log("starting RC failsafe trigger")
    start_RC_failsafe(sensors)

    log("waiting for sensors data")
    sensors.await_condition(is_data_available, 30)

    log("checking preflight parameters")
    ret = drone.check_preflight_params()
    if ret != 0:
        log("takeoff altitude and geo-fence not set as expected...exiting")
        return
        

    arm(drone, sensors)
    sleep(10)
    takeoff(drone, sensors)
    sleep(5)

    waypoints_wgs84 = find_waypoints(drone, sensors, 10.0)
    fly_waypoints(drone, sensors, waypoints_wgs84)
    sleep(5)
    land(drone, sensors)
    drone.disarm()
    sensors.await_condition(is_disarmed, 30)

    rospy.loginfo("box test: SUCCESS")


if __name__ == "__main__":
    rospy.init_node("test_fly_box")
    main()
    rospy.signal_shutdown("test finished")
