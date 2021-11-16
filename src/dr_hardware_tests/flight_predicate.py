import dataclasses

from droneresponse_mathtools import Lla

from .Drone import FlightMode
from .sensor import SensorData


def is_data_available(data: SensorData) -> bool:
    a = dataclasses.asdict(data)
    # Geofence not needed to run tests
    del a["geofence"]
    for _, msg in a.items():
        if msg == None:
            return False
    return True


def is_armed(data: SensorData):
    return data.state.armed


def is_disarmed(data: SensorData):
    return not data.state.armed


def is_loiter_mode(data: SensorData):
    return data.state.mode == FlightMode.LOITER.value


def is_posctl_mode(data: SensorData):
    return data.state.mode == FlightMode.POSCTL.value


def is_takeoff_mode(data: SensorData):
    return data.state.mode == FlightMode.TAKEOFF.value


def is_offboard_mode(data: SensorData):
    return data.state.mode == FlightMode.OFFBOARD.value


def is_takeoff_or_offboard_mode(data: SensorData):
    return is_takeoff_mode(data) or is_offboard_mode(data)


def make_func_is_alt_reached(alt: float, threshold: float = 0.25):
    def is_takeoff_alt_reached(data: SensorData):
        delta_alt = alt - data.relative_altitude.data
        delta_alt = abs(delta_alt)
        return delta_alt < threshold

    return is_takeoff_alt_reached


def make_func_is_drone_at_target(target_wgs84: Lla,
                                 threshold_distance_meters: float = 0.25):
    def is_arrived(data: SensorData):
        lat = data.position.latitude
        lon = data.position.longitude
        alt = data.position.altitude
        current_pos = Lla(lat, lon, alt)
        return current_pos.distance(target_wgs84) < threshold_distance_meters

    return is_arrived


def is_on_ground(data: SensorData):
    # this constant comes from
    # http://docs.ros.org/en/api/mavros_msgs/html/msg/ExtendedState.html
    _LANDED_STATE_ON_GROUND = 1
    return data.extended_state.landed_state == _LANDED_STATE_ON_GROUND


def is_off_ground(data: SensorData):
    # _LANDED_STATE_ON_GROUND comes from
    # http://docs.ros.org/en/api/mavros_msgs/html/msg/ExtendedState.html
    _LANDED_STATE_ON_GROUND = 1
    return data.extended_state.landed_state > _LANDED_STATE_ON_GROUND


def is_inside_geofence(data: SensorData):
    geofence = data.geofence
    coordinates = [(w.x_lat, w.y_long) for w in geofence.waypoints]
    pos = data.position
    return inside_polygon(len(coordinates), coordinates, (pos.latitude, pos.longitude))


def inside_polygon(num_vertices: int, polygon, location):
    test_lat, test_lon = location
    inside = False
    i = 0
    j = num_vertices - 1
    lat = [p[0] for p in polygon]
    lon = [p[1] for p in polygon]
    while i < num_vertices:
        if ((lat[i] > test_lat) != (lat[j] > test_lat)) and (
            test_lon < (lon[j] - lon[i]) * (test_lat - lat[i]) / (lat[j] - lat[i]) + lon[i]
        ):

            inside = not inside
        j = i
        i += 1

    return inside


def is_user_ready_to_start(data: SensorData):
    """If this function returns True then the user has indicated (with the RC transmitter) that
    they are ready to start the test.
    """
    if data.rcin is None:
        return False
    chan5_raw = data.rcin.channels[5]
    chan8_raw = data.rcin.channels[8]

    is_chan5_ok = 1380 <= chan5_raw and chan5_raw <= 1480
    is_chan8_ok = chan8_raw < 1500
    return is_chan5_ok and is_chan8_ok


def is_user_taking_control(data: SensorData):
    """If this function returns True, then the user is trying to take control with the RC
    transmitter
    """
    if data.state is None:
        return False
    if data.rcin is None:
        return False

    # if the flight controller enters these modes, it means the user is trying to take control
    user_control_modes = [
        FlightMode.STABILIZED.value,
        FlightMode.ALTCTL.value,
        FlightMode.POSCTL.value,
        FlightMode.RTL.value,
    ]
    if data.state.mode in user_control_modes:
        return True

    # if the user tries to land using the RC, then we need to exit
    chan5_raw = data.rcin.channels[5]
    return 1160 <= chan5_raw and chan5_raw <= 1320
