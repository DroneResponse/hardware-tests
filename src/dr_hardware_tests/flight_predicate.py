import dataclasses

from droneresponse_mathtools import Lla
from .Drone import FlightMode
from .sensor import SensorData


def is_data_available(data: SensorData) -> bool:
    a = dataclasses.asdict(data)
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


def make_func_is_alt_reached(alt: float, threshold:float=0.25):
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
