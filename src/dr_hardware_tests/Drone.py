import os
from .sleepy import sleep

from dr_hardware_tests.gimbal import Gimbal
os.environ["MAVLINK20"] = "1"

import math
from collections import namedtuple
from collections.abc import MutableMapping
from copy import copy
from dataclasses import dataclass, field
import enum
import queue
from threading import Event, Lock, Thread
from typing import Callable, List, Tuple, TypedDict

from droneresponse_mathtools import Lla

from geographic_msgs.msg import GeoPoseStamped
from mavros import mavlink
from mavros_msgs.msg import GlobalPositionTarget, Mavlink, ParamValue, PositionTarget, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, WaypointClear, WaypointPush
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink1
from pymavlink.dialects.v20 import common as mavlink2
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler

import rospy

from .MavlinkSender import MavlinkSender
from .HeartbeatSender import HeartbeatSender, MavType, MavState

@dataclass(frozen=True)
class ServiceData:
    name: str
    topic: str
    ServiceClass: object = field(hash=False, repr=False, compare=False)


@dataclass
class RosService:
    topic: str
    proxy: Callable

    def call_service(self, *service_args):
        return self.proxy(*service_args)


_MAVROS_SERVICES = {
    ServiceData('arm', 'mavros/cmd/arming', CommandBool),
    ServiceData('param_get', 'mavros/param/get', ParamGet),
    ServiceData('param_set', 'mavros/param/set', ParamSet),
    ServiceData('set_mode', 'mavros/set_mode', SetMode),
    ServiceData('clear_geofence', 'mavros/geofence/clear', WaypointClear),
    ServiceData('set_geofence', 'mavros/geofence/push', WaypointPush),
}

_SERVICE_TIMEOUT = 10.0  # seconds

class FlightMode(str, enum.Enum):
    # These docs list other possible mode values:
    # http://wiki.ros.org/mavros/CustomModes
    ALTCTL = "ALTCTL"
    LAND = "AUTO.LAND"
    LOITER = "AUTO.LOITER"
    MISSION = "AUTO.MISSION"
    OFFBOARD = "OFFBOARD"
    POSCTL = "POSCTL"
    RTL = "AUTO.RTL"
    STABILIZED = "STABILIZED"
    TAKEOFF = "AUTO.TAKEOFF"
    # These flight modes were deliberately excluded:
    # ACRO = "ACRO"
    # RATTITUDE = "RATTITUDE"
    # MANUAL = "MANUAL"
    # READY = "AUTO.READY"
    # RTGS = "AUTO.RTGS"


class Drone:
    """sends commands to the drone
    """
    def __init__(self, simulate_gcs_heartbeat=True, init_gimbal=True):
        self.services: MutableMapping[str, RosService] = {}
        for service in _MAVROS_SERVICES:
            name = service.name
            service_proxy = rospy.ServiceProxy(service.topic, service.ServiceClass, persistent=True)
            self.services[name] = RosService(proxy=service_proxy, topic=service.topic)
        for ros_srv in self.services.values():
            rospy.wait_for_service(ros_srv.topic, _SERVICE_TIMEOUT)

        self._setpoint_pub     = rospy.Publisher("mavros/setpoint_position/global", GeoPoseStamped, queue_size=1)
        self._vel_setpoint_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self._mavlink_pub = rospy.Publisher("mavlink/to", Mavlink, queue_size=1)
        
        self._heartbeat_senders: List[HeartbeatSender] = []
        if simulate_gcs_heartbeat: 
            # A system_id near 255 is recommend by the mavlink protocol:
            # https://mavlink.io/en/guide/routing.html    
            # since QGroundControl defaults to 255 we will set this to 254
            gcs_system_id=254
            hb_mavlink_sender = MavlinkSender(gcs_system_id, mavlink2.MAV_COMP_ID_MISSIONPLANNER, self._mavlink_pub)
            # the send frequency of 2 is based on this section of mavlink website says:
            # https://mavlink.io/en/services/heartbeat.html#heartbeat-broadcast-frequency
            gcs_hb = HeartbeatSender(hb_mavlink_sender, component_type=MavType.GCS, send_frequency=2)
            gcs_hb.mav_state = MavState.ACTIVE
            self._heartbeat_senders.append(gcs_hb)
        
        self.system_id = rospy.get_param('mavros/target_system_id')
        self.component_id = mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER
        self.mavlink_sender = MavlinkSender(system_id=self.system_id, component_id=self.component_id, mavlink_pub=self._mavlink_pub)
        self.onboard_heartbeat = HeartbeatSender(self.mavlink_sender, MavType.ONBOARD_CONTROLLER)
        self._heartbeat_senders.append(self.onboard_heartbeat)
        self.gimbal = False
        if init_gimbal:
            self.gimbal = Gimbal(self.mavlink_sender)

    
    def start(self):
        for hb in self._heartbeat_senders:
            hb.start()
        
        self.onboard_heartbeat.mav_state = MavState.ACTIVE
        if self.gimbal:
            self.gimbal.start()

    def check_preflight_params(self):
        test_params = {
            'MIS_TAKEOFF_ALT': {
                'property': "real",
                'test': lambda MIS_TAKEOFF_ALT: MIS_TAKEOFF_ALT < 2,
                'error_message': "The takeoff altitude is dangerously low. It must be higher than 2 meters.",
                'problems': [],
            },
            'GF_ACTION': {
                'property': "integer",
                'test': lambda GF_ACTION: GF_ACTION == 0,
                'error_message': 'geofence action is set to do nothing. This means the UAV  will do nothing if it moves outside the geofence. You must choose a different geofence action',
                'problems': [],
            },
            'GF_MAX_HOR_DIST': {
                'property': "real",
                'test': lambda GF_MAX_HOR_DIST: GF_MAX_HOR_DIST < 5 or GF_MAX_HOR_DIST > 500,
                'error_message': "horizontal geofence distance is invalid. It must be greater than 5 meters and less than 500 meter.",
                'problems': [],
            },
            'GF_MAX_VER_DIST': {
                'property': "real",
                'test': lambda GF_MAX_VER_DIST: GF_MAX_VER_DIST < 2 or GF_MAX_VER_DIST > 121.92,
                'error_message': "vertical geofence distance is invalid. It must be greater than 2 meters and less than 121.92 meters (400 ft).",
                'problems': [],
            },

        }

        for param_name, test_data in test_params.items():
            result: ParamGet = self.get_param(param_name)
            if not result.success:
                test_data['problems'].append(f"Could not get {param_name} parameter from PX4")
            else:
                test_data['value'] = getattr(result.value, test_data['property'])
        
        for param_name, test_data in test_params.items():
            if 'value' not in test_data:
                continue
            value = test_data['value']
            bad_result = test_data['test'](value)
            if bad_result:
                test_data['problems'].append(f"PROBLEM: {test_data['error_message']}")
                test_data['problems'].append(f"    You must change: {param_name}")
                test_data['problems'].append(f"    Found: {param_name} = {value}")
                test_data['problems'].append(f"    See Parameter Reference: https://docs.px4.io/master/en/advanced_config/parameter_reference.html#{param_name}")
        
        problems = []
        for test_data in test_params.values():
            for test_err in test_data['problems']:
                problems.append(test_err)
        
        if not problems:
            return 0
        else:
            error_message = "\n".join(problems)
            rospy.logfatal("Preflight checks failed")
            rospy.logfatal(error_message)
            return -1

    def stop(self):
        pass

    def arm(self):
        return self.services['arm'].call_service(True)

    def disarm(self):
        return self.services['arm'].call_service(False)
    
    def get_param(self, param_name):
        return self.services['param_get'].call_service(param_name)
    
    def read_takeoff_alt(self):
        for _ in range(5):
            result = self.get_param('MIS_TAKEOFF_ALT')
            if result.success:
                return result.value.real
            sleep(0.2)
        raise Exception("could not read takeoff altitude")

    def set_param(self, param_name, real_value=0.0, integer_value=0):
        msg = ParamValue()
        msg.integer = int(integer_value)
        msg.real = float(real_value)

        response = self.services['param_set'].call_service(param_name, msg)
        return response.success

    def set_mode(self, mode: FlightMode):
        """Change the flight controller's mode
        Return True if successful or False if not.
        """
        if not isinstance(mode, FlightMode):
            raise TypeError(f"{mode} is not a FlightMode")
        return self.services['set_mode'].call_service(0, mode.value)

    def clear_geofence(self):
        self.services['clear_geofence'].call_service()

    def set_geofence(self, coordinates):
        fence_coordinates = []

        for latitude, longitude in coordinates:
            mission_item = Waypoint()
            mission_item.frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            mission_item.command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            mission_item.is_current = False
            mission_item.autocontinue = False

            mission_item.param1 = len(coordinates)
            mission_item.param2 = 0
            mission_item.param3 = 0
            mission_item.param4 = 0
            mission_item.x_lat = latitude
            mission_item.y_long = longitude
            mission_item.z_alt = 0

            fence_coordinates.append(mission_item)

        self.services['set_geofence'].call_service(0, fence_coordinates)
    
    def send_gimbal_setpoint(self, quaternion):
        message = mavlink2.MAVLink_gimbal_manager_set_attitude_message(target_system=1, target_component=1, flags=0, gimbal_device_id=0, q=quaternion, angular_velocity_x=math.nan, angular_velocity_y=math.nan, angular_velocity_z=math.nan)
        message.pack(mavutil.mavlink.MAVLink("", 2, 1))
        gimbal_cmd = mavlink.convert_to_rosmsg(message)
        self._mavlink_pub.publish(gimbal_cmd)

    
    def send_setpoint(self, lla: Lla, yaw=0.0):
        """Send a setpoint command to the flight controller
        
        Arguments
        =========
        lla: Lla
            Sends a global position set point with latitue, longitude and altitude values
        yaw: float
            Specifies the compass direction that the drone will face while flying (units in radians)
        is_yaw_set: bool
            if True, then the yaw value will be recognized. Otherwise the yaw value is ignored
        """
        lat, lon, alt = lla.lat, lla.lon, lla.alt
        setpoint = self._build_lla_setpoint(lat, lon, alt, yaw=yaw)
        self._setpoint_pub.publish(setpoint)

    @staticmethod
    def _build_lla_setpoint(latitude, longitude, altitude, yaw=0.0):
        """Builds a message for the /mavros/setpoint_position/global
        MAVROS interprets altitude as above mean seal level (AMSL).
        Our GPS sensor is telling us altitude above the WGS-84 ellipsoid.
        """
        geo_pose_setpoint = GeoPoseStamped()
        geo_pose_setpoint.header.stamp = rospy.Time.now()
        geo_pose_setpoint.pose.position.latitude = latitude
        geo_pose_setpoint.pose.position.longitude = longitude
        geo_pose_setpoint.pose.position.altitude = altitude

        roll = 0.0
        pitch = 0.0
        q = quaternion_from_euler(roll, pitch, yaw)
        geo_pose_setpoint.pose.orientation.x = q[0]
        geo_pose_setpoint.pose.orientation.y = q[1]
        geo_pose_setpoint.pose.orientation.z = q[2]
        geo_pose_setpoint.pose.orientation.w = q[3]

        return geo_pose_setpoint
    
    def send_velocity(self, ned_tuple: Tuple[float, float, float]):
        setpoint = self._build_local_setpoint(*ned_tuple)
        self._vel_setpoint_pub.publish(setpoint)

    @staticmethod
    def _build_local_setpoint(north, east, down, is_velocity=True):
        """
        Build a PositionTarget object.
        if is_velocity is True, then the message controls the drone's velcity
        otherwise it controls the drone's position
        """
        # This message is documented here:
        # http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html
        pos_target = PositionTarget()
        pos_target.header.stamp = rospy.Time.now()
        pos_target.coordinate_frame = 1  # FRAME_LOCAL_NED = 1

        # To set the type_mask we need to know what the value means
        # It's a bit mask. Each bit controls whether or not a field is ignored.
        # if the bit is 1 then the field corresponding to that bit is ignored
        #
        # First we will set all the bits to 1.
        # Later we will zero out the bits we need
        pos_target.type_mask = 0b111_111_111_111

        if is_velocity:
            # we need to zero out the velocity bits
            pos_target.type_mask = pos_target.type_mask ^ 0b111_000
            pos_target.velocity.x = north
            pos_target.velocity.y = east
            pos_target.velocity.z = down
        else:
            # we need to zero out the position bits
            pos_target.type_mask = pos_target.type_mask ^ 0b111
            pos_target.position.x = north
            pos_target.position.y = east
            pos_target.position.z = down
        return pos_target

    
