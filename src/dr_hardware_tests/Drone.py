from collections import namedtuple
from collections.abc import MutableMapping
from copy import copy
from dataclasses import dataclass, field
import enum
from threading import Event, Lock, Thread
from typing import Callable, TypedDict

from geographic_msgs.msg import GeoPoseStamped
from mavros import mavlink
from mavros_msgs.msg import GlobalPositionTarget, Mavlink, ParamValue, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
from pymavlink import mavutil
from pymavlink.dialects.v10 import common
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler

import rospy


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
}

_SERVICE_TIMEOUT = 10.0  # seconds

class FlightMode(str, enum.Enum):
    # These docs list other possible mode values:
    # http://wiki.ros.org/mavros/CustomModes
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
    # ALTCTL = "ALTCTL"
    # RATTITUDE = "RATTITUDE"
    # MANUAL = "MANUAL"
    # READY = "AUTO.READY"
    # RTGS = "AUTO.RTGS"


class Drone:
    """sends commands to the drone
    """
    def __init__(self):
        self.services: MutableMapping[str, RosService] = {}
        for service in _MAVROS_SERVICES:
            name = service.name
            service_proxy = rospy.ServiceProxy(service.topic, service.ServiceClass, persistent=True)
            self.services[name] = RosService(proxy=service_proxy, topic=service.topic)
        for ros_srv in self.services.values():
            rospy.wait_for_service(ros_srv.topic, _SERVICE_TIMEOUT)

        self._mavlink_pub = rospy.Publisher("mavlink/to", Mavlink, queue_size=1)
        self._heartbeat_thread = Thread(target=self._heartbeat)
    
    def start(self):
        self._heartbeat_thread.start()

    def stop(self):
        pass

    def arm(self):
        return self.services['arm'].call_service(True)

    def disarm(self):
        return self.services['arm'].call_service(False)
    
    def get_param(self, param_name):
        return self.services['param_get'].call_service(param_name)

    def set_param(self, param_name, real_value=0.0, integer_value=0):
        msg = ParamValue()
        msg.integer = int(integer_value)
        msg.real = float(real_value)
        response = self.services['param_set'].call_service(param_id=param_name, value=msg)
        return response.success

    def set_mode(self, mode: FlightMode):
        """Change the flight controller's mode
        Return True if successful or False if not.
        """
        if not isinstance(mode, FlightMode):
            raise TypeError(f"{mode} is not a FlightMode")
        return self.services['set_mode'].call_service(0, mode.value)
    
    def _heartbeat(self):
        heartbeat_message = self._make_heartbeat_message()
        heart_rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self._mavlink_pub.publish(heartbeat_message)
            # rospy.loginfo("sent heartbeat")
            heart_rate.sleep()
    
    @staticmethod
    def _make_heartbeat_message():
        message = common.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
            # # type=common.MAV_TYPE_ONBOARD_CONTROLLER,
            # type=common.MAV_TYPE_GCS,
            # # autopilot=common.MAV_AUTOPILOT_INVALID,
            # autopilot=0,
            # base_mode=0,
            # custom_mode=0,
            # system_status=0,
            # mavlink_version=0)
        
        # protocol_handler = mavutil.mavlink.MAVLink(
        #     file="",
        #     srcSystem=2,
        #     srcComponent=common.MAV_COMP_ID_ONBOARD_COMPUTER
        # )
        # message.pack(protocol_handler)
        message.pack(mavutil.mavlink.MAVLink("", 2, 1))
        return mavlink.convert_to_rosmsg(message)
    