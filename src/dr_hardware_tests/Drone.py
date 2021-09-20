from collections import namedtuple
from collections.abc import MutableMapping
from copy import copy
from dataclasses import dataclass, field
from threading import Event, Lock, Thread
from typing import Callable, TypedDict

from geographic_msgs.msg import GeoPoseStamped
from mavros import mavlink
from mavros_msgs.msg import GlobalPositionTarget, Mavlink, ParamValue, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
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

    def stop(self):
        pass

    def arm(self):
        return self.services['arm'].call_service(True)

    def disarm(self):
        return self.services['arm'].call_service(False)
