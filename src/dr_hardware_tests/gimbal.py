import enum

from dataclasses import dataclass, field
from queue import Queue
from threading import Thread
from typing import List, MutableMapping, Tuple, Union

import mavros.mavlink
import rospy

from mavros_msgs.msg import Mavlink
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink1
from pymavlink.dialects.v20 import common as mavlink2


_mav1 = mavlink1.MAVLink(file=None)
_mav2 = mavlink2.MAVLink(file=None)


def decode_mavlink1(buff: Union[bytes, bytearray]) -> mavlink1.MAVLink_message:
    return _mav1.decode(buff)


def decode_mavlink2(buff: Union[bytes, bytearray]) -> mavlink2.MAVLink_message:
    return _mav2.decode(buff)


class _CmdType(enum.Enum):
    EXIT = enum.auto()
    FIND_GIMBALS = enum.auto()
    RECV_MAVLINK = enum.auto()
    SET_ATTITUDE  = enum.auto()


@dataclass
class _GimbalCommand:
    command_type: _CmdType = None
    quaternion: Tuple[float, float, float, float] = None
    mavlink_message: mavlink2.MAVLink_message = None
    return_channel: Queue = field(default_factory=Queue)    


# The set of mavlink messages we want to receive as input
# https://mavlink.io/en/services/gimbal_v2.html#messagecommandenum-summary
MAVLINK_GIMBAL_MSG_IDS = {
    # Gimbal manager messages
    mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
    mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS,
    # mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE,
    # mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW,

    # Gimbal device messages
    mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
    #mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE,
    #mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
}


# This is a python translation of GIMBAL_MANAGER_CAP_FLAGS 
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_CAP_FLAGS
class _GimbalManagerCapability(enum.Flag):
    GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT               = 1 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL               = 2 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS             = 4 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW           = 8 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK             = 16 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS            = 32 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW          = 64 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK            = 128 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS              = 256 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.  
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW            = 512 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.   
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK              = 1024 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW     = 2048 # Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL  = 65536 # Gimbal manager supports to point to a local position.
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072 # Gimbal manager supports to point to a global latitude, longitude, altitude position.


@dataclass
# This class is based on the GIMBAL_MANAGER_INFORMATION message
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION
class _GimbalManager:
    system_id: int
    component_id: int
    capability_flags: _GimbalManagerCapability # Bitmap of gimbal capability flags.
    gimbal_device_id: int # Gimbal device ID that this gimbal manager is responsible for.
    roll_min: float # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left) in radians
    roll_max: float	# Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left) in radians
    pitch_min: float	# Minimum pitch angle (positive: up, negative: down) in radians
    pitch_max: float	# Maximum pitch angle (positive: up, negative: down) in radians
    yaw_min: float	# Minimum yaw angle (positive: to the right, negative: to the left) in radians
    yaw_max: float	# Maximum yaw angle (positive: to the right, negative: to the left) in radians


# GIMBAL_MANAGER_FLAGS
class _GimbalManagerFlags(enum.Flag):
    GIMBAL_MANAGER_FLAGS_RETRACT    = 1 # Based on GIMBAL_DEVICE_FLAGS_RETRACT
    GIMBAL_MANAGER_FLAGS_NEUTRAL    = 2 # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK  = 4 # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8 # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
    GIMBAL_MANAGER_FLAGS_YAW_LOCK   = 16 # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK


# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS
@dataclass
class _GimbalManagerStatus:
    system_id: int
    component_id: int
    flags: _GimbalManagerFlags # High level gimbal manager flags currently applied.
    gimbal_device_id: int # Gimbal device ID that this gimbal manager is responsible for.
    primary_control_sysid: int # System ID of MAVLink component with primary control, 0 for none.
    primary_control_compid: int # Component ID of MAVLink component with primary control, 0 for none.
    secondary_control_sysid: int # System ID of MAVLink component with secondary control, 0 for none.
    secondary_control_compid: int # Component ID of MAVLink component with secondary control, 0 for none.


# This is a python translation of GIMBAL_DEVICE_FLAGS
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_FLAGS
class _GimbalDeviceFlags(enum.Flag):
    GIMBAL_DEVICE_FLAGS_RETRACT    = 1 # Set to retracted safe position (no stabilization), takes presedence over all other flags.
    GIMBAL_DEVICE_FLAGS_NEUTRAL    = 2 # Set to neutral position (horizontal, forward looking, with stabiliziation), takes presedence over all other flags except RETRACT.
    GIMBAL_DEVICE_FLAGS_ROLL_LOCK  = 4 # Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.
    GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8 # Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.
    GIMBAL_DEVICE_FLAGS_YAW_LOCK   = 16 # Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).


# this is a python translation of GIMBAL_DEVICE_ERROR_FLAGS
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ERROR_FLAGS
class _GimbalDeviceErrorFlags(enum.Flag):
    NOTHING                                       = 0 # for no failure
    GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT       = 1 # Gimbal device is limited by hardware roll limit.
    GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT      = 2 # Gimbal device is limited by hardware pitch limit.
    GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT        = 4 # Gimbal device is limited by hardware yaw limit.
    GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR       = 8 # There is an error with the gimbal encoders.
    GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR         = 16 # There is an error with the gimbal power source.
    GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR         = 32 # There is an error with the gimbal motor's.
    GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR      = 64 # There is an error with the gimbal's software.
    GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR         = 128 # There is an error with the gimbal's communication.
    GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256 # Gimbal is currently calibrating.


@dataclass
class _GimbalStatus:
    system_id: int
    component_id: int
    flags: _GimbalDeviceFlags
    q: Tuple[float, float, float, float]
    angular_velocity_x: float # radians per second
    angular_velocity_y: float # radians per second
    angular_velocity_z: float # radians per second
    failure_flags: _GimbalDeviceErrorFlags



@dataclass(order=True, frozen=True)
class _MavlinkNode:
    system_id: int
    component_id: int


class Gimbal:

    def __init__(self):
        self.cmd_queue = Queue()

        self.gimbal_managers: MutableMapping[_MavlinkNode, _GimbalManager] = dict()
        self.gimbal_manager_status: MutableMapping[_MavlinkNode, _GimbalManagerStatus] = dict()
        self.gimbal_status: MutableMapping[_MavlinkNode, _GimbalStatus] = dict()

        self.mavlink_pub: rospy.Publisher = None
        self.mavlink_sub: rospy.Subscriber = None
        self.worker_thread = Thread(target=self._run)

        self._mav_msg_dispatch = {
            mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION: self._recv_gimbal_manager_information,
            mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS: self._recv_gimbal_manager_status,
            mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: self._recv_gimbal_device_attitude_status,
        }

    def _recv_gimbal_manager_information(self, msg: mavlink2.MAVLink_gimbal_manager_information_message):
        mavnode = _MavlinkNode(system_id=msg.get_srcSystem(), cmp_id=msg.get_srcComponent())
        gimbal_manager = _GimbalManager(
            system_id=mavnode.system_id,
            component_id=mavnode.component_id,
            capability_flags=_GimbalManagerCapability(msg.cap_flags),
            gimbal_device_id=msg.gimbal_device_id,
            roll_min=msg.roll_min,
            roll_max=msg.roll_max,
            pitch_min=msg.pitch_min,
            pitch_max=msg.pitch_max,
            yaw_min=msg.yaw_min,
            yaw_max=msg.yaw_max,
        )
        self.gimbal_managers[mavnode] = gimbal_manager

    def _recv_gimbal_manager_status(self, msg: mavlink2.MAVLink_gimbal_manager_status_message):
        mavnode = _MavlinkNode(system_id=msg.get_srcSystem(), cmp_id=msg.get_srcComponent())
        manager_status = _GimbalManagerStatus(
            system_id=mavnode.system_id,
            component_id=mavnode.component_id,
            flags=_GimbalManagerFlags(msg.flags),
            gimbal_device_id=msg.gimbal_device_id,
            primary_control_sysid=msg.primary_control_sysid,
            primary_control_compid=msg.primary_control_compid,
            secondary_control_sysid=msg.secondary_control_sysid,
            secondary_control_compid=msg.secondary_control_compid,
        )
        self.gimbal_manager_status[mavnode] = manager_status

    def _recv_gimbal_device_attitude_status(self, msg: mavlink2.MAVLink_gimbal_device_attitude_status_message):
        mavnode = _MavlinkNode(system_id=msg.get_srcSystem(), cmp_id=msg.get_srcComponent())
        gimbal_status = _GimbalStatus(
            system_id=mavnode.system_id,
            component_id=mavnode.component_id,
            flags=_GimbalDeviceFlags(msg.flags),
            q=(msg.q[0], msg.q[1], msg.q[2], msg.q[3]),
            angular_velocity_x=msg.angular_velocity_x,
            angular_velocity_y=msg.angular_velocity_y,
            angular_velocity_z=msg.angular_velocity_z,
            failure_flags=_GimbalDeviceErrorFlags(msg.failure_flags)
        )
        self.gimbal_status[mavnode] = gimbal_status


    def start(self):
        self.mavlink_pub: rospy.Publisher = rospy.Publisher("mavlink/to", Mavlink, queue_size=1)
        self.mavlink_sub: rospy.Subscriber = rospy.Subscriber(self.topic, self.TopicType, self._mavlink_sub_callback)
        self.worker_thread.start()

    def _mavlink_sub_callback(self, msg):
        msg_bytes: bytearray = mavros.mavlink.convert_to_bytes(msg)
        mavmsg = decode_mavlink2(msg_bytes)
        # If we're receiving a message that we care about for to gimbal operations
        if mavmsg.get_msgId() in MAVLINK_GIMBAL_MSG_IDS:
            self.cmd_queue.put(mavmsg)
    
    def _find_gimbals(self):
        # The protocol for "discovery of gimbal managers" is documented here:
        # https://mavlink.io/en/services/gimbal_v2.html#discovery-of-gimbal-manager
        # In summary, we need to broadcast a command that tells all the gimbal managers we want 
        # information. The gimbal managers respond so we can know their component IDs and
        # capabilities.

        # To find the gimbal manager We need to create a COMMAND_LONG message
        # https://mavlink.io/en/messages/common.html#COMMAND_LONG

        # And our COMMAND_LONG needs to specify MAV_CMD_REQUEST_MESSAGE as the command
        # https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE

        ADDRESS_OF_REQUESTOR = 1 # constant for param7 of MAV_CMD_REQUEST_MESSAGE
        request_cmd = mavlink2.MAVLink_command_long_message(
            target_system=0, # broadcast to all systems
            target_component=0, # broadcast to all components
            command=mavlink2.MAV_CMD_REQUEST_MESSAGE, # this command is a request message
            confirmation=0,
            param1=mavlink2.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION, # the message we request
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=ADDRESS_OF_REQUESTOR # the Response Target
        )

        request_cmd.pack(mavutil.mavlink.MAVLink("", 2, 1))
        ros_msg = mavros.mavlink.convert_to_rosmsg(request_cmd)
        self.mavlink_pub.publish(ros_msg)
    
    def _recv_mavlink(self, msg: mavlink2.MAVLink_message):
        msg_id = msg.get_msgId()
        recv_method = self._mav_msg_dispatch[msg_id]
        recv_method(msg)
    
    def _run(self):
        while not rospy.is_shutdown():
            cmd: _GimbalCommand = self.cmd_queue.get()
            if cmd.command_type == _CmdType.EXIT:
                return
            elif cmd.command_type == _CmdType.FIND_GIMBALS:
                self._find_gimbals()
            elif cmd.command_type == _CmdType.RECV_MAVLINK:
                self._recv_mavlink()


    

mavlink_from_meta = SensorMeta('mavlink_from', 'mavlink/from', Mavlink)

def find_gimbal_managers(pub: rospy.Publisher, sub: rospy.Subscriber, wait_time:float = 4.0):
    """Finds the gimbal managers
    Parameters:
        pub should be 
    """
    pass
