# Set the MAVLink Version to 2.0
import os
from typing import Union

from mavros import mavlink; os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink1
from pymavlink.dialects.v20 import common as mavlink2

_mav1 = mavlink1.MAVLink(file=None)
_mav2 = mavlink2.MAVLink(file=None)

def decode_mavlink1(buff: Union[bytes, bytearray]) -> mavlink1.MAVLink_message:
    return _mav1.decode(buff)

def decode_mavlink2(buff: Union[bytes, bytearray]) -> mavlink2.MAVLink_message:
    return _mav2.decode(buff)
