import dataclasses
import enum
import math

from dataclasses import dataclass
from queue import Queue
from threading import Thread, Event
from typing import Tuple
from genpy import message

import rospy
from droneresponse_mathtools import Lla, Position

from .Drone import Drone
from .flight_helpers import compass_to_enu_up


class _Flag(enum.Enum):
    GET_SETPOINT = enum.auto()
    SET_SETPOINT = enum.auto()
    GET_VELOCITY = enum.auto()
    SET_VELOCITY = enum.auto()
    GET_YAW = enum.auto()
    SET_YAW = enum.auto()
    SEND_NOW = enum.auto()
    STOP = enum.auto()

@dataclass(frozen=True)
class _Message:
    flag: _Flag
    setpoint: Position = None
    velocity: Tuple[float, float, float] = None
    yaw: float = None
    data_channel: Queue = None
    done_event: Event = None



class SetpointSender:
    def __init__(self, drone: Drone, send_frequency:float=25.0):
        self._drone: Drone = drone
        self._message_queue:Queue = Queue()
        self._setpoint: Lla = None
        self._velocity: Tuple[float, float, float] = None
        self._yaw: float = 0.0
        self._mode = None
        self._sender_thread = Thread(target=self._run)

        self._timer_thread = Thread(target=self._run_timer)
        self._send_frequency = send_frequency

    
    def start(self):
        self._sender_thread.start()
        self._timer_thread.start()

    def stop(self, await_stop=True):
        if not self._sender_thread.is_alive():
            return
        message = _Message(flag=_Flag.STOP)
        self._message_queue.put(message)
        if await_stop:
            self._sender_thread.join()

    @property
    def setpoint(self) -> Lla:
        result_channel = Queue()
        message = _Message(flag=_Flag.GET_SETPOINT, data_channel=result_channel)
        self._message_queue.put(message)
        return result_channel.get(timeout=1.0)
    
    @setpoint.setter
    def setpoint(self, position: Position):
        lla = None
        if position is not None:
            lla = position.to_lla()
        done_event = Event()
        message = _Message(flag=_Flag.SET_SETPOINT, setpoint=lla, done_event=done_event)
        self._message_queue.put(message)
        done_event.wait()
    
    @property
    def velocity(self) -> Tuple[float, float, float]:
        result_channel = Queue()
        message = _Message(flag=_Flag.GET_VELOCITY, data_channel=result_channel)
        self._message_queue.put(message)
        return result_channel.get(timeout=1.0)
    
    @velocity.setter
    def velocity(self, ned: Tuple[float, float, float]):
        done_event = Event()
        message = _Message(flag=_Flag.SET_VELOCITY, velocity=ned, done_event=done_event)
        self._message_queue.put(message)
        done_event.wait()
    
    @property
    def yaw(self) -> float:
        result_channel = Queue()
        message = _Message(flag=_Flag.GET_YAW, data_channel=result_channel)
        self._message_queue.put(message)
        return result_channel.get(timeout=1.0)

    @yaw.setter
    def yaw(self, yaw_degrees: float):
        if yaw_degrees is None:
            raise TypeError("yaw_degrees cannot be none")
        
        yaw_degrees = float(yaw_degrees)
        if yaw_degrees <= -180.0 or yaw_degrees > 360.0:
            rospy.logwarn(f"Setting yaw to: {yaw_degrees} degrees.")
        else:
            rospy.loginfo(f"Setting yaw to: {yaw_degrees} degrees.")
        yaw_degrees = compass_to_enu_up(yaw_degrees)
        yaw = math.radians(yaw_degrees)
        done_event = Event()
        message = _Message(flag=_Flag.SET_YAW, yaw=yaw, done_event=done_event)
        self._message_queue.put(message)
        done_event.wait()

    def _run(self):
        def stop_callback():
            self.stop(await_stop=False)

        rospy.on_shutdown(stop_callback)
        
        while not rospy.is_shutdown():
            message: _Message = self._message_queue.get()

            if message.flag == _Flag.STOP:
                return

            elif message.flag == _Flag.SET_SETPOINT:
                new_setpoint = message.setpoint
                if new_setpoint is not None:
                    new_setpoint = new_setpoint.to_lla()
                self._setpoint = new_setpoint
                self._velocity = None
                
            elif message.flag == _Flag.GET_SETPOINT:
                getter_return_value = None
                if self._setpoint is not None:
                    # copy the value
                    lat, lon, alt = self._setpoint.lat, self._setpoint.lon, self._setpoint.alt
                    getter_return_value = Lla(lat, lon, alt)
                message.data_channel.put(getter_return_value)
            
            elif message.flag == _Flag.SET_VELOCITY:
                self._velocity = message.velocity
                self._setpoint = None
            
            elif message.flag == _Flag.GET_VELOCITY:
                getter_return_value = None
                if self._velocity is not None:
                    # copy it
                    getter_return_value = tuple(self._velocity)
                message.data_channel.put(getter_return_value)
            
            elif message.flag == _Flag.GET_YAW:
                message.data_channel.put(self._yaw)
            
            elif message.flag == _Flag.SET_YAW:
                self._yaw = message.yaw
                
            elif message.flag == _Flag.SEND_NOW:
                if self._setpoint is not None:
                    self._drone.send_setpoint(self._setpoint, yaw=self._yaw)
                elif self._velocity is not None:
                    self._drone.send_velocity(self._velocity, yaw=self._yaw)
            
            if message.done_event is not None:
                message.done_event.set()


    def _run_timer(self):
        message = _Message(flag=_Flag.SEND_NOW)
        rate = rospy.Rate(self._send_frequency)
        while not rospy.is_shutdown():
            try:
                if self._sender_thread.is_alive():
                    self._message_queue.put(message)
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        

