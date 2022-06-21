import enum
import time
from typing import Callable
from dataclasses import dataclass
from queue import Queue
from threading import Event, Thread

import rospy
from droneresponse_mathtools import Lla, geoid_height
from mavros_msgs.srv import ParamGet
from ruckig import InputParameter, Result, Ruckig, Trajectory

from dr_hardware_tests.Drone import Drone
from dr_hardware_tests.SetpointSender import SetpointSender


def amsl_to_ellipsoid(pos: Lla) -> Lla:
    """
    Return the Lla with its altitude specified as meters above the ellipsoid 
    Given an Lla with it's altitude specified as meters above mean sea level (AMSL)
    """
    amsl_alt = pos.altitude
    geoid_separation = geoid_height(pos.latitude, pos.longitude)
    ellipsoid_alt = amsl_alt + geoid_separation
    return Lla(pos.latitude, pos.longitude, ellipsoid_alt)


def ellipsoid_to_amsl(pos: Lla) -> Lla:
    """
    Return the Lla with its altitude specified as meters above mean sea level
    Given an Lla with it's altitude specified as meters above the ellipsoid
    """
    wgs84_alt = pos.altitude
    geoid_separation = geoid_height(pos.latitude, pos.longitude)
    amsl_alt = wgs84_alt - geoid_separation
    return Lla(pos.latitude, pos.longitude, amsl_alt)


# TrajectoryFunction returns the drone's position along the path as a function of time.
# Where the time is given as seconds since the maneuver started.
# The position returned has its altitude specified as meters above mean sea level
# By convention, instances of this function are often called "s"
TrajectoryFunction = Callable[[float], Lla]


class TrajectoryGenerator:
    def __init__(self,
            max_horizontal_acceleration: float,
            max_up_acceleration: float,
            max_down_acceleration: float,
            max_jerk: float
        ) -> None:
        """
        Builds a trajectory generator.

        Read px4 parameters to set these arguments:
            max_horizontal_acceleration = MPC_ACC_HOR
            max_up_acceleration = MPC_ACC_UP_MAX
            max_down_acceleration = MPC_ACC_DOWN_MAX
            max_jerk = MPC_JERK_AUTO

        """
        self.max_horizontal_acceleration = abs(max_horizontal_acceleration)
        self.max_up_acceleration = abs(max_up_acceleration)
        self.max_down_acceleration = abs(max_down_acceleration)
        self.max_jerk = abs(max_jerk)
    
    def make(self, start_pos: Lla, stop_pos: Lla, speed: float) -> TrajectoryFunction:
        """
        Make a TrajectoryFunction, s(t).

        The TrajectoryFunction returns the drone's position along the path as a function of time.
        Where the time is given as seconds since the maneuver started. The position returned
        specifies its altitude in meters above mean sea level

        Arguments:
            start_pos: Lla of the drone's position when the maneuver starts with
                it's altitude specified as meters above the ellipsoid
            stop_pos: Lla of the destination with altitude given as meters above
                the ellipsoid
            speed: how fast the drone should fly in meters per second

        
        """
        inp = InputParameter(3)

        inp.current_position = [0.0, 0.0, 0.0]
        inp.current_velocity = [0.0, 0.0, 0.0]
        inp.current_acceleration = [0.0, 0.0, 0.0]

        stop_ned = start_pos.distance_ned(stop_pos)
        stop_north = stop_ned[0]
        stop_east = stop_ned[1]
        stop_down = stop_ned[2]
        inp.target_position = [stop_north, stop_east, stop_down]
        inp.target_velocity = [0.0, 0.0, 0.0]
        inp.target_acceleration = [0.0, 0.0, 0.0]

        # set this to the speed you wish to fly at
        # in this case we want to fly at 5 meters per second
        inp.max_velocity = [speed, speed, speed]

        # get horizontal acceleration limit from MPC_ACC_HOR (this is 3 by default and could be 2 - 15)
        # for upward acceleration use MPC_ACC_UP_MAX (4 by default) (could be 2 - 15)
        # inp.max_acceleration = [3.0, 3.0, 4.0]
        inp.max_acceleration = [
            self.max_horizontal_acceleration,
            self.max_horizontal_acceleration,
            self.max_down_acceleration
        ]
        inp.min_acceleration = [
            -1.0 * self.max_horizontal_acceleration,
            -1.0 * self.max_horizontal_acceleration,
            -1.0 * self.max_up_acceleration
        ]

        # for jerk we want to use MPC_JERK_AUTO (this is 4 by default)
        inp.max_jerk = [
            self.max_jerk,
            self.max_jerk,
            self.max_jerk
        ]

        otg = Ruckig(3)
        trajectory = Trajectory(3)

        # Calculate the trajectory in an offline manner
        calc_result = otg.calculate(inp, trajectory)
        if calc_result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')
        

        def s(time: float):
            # Then, we can calculate the kinematic state at a given time
            new_position, new_velocity, new_acceleration = trajectory.at_time(time)
            north = new_position[0]
            east = new_position[1]
            down = new_position[2]

            target_lla_ellipsoid = start_pos.move_ned(north, east, down)
            target_lla_amsl = ellipsoid_to_amsl(target_lla_ellipsoid)
            return target_lla_amsl
        
        return s, trajectory.duration


def make_TrajectoryGenerator(drone: Drone):
    def read_param(param_name):
        result: ParamGet = drone.get_param(param_name)
        if not result.success:
            raise Exception(f"could not read PX4 parameter {param_name}")
        return result.value.real
    
    max_horizontal_acceleration = read_param("MPC_ACC_HOR")
    max_up_acceleration = read_param("MPC_ACC_UP_MAX")
    max_down_acceleration = read_param("MPC_ACC_DOWN_MAX")
    max_jerk = read_param("MPC_JERK_AUTO")

    return TrajectoryGenerator(max_horizontal_acceleration, max_up_acceleration, max_down_acceleration, max_jerk)


class _Message(enum.Enum):
    SEND_NOW = enum.auto()
    STOP = enum.auto()


class TrajectorySender:
    def __init__(self, s: TrajectoryFunction, duration: float, setpoint_sender: SetpointSender,  send_frequency: float = 20):
        self._s = s
        self._duration = duration
        self._setpoint_sender = setpoint_sender
        self._send_frequency = send_frequency
        self._message_queue: Queue = Queue()
        self._stop_event = Event()

        self._update_thread = Thread(target=self._run)
        self._timer_thread = Thread(target=self._run_timer)

    def start(self):
        self._update_thread.start()
        self._timer_thread.start()

    def stop(self, await_stop=True):
        self._stop_event.set() # stops the timer
        if not self._update_thread.is_alive():
            return
        self._message_queue.put(_Message.STOP)
        if await_stop:
            self._update_thread.join()
    
    def join(self):
        self._update_thread.join()

    def _run(self):
        def stop_callback():
            self.stop(await_stop=False)

        rospy.on_shutdown(stop_callback)

        start_time = time.time()
        t = 0
        self._setpoint_sender.setpoint = self._s(t)
        while not rospy.is_shutdown() and t < self._duration:
            message: _Message = self._message_queue.get()
            if message == _Message.STOP:
                break
            elif message == _Message.SEND_NOW:
                t = time.time() - start_time
                self._setpoint_sender.setpoint = self._s(t)
            else:
                rospy.logerr(f"Unknown message type {message}.")
        if not self._stop_event.is_set():
            self._stop_event.set()   
    
    def _run_timer(self):
        message = _Message.SEND_NOW
        rate = rospy.Rate(self._send_frequency)
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                if self._update_thread.is_alive():
                    self._message_queue.put(message)
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
