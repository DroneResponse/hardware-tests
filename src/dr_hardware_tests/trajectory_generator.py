import time

from concurrent.futures import thread
from threading import Event, Thread
from droneresponse_mathtools import Lla, Pvector
from dr_hardware_tests.SetpointSender import SetpointSender
from mavros_msgs.srv import ParamGet
from ruckig import InputParameter, Ruckig, Trajectory, Result

from nodes.box import ellipsoid_to_amsl

from .Drone import Drone


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
    
    def make(self, start_pos: Lla, stop_lla: Lla, speed: float):
        """
        Make a function s(t)

        It returns the setpoint LLA given time since we started the maneuver in seconds
        """
        inp = InputParameter(3)

        inp.current_position = [0.0, 0.0, 0.0]
        inp.current_velocity = [0.0, 0.0, 0.0]
        inp.current_acceleration = [0.0, 0.0, 0.0]

        stop_ned = start_pos.distance_ned(stop_lla)
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
        inp.min_velocity = [
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

            target_lla = start_pos.move_ned(north, east, down)
            return target_lla
        
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

def fly_trajectory(s, duration: float, setpoint_sender: SetpointSender):
    result = Event()
    def fly():
        start_time = time.time()
        end_time = start_time + duration

        t = 0
        while start_time + t < end_time:
            lla = s(t)
            setpoint_sender.lla = ellipsoid_to_amsl(lla)
            time.sleep(0.05)
            t = time.time() - start_time
        result.set()            

    thread = Thread(target=fly)
    thread.start()