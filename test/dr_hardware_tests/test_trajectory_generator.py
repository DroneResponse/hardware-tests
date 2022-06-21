from builtins import round
from unittest import mock
from unittest.mock import NonCallableMock, Mock, patch
import unittest

import numpy as np

from droneresponse_mathtools import Lla, geoid_height
from dr_hardware_tests.trajectory_generator import TrajectoryFunction, TrajectoryGenerator, TrajectorySender, amsl_to_ellipsoid, ellipsoid_to_amsl

class TestTrajectoryGenerator(unittest.TestCase):
    def assertLlaAlmostEqual(self, a: Lla, b: Lla):
        self.assertAlmostEqual(a.distance(b), 0.0, 2)

    def test_TrajectoryGenerator1(self):
        """
        This test is inspired by the first maneuver in the box test. In the box test 
        we fly 25 meters west and 3 meters up. But for now, I'm going to ignore the change in
        altitude. So we're only flying 25 meters west.

        Given:
            jerk limit = 4 m/s^3
            acceleration limit = 2 m/s^2
            speed limit = 5 m/s
            position limit = 25 m

        Find s(t)
        """
        trajectory_factory = TrajectoryGenerator(
            max_horizontal_acceleration=2.0,
            max_up_acceleration=2.0,
            max_down_acceleration=2.0,
            max_jerk=4.0)

        # Start position at the ND testing field:
        elevation_amsl = 220.5533
        start_pos_amsl = Lla(41.71484711400101, -86.24179959559793,
                             elevation_amsl + 10.0)
        start_pos = amsl_to_ellipsoid(start_pos_amsl)

        stop_pos = start_pos.move_ned(0, -25, 0)
        stop_pos_amsl = ellipsoid_to_amsl(stop_pos)

        speed_limit = 5.0
        s, duration = trajectory_factory.make(start_pos, stop_pos, speed=speed_limit)

        self.assertLlaAlmostEqual(start_pos_amsl, s(0))
        self.assertLlaAlmostEqual(stop_pos_amsl, s(duration))

        # time step is 0.05
        # if we make it too small the test fails because jerk is allowed to change instantaneously.
        # Our algorithm sees this as acceleration changing too quickly (when it's not)
        t_plot = np.arange(0, duration + 0.1, 0.05) 
        pos_plot = [amsl_to_ellipsoid(s(t)) for t in t_plot]

        # Make sure we don't move faster than the speed limit
        speed_plot = []
        for i, _ in enumerate(t_plot[:-1]):
            dt = t_plot[i+1] - t_plot[i]
            displacement = pos_plot[i+1].distance(pos_plot[i])
            average_speed = displacement / dt
            speed_plot.append(round(average_speed, 3))
            self.assertTrue(average_speed <= speed_limit + 0.001) 
        
        # Make sure we reach max speed
        self.assertIn(5.0, speed_plot)

        # Make sure acceleration doesn't go above 2.0
        acceleration_plot = np.zeros(len(speed_plot) - 1)
        for i, average_speed in enumerate(speed_plot[:-1]):
            dt = t_plot[i+1] - t_plot[i]
            delta_speed = speed_plot[i+1] - average_speed
            a = abs(delta_speed) / dt
            
            # we allow for numerical inaccuracies (we cannot control the drone with millimeter accuracy anyway)
            a = round(a, 3) 
            
            self.assertLessEqual(a, 2.0)                
            acceleration_plot[i] = a
        
        # Make sure we reach max acceleration
        self.assertIn(2.0, acceleration_plot)

        jerk_plot = np.zeros(len(acceleration_plot) - 1)
        # Make sure jerk doesn't go above 4.0
        for i, _ in enumerate(acceleration_plot[:-1]):
            dt = t_plot[i+1] - t_plot[i]
            delta_acceleration = acceleration_plot[i+1] - acceleration_plot[i]
            print(delta_acceleration)
            average_jerk = abs(delta_acceleration) / dt
            average_jerk = round(average_jerk, 3)
            
            self.assertLessEqual(average_jerk, 4.0, f"jerk is too big at t={t_plot[i]}")

            jerk_plot[i] = average_jerk
        
        # make sure we reach max jerk
        self.assertIn(4.0, jerk_plot)

