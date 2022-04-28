from threading import Event, Lock, Thread
import time
import sys
from typing import Tuple
from dr_hardware_tests.Drone import Drone

import rospy

from .sensor import RospyShutdownException, SensorSynchronizer
from .flight_predicate import is_data_available, is_off_ground, is_posctl_mode
from .flight_predicate import is_user_taking_control


def _quit_on_rc_trigger(sensors: SensorSynchronizer, failsafe_engaged_event: Event):
    try:
        rospy.loginfo("RC failsafe waiting for data")
        sensors.await_condition(is_data_available)

        rospy.loginfo("RC failsafe engaged")
        failsafe_engaged_event.set()
        sensors.await_condition(is_user_taking_control)
    except RospyShutdownException:
        return
    rospy.logfatal("Humans have taken control of the drone")
    rospy.signal_shutdown("Humans control the drone")


def start_drone_io() -> Tuple[Drone, SensorSynchronizer]:
    """Create the objects needed to interact with the drone.

    This creates and starts a Drone object that you can use to control the drone.
    This also creates a sensor synchronizer that lets you access sensor data.

    Returns a tuple holding the drone and sensor synchronizer (in that order)
    """
    drone: Drone = Drone()
    drone.start()
    sensors: SensorSynchronizer = SensorSynchronizer()
    sensors.start()
    return drone, sensors


def start_RC_failsafe(sensors: SensorSynchronizer):
    failsafe_engaged_event = Event()
    thread = Thread(target=_quit_on_rc_trigger, args=[sensors, failsafe_engaged_event])
    thread.start()
    failsafe_engaged_event.wait()


class _AutomicBool:
    def __init__(self, initial_value: bool):
        self.lock = Lock()
        self._value = initial_value
    
    @property
    def value(self) -> bool:
        with self.lock:
            return self._value
    @value.setter
    def value(self, new_value: bool):
        with self.lock:
            self._value = new_value


def sleep(sleep_time: float):
    """sleep for the time specified in seconds
    """

    done_event = Event()
    sleep_successful = _AutomicBool(False)

    class SleeperThread(Thread):
        def run(self) -> None:
            time.sleep(sleep_time)
            sleep_successful._value = True
            done_event.set()
    
    def shutdown_handler():
        done_event.set()
    
    thread = SleeperThread()
    thread.daemon = True
    thread.start()

    rospy.on_shutdown(shutdown_handler)

    done_event.wait()

    if not sleep_successful.value:
        raise rospy.ROSInterruptException("the program shutdown before done sleeping")
