from threading import Event, Lock, Thread
import time

import rospy


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