#
# export PYTHONPATH="$PYTHONPATH:$PWD/src:$PWD/test:$PWD"
from threading import Event
import time
from unittest import mock
from unittest.mock import NonCallableMock, Mock, patch
import unittest

import dr_hardware_tests.HeartbeatSender as HBModule
from dr_hardware_tests.HeartbeatSender import HeartbeatSender, MavType


class CalledNumberOfTimesEvent:
    def __init__(self, n: int):
        self.call_count = 0
        self.n = n
        self.event = Event()
    def on_call(self, msg):
        self.call_count += 1
        if self.call_count >= self.n:
            self.event.set()

class CalledWithMavstateEvent:
    def __init__(self):
        self.event = Event()
    def on_call(self, msg: HBModule.mavlink2.MAVLink_heartbeat_message):
        if msg.system_status == HBModule.MavState.ACTIVE:
            self.event.set()

def mock_sleep():
    time.sleep(0.001)

class TestHeartbeatSender(unittest.TestCase):
    @patch('dr_hardware_tests.HeartbeatSender.rospy')
    def test_the_basics(self, rospy):
        rospy.is_shutdown = Mock(return_value=False)
        mock_rate = rospy.Rate()
        mock_rate.sleep = Mock(side_effect=mock_sleep)

        called_event = CalledNumberOfTimesEvent(3)
        mock_mavlink_sender = NonCallableMock()
        mock_mavlink_sender.send = Mock(side_effect=called_event.on_call)

        hb = HeartbeatSender(mock_mavlink_sender, component_type=MavType.ONBOARD_CONTROLLER)

        hb.start()
        called_event.event.wait()
        hb.stop()
    
    @patch('dr_hardware_tests.HeartbeatSender.rospy')
    def test_changing_mavstate(self, rospy):
        rospy.is_shutdown = Mock(return_value=False)
        mock_rate = rospy.Rate()
        mock_rate.sleep = Mock(side_effect=mock_sleep)

        called_event = CalledWithMavstateEvent()
        mock_mavlink_sender = NonCallableMock()
        mock_mavlink_sender.send = Mock(side_effect=called_event.on_call)

        hb = HeartbeatSender(mock_mavlink_sender, component_type=MavType.ONBOARD_CONTROLLER)
        hb.start()
        mock_sleep()
        current_state = hb.mav_state
        self.assertEqual(current_state, HBModule.MavState.UNINIT)
        mock_sleep()
        hb.mav_state = HBModule.MavState.ACTIVE
        mock_sleep()
        called_event.event.wait()
        current_state = hb.mav_state
        self.assertEqual(current_state, HBModule.MavState.ACTIVE)
        hb.stop()



if __name__ == '__main__':
    unittest.main()