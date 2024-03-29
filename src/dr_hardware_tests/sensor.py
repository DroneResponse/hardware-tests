from collections.abc import Mapping, MutableMapping
from dataclasses import dataclass, field
from queue import Empty, Queue
from threading import Event, Lock, Thread
from typing import Callable, Iterable
import copy
import dataclasses
import enum
import sys

from diagnostic_msgs.msg import DiagnosticArray
from genpy import message
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, WaypointList, RCIn
import rospy
from rospy import client
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64


@dataclass(frozen=True)
class SensorMeta:
    name: str
    topic: str
    TopicType: object = field(hash=False, repr=False, compare=False)

    def make_sub(self, callback) -> rospy.Subscriber:
        return rospy.Subscriber(self.topic, self.TopicType, callback)


MAVROS_SENSORS = {
    SensorMeta("battery", "mavros/battery", BatteryState),
    SensorMeta("diagnostics", "/diagnostics", DiagnosticArray),
    SensorMeta("estimator_status", "mavros/estimator_status", EstimatorStatus),
    SensorMeta("extended_state", "mavros/extended_state", ExtendedState),
    SensorMeta("imu", "mavros/imu/data", Imu),
    SensorMeta("position", "mavros/global_position/global", NavSatFix),
    SensorMeta("relative_altitude", "mavros/global_position/rel_alt", Float64),
    SensorMeta("state", "mavros/state", State),
    SensorMeta("velocity", "mavros/local_position/velocity_local", TwistStamped),
    SensorMeta("geofence", "mavros/geofence/waypoints", WaypointList),
    SensorMeta("rcin", "mavros/rc/in", RCIn),
}


@dataclass(frozen=True)
class SensorData:
    battery: BatteryState = None
    diagnostics: DiagnosticArray = None
    estimator_status: EstimatorStatus = None
    extended_state: ExtendedState = None
    imu: Imu = None
    position: NavSatFix = None
    relative_altitude: Float64 = None
    state: State = None
    velocity: TwistStamped = None
    geofence: WaypointList = None
    rcin: RCIn = None


SensorTest = Callable[[SensorData], bool]


class RospyShutdownException(Exception):
    """This exception is raised when a function cannot finish because rospy shutting down
    """
    pass


class _MessageFlag(enum.Enum):
    EXIT = enum.auto()
    NEW_SENSOR_DATA = enum.auto()
    NEW_CLIENT = enum.auto()
    DATA_REQUEST = enum.auto()


@dataclass
class _SynchronizerClient:
    name: int
    test_func: SensorTest
    return_channel: Queue = field(default_factory=lambda: Queue())


@dataclass(frozen=True)
class _Message:
    flag: _MessageFlag
    new_data: Mapping = None
    client: _SynchronizerClient = None
    data_channel: Queue = None


class _ClientReturnMessage(enum.Enum):
    EXIT = enum.auto()
    SUCCESS = enum.auto()


class AtomicCounter:
    def __init__(self):
        self.lock = Lock()
        self._next = 0

    def next(self):
        with self.lock:
            result = self._next
            self._next = self._next + 1
            return result


class SensorSynchronizer:
    """Keeps track of sensor state. Allows clients to block until some condition is met

    The main method for clients is await_condition. You pass in a function that takes SensorData as
    an argument and it returns true when the condition is met. This method will block unit the
    condition is true.

    """
    def __init__(self):
        self.data: SensorData = SensorData()
        self._queue = Queue()
        self.thread = Thread(target=self._run)
        self._is_shutdown = Event()
        self.clients: MutableMapping[int, _SynchronizerClient] = {}

        # for making clients
        self.client_counter = AtomicCounter()

    def start(self, sensors: Iterable[SensorMeta] = MAVROS_SENSORS):
        self.thread.start()

        for sensor_meta in sensors:
            sensor_callback = self._make_subscriber_callback(sensor_meta.name)
            sensor_meta.make_sub(sensor_callback)

    def update_sensor_data(self, name, data):
        sensor_data = {}
        sensor_data[name] = data
        message = _Message(flag=_MessageFlag.NEW_SENSOR_DATA,
                           new_data=sensor_data)
        self._queue.put(message)

    def sensor_data(self) -> SensorData:
        data_channel = Queue()
        message = _Message(flag=_MessageFlag.DATA_REQUEST, data_channel=data_channel)
        self._queue.put(message)
        return data_channel.get()

    def await_condition(self, func: SensorTest, timeout=None):
        """block until func returns True or rospy tries to shutdown
        """
        client = self._make_client(func)
        message = _Message(flag=_MessageFlag.NEW_CLIENT, client=client)
        self._queue.put(message)
        try:
            result: _ClientReturnMessage = client.return_channel.get(timeout=timeout)
            if result == _ClientReturnMessage.EXIT:
                raise RospyShutdownException()
        except Empty:
            raise TimeoutError("sensor test timeout. The condition was never met")

    def _run(self):
        rospy.on_shutdown(self._on_shutdown)

        while not self._is_shutdown.is_set():
            message: _Message = self._queue.get()
            if message.flag == _MessageFlag.NEW_SENSOR_DATA:
                self._update(**message.new_data)
                self._update_clients()
            elif message.flag == _MessageFlag.NEW_CLIENT:
                client_name = message.client.name
                self.clients[client_name] = message.client
                self._update_clients()
            elif message.flag == _MessageFlag.EXIT:
                for client in self.clients.values():
                    client.return_channel.put(_ClientReturnMessage.EXIT)
                self.clients.clear()
            elif message.flag == _MessageFlag.DATA_REQUEST:
                result = copy.copy(self.data)
                message.data_channel.put(result)

    def _on_shutdown(self):
        self._is_shutdown.set()
        shutdown_message = _Message(flag=_MessageFlag.EXIT)
        self._queue.put(shutdown_message)

    def _update(self, **sensor_data):
        self.data = dataclasses.replace(self.data, **sensor_data)

    def _update_clients(self):
        all_clients = list(self.clients.values())
        for client in all_clients:
            self._check_client(client)

    def _check_client(self, client: _SynchronizerClient):
        test_result = False
        try:
            test_result = client.test_func(self.data)
        except Exception as e:
            rospy.logwarn(f"SensorSynchronizer client function raised {e}")
        if test_result:
            client.return_channel.put(_ClientReturnMessage.SUCCESS)
            del self.clients[client.name]

    def _make_client(self, test_func: SensorTest) -> _SynchronizerClient:
        client_id = self.client_counter.next()
        return _SynchronizerClient(name=client_id, test_func=test_func)

    def _make_subscriber_callback(self, sensor_name: str):
        def callback_func(message):
            self.update_sensor_data(sensor_name, message)

        return callback_func
