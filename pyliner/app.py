from abc import abstractmethod
from threading import Event

from pyliner.action import ACTION_CONTROL_GRANT, ACTION_CONTROL_REVOKE, \
    ACTION_CONTROL_REQUEST, ACTION_CONTROL_RELEASE
from pyliner.app_access import AppAccess, InvalidStateError, AppDetachedError
from pyliner.intent import IntentFilter, Intent, Broadcaster
from pyliner.pyliner_error import PylinerError
from pyliner.util import Loggable


class App(Loggable):
    """Apps respond to events and control the vehicle when appropriate.
    A Sensor is a long-running passive component of Pyliner.

    An app may be attached or detached at any time by the program. In the event
    that an app is detached, it should cease any auxiliary tasks by stopping
    threads or disabling event listeners.

    A app is given an AppAccess token when it is attached. This token is how
    the app broadcasts and receive intents from the vehicle it is attached to.

    Apps should be responsive. They can register for telemetry events and
    receive intents from other apps.

    If an app requests control of the vehicle it must wait until such control is
    granted before and commands that it sends are accepted.

    Lifecycle:
           +--------------------------+
           v                          |
        attach -> start <-> stop -> detach
    """

    ATTACHED = 'ATTACHED'
    DETACHED = 'DETACHED'

    def __init__(self):
        super(App, self).__init__()
        self._state = App.DETACHED
        self.vehicle = None
        """:type: AppAccess"""

    @property
    @abstractmethod
    def qualified_name(self):
        # type: () -> str
        """The system-unique identifier for this App.

        This name is used for explicit intent resolution.
        """
        raise NotImplementedError()

    @property
    def state(self):
        """The state of the App, ATTACHED or DETACHED."""
        return self._state

    def attach(self, vehicle_wrapper):
        # type: (AppAccess) -> None
        """Called when the app is attached to a vehicle.

        Call super before any subclass code.

        The app is given a wrapper to interact with the vehicle it has been
        attached to. Use this wrapper for platform-dependant operations.

        The app should start any threads or processes that it runs here.
        """
        if self._state is App.ATTACHED:
            raise InvalidStateError('Service is currently attached.')
        self._state = App.ATTACHED
        self.vehicle = vehicle_wrapper
        self.logger = vehicle_wrapper.logger

    def detach(self):
        """Called when the app is detached from a vehicle.

        Call super after any subclass code if the user code depends on vehicle
        or any logging calls.

        The wrapper will be valid until this method returns, after which the
        vehicle will no longer respond to calls through the wrapper nor pass
        events through to the app.

        The service should stop any threads or processes that it started.
        """
        if self._state is App.DETACHED:
            raise AppDetachedError('Service cannot be detached at this time.')
        self._state = App.DETACHED
        self.vehicle = None
        self.logger = None

    def control_block(self):
        """Return a ControlBlock context manager, which when entered, waits
        for vehicle control before broadcasting Intents.
        """
        if self.state is App.DETACHED:
            raise AppDetachedError('Detached App will never get control.')
        return _ControlBlock(self)


class _ControlBlock(Broadcaster):
    def __init__(self, app):
        self.app = app

        self._auth = None
        """:type: ControlToken"""
        self._control = Event()
        self._exit = False
        self._grant_filter = None
        self._revoke_filter = None

    def __enter__(self):
        # Add filters for control block listener.
        self._grant_filter = self.app.vehicle.add_filter(
            intent_filter=IntentFilter(actions=[ACTION_CONTROL_GRANT]),
            callback=lambda i: self.grant(i.data)
        )
        self._revoke_filter = self.app.vehicle.add_filter(
            intent_filter=IntentFilter(actions=[ACTION_CONTROL_REVOKE]),
            callback=lambda i: self.revoke()
        )

        # Send control request.
        print("Control block enter") + str(self.app.qualified_name)
        enqueue = self.app.vehicle.broadcast(Intent(
            action=ACTION_CONTROL_REQUEST,
            data=self.app.qualified_name
        )).first().result
        if not enqueue:
            raise PylinerError('Could not request control.')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._control.wait()
        self.app.vehicle.broadcast(Intent(
            action=ACTION_CONTROL_RELEASE,
            data=self._auth
        ))
        self.app.vehicle.remove_filter(self._grant_filter)
        self.app.vehicle.remove_filter(self._revoke_filter)

    def broadcast(self, intent):
        self._control.wait()
        return self.app.vehicle.broadcast(intent)

    def grant(self, data):
        if self._exit:
            return False
        else:
            self._auth = data
            self._control.set()
            return True

    def request(self, data):
        self._control.wait()
        return self._auth.request(data)

    def revoke(self):
        self._auth = None
        self._control.clear()
