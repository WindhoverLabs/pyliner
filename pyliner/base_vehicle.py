"""
The Base Vehicle module exposes the base class for Vehicle. A lot of the setup
for intra-vehicle communication occurs here.

Classes:
    BaseVehicle  The base class for Vehicle
"""
import atexit
import logging
from abc import ABCMeta
from collections import defaultdict

from pyliner.action import ACTION_VEHICLE_SHUTDOWN, ACTION_APP_ATTACH, \
    ACTION_APP_DETACH, ACTION_APP_LIST
from pyliner.app_access import AppAccess
from pyliner.app import App
from pyliner.intent import Intent, IntentNoReceiverError, IntentExplicitFailure, \
    Broadcaster
from pyliner.intent import IntentFilter
from pyliner.intent import IntentFuture
from pyliner.util import Loggable


class BaseVehicle(Loggable, Broadcaster):
    """
    Contains the bare-minimum required for Pyliner. All additional functionality
    is provided through the Vehicle class. App management is performed via the
    app-lifecycle methods.
    """
    __metaclass__ = ABCMeta

    def __init__(self, vehicle_id, logger=None):
        """Constructor for BaseVehicle.

        Args:
            vehicle_id: Vehicle ID. Should be unique.
            logger: If None, defaults to 'logging.getLogger(vehicle_id)'.
        """
        logging.basicConfig()
        super(BaseVehicle, self).__init__(
            logger=logger or logging.getLogger(vehicle_id))

        # Register Shutdown
        atexit.register(self.shutdown)

        # Instance attributes
        self.apps = {}
        """:type: dict[str, AppAccess]"""
        self.is_shutdown = False
        self.vehicle_id = vehicle_id

        # self._broadcast_pool = Pool() # TODO Python 3
        # self._dynamic_filters = set()
        self._intent_filters = defaultdict(lambda: set())
        """:type: dict[str, set[AppAccess]]"""

        # Register self App
        self.attach_app(VehicleApp())

    def add_filter(self, intent_filter, app):
        # type: (IntentFilter, AppAccess) -> None
        """Add an intent filter to this vehicle.

        Args:
            intent_filter (IntentFilter): The filter to add.
            app (AppAccess): If an intent matches a filter, the app to call.
        """
        for action in intent_filter.actions:
            self._intent_filters[action].add(app)

    def attach_app(self, app):
        """Attach an app to this vehicle.

        Args:
            app (App): The app to attach to this vehicle.
        """
        if app.qualified_name in self.apps:
            raise ValueError('Attempting to enable an App with the same name as'
                             ' an existing app.')
        elif not isinstance(app, App):
            return TypeError('app must be a subclass of App.')

        vehicle_token = AppAccess(app)
        self.apps[app.qualified_name] = vehicle_token
        vehicle_token.attach(self)

    def broadcast(self, intent):
        # type: (Intent) -> IntentFuture
        """Broadcast an Intent to listening Apps."""
        future = IntentFuture(caused_by=intent)
        # TODO Multithreading is a headache
        # threading.Thread(target=self._broadcast_thread, args=(intent, future))\
        #     .start()
        self._broadcast_thread(intent, future)
        return future

    def _broadcast_thread(self, intent, future):
        # type: (Intent, IntentFuture) -> None
        """Called by broadcast. Broadcasts intents to listening Apps.

        May be in a separate thread.
        """
        self.debug('Broadcasting: ' + str(intent))
        if intent.is_explicit():
            try:
                self.apps[intent.component].receive(intent, future)
            except KeyError:
                future.failure = IntentExplicitFailure(
                    'There is no App with the name: ' + str(intent.component))
        else:
            try:
                intent_filters = self._intent_filters[intent.action]
            except KeyError:
                future.failure = IntentNoReceiverError(
                    'There are no Apps accepting {}.'.format(intent.action))
            else:
                for app in intent_filters:
                    app.receive(intent, future)
        future.complete = True

    def detach_app(self, name):
        """Disable an app by removing it from this vehicle.

        Note:
            Any apps that attempt to call the disabled app by name will
            raise a KeyError.
        """
        try:
            app = self.apps[name]
        except KeyError:
            raise KeyError('Cannot find App to detach.')
        del self.apps[name]
        app.detach()

    def remove_filter(self, intent_filter, app_access):
        """Remove an intent filter from this vehicle."""
        for action in intent_filter.actions:
            self._intent_filters[action].remove(app_access)

    def shutdown(self):
        """Shutdown all components on vehicle and detach.

        This method is normally registered to be called automatically at
        interpreter exit.

        Shutdown Order:
            1. Broadcast Shutdown
            2. Wait for broadcast to finish.
            3. Detach Apps
        """
        self.info('Vehicle {} is shutting down.'.format(self.vehicle_id))
        if not self.is_shutdown:
            self.is_shutdown = True
            self.broadcast(Intent(action=ACTION_VEHICLE_SHUTDOWN)).wait()
            for app in self.apps.values():
                app.detach()
        self.info('Shutdown complete.')


class VehicleApp(App):
    """VehicleApp handles simple administrative tasks.

    Attach App:
        This App responds to ACTION_APP_ATTACH, with the new App to attach as
        its data.

    Detach App:
        This App responds to ACTION_APP_DETACH by detaching the App with the
        qualified name in data.

    App List:
        This App will return a list of all enabled Apps on the vehicle when it
        receives an intent with the ACTION_APP_LIST action.
    """
    def attach(self, vehicle):
        super(VehicleApp, self).attach(vehicle)
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_APP_ATTACH]),
            lambda i: self.vehicle._vehicle.attach_app(i.data))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_APP_DETACH]),
            lambda i: self.vehicle._vehicle.detach_app(i.data))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_APP_LIST]),
            lambda i: self.vehicle._vehicle.apps.keys())

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.vehicle_app'
