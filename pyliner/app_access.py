"""
The Vehicle Access module provides a token-ish interface for a component to
interact with a vehicle, and the other components of the vehicle.

Classes:
    AppAccess  Access controller between a Vehicle and its components.
"""

from pyliner.intent import Intent, Broadcaster
from pyliner.intent import IntentFilter
from pyliner.intent import IntentFuture, IntentResponse
from pyliner.pyliner_error import PylinerError
from pyliner.util import Loggable


__all__ = ['InvalidStateError', 'AppDetachedError', 'AppAccess']


class InvalidStateError(PylinerError):
    """Raised if continuing would perform an illegal state transition."""
    pass


class AppDetachedError(InvalidStateError):
    """Raised if an App attempts to do something that is invalid because it is
    detached from a vehicle."""
    pass


class AppAccess(Loggable, Broadcaster):
    """
    The AppAccess class acts as a boundary between Apps on a vehicle so that
    they can not interact destructively with each other. Apps must access the
    vehicle and other Apps through the interface provided by their AppAccess.
    """
    def __init__(self, app):
        super(AppAccess, self).__init__()
        self.app = app
        self.callback = None

        self._vehicle = None
        """:type: BaseVehicle"""
        self._filter = {}
        """:type: dict[IntentFilter, Callable[[Intent], Any]"""

    def attach(self, vehicle):
        """Attach the App to a Vehicle."""
        self._vehicle = vehicle
        self.logger = vehicle.logger.getChild(self.app.qualified_name)
        self.info('Attaching {}'.format(self.app.qualified_name))
        self.app.attach(self)

    def detach(self):
        """Detach the App from the Vehicle it was previously attached to."""
        self.info('Detaching {}'.format(self.app.qualified_name))
        self.clear_filter()
        self.app.detach()
        self._vehicle = None
        self.logger = None

    def add_filter(self, intent_filter, callback):
        """Add an intent filter.

        Args:
            intent_filter (IntentFilter): The filter to apply to incoming
                intents.
            callback (Callable[[Intent], Any]: If an incoming intent passes the
                filter, the callback will be run with the intent as an argument.

        Returns:
            IntentFilter: Same object as intent_filter.
        """
        if not isinstance(intent_filter, IntentFilter):
            raise TypeError('intent_filter must be IntentFilter.')
        if not callable(callback):
            raise TypeError('callback must be callable.')
        self._filter[intent_filter] = callback
        self._vehicle.add_filter(intent_filter, self)
        return intent_filter

    def broadcast(self, intent):
        # type: (Intent) -> IntentFuture
        """Broadcast an intent to the vehicle."""
        if not self._vehicle:
            raise AppDetachedError('Cannot broadcast while detached.')
        if not intent.origin:
            intent.origin = self.app.qualified_name
        return self._vehicle.broadcast(intent)

    def clear_filter(self):
        """Remove all intent filters."""
        for filter_pair in list(self._filter.keys()):
            self.remove_filter(filter_pair)

    def receive(self, intent, future):
        """Receive an intent and pass it to any callbacks that are listening.

        Any callback return that is not None is passed back to the App that
        broadcast the Intent.

        If callback is set, it will be called only if the intent did not match
        any other set filters.

        Args:
            intent (Intent): The intent that was broadcast.
            future (IntentFuture): The Future to return results into.
        """
        def handle(cb):
            result = exception = None
            try:
                result = cb(intent)
            except Exception as e:
                exception = e
                self.exception('While handling {}, an Exception was raised:'
                               .format(intent))
            if result is not None or exception is not None:
                future.add(IntentResponse(result=result, exception=exception))

        handled = False
        for intent_filter, callback in self._filter.items():
            if intent in intent_filter:
                handle(callback)
                handled = True
        if not handled and callable(self.callback):
            handle(self.callback)

    @property
    def shutdown(self):
        return self._vehicle.is_shutdown

    def remove_filter(self, intent_filter):
        """Remove a filter from this access."""
        del self._filter[intent_filter]
        self._vehicle.remove_filter(intent_filter, self)
