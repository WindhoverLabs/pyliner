from datetime import datetime
from threading import Event

from pyliner.action import ACTION_RTL, ACTION_TELEM
from pyliner.intent import Intent


# class _AppWrapper(object):
#     def __init__(self, vehicle, mapping):
#         self.vehicle = vehicle
#         self.mapping = mapping
#
#     def __getattr__(self, item):
#         try:
#             return lambda *args, **kwrags: self.mapping[item]()


class ScriptingWrapper(object):
    """Wraps a vehicle for a scripting environment.

    Implements context manager (with-statement) functionality. Unhandled
    exceptions in the context manager are passed to `failure_callback` to
    give the user a chance to fail gracefully before exiting the context
    manager.

    Apps on the vehicle are directly accessible by their qualified name,
    and Apps which are explicitly supported by this wrapper are accessible
    through shorter names.

    Examples:
        with ScriptingWrapper(vehicle) as rocky:
            ... do things
    """
    def __init__(self, vehicle, failure_callback=None):
        """
        Args:
            vehicle (BasePyliner): The vehicle to wrap.
            failure_callback (Callable[[Pyliner, Tuple], None]): Function
                handle that will be invoked on an unhandled exception of the
                controlling script.
        """
        self._vehicle = vehicle
        # TODO if no failure callback, define RTL instead of using if-else
        # in __exit__.
        self.failure_callback = failure_callback
        self.mapping = {
            'ctrl': 'com.windhover.pyliner.apps.controller',
            'fd': 'com.windhover.pyliner.apps.flight_director',
            'fence': 'com.windhover.pyliner.apps.geofence',
            'geographic': 'com.windhover.pyliner.apps.geographic',
            'nav': 'com.windhover.pyliner.apps.navigation',
            'com': 'com.windhover.pyliner.apps.communication'
        }
        # self.mapping = {
        #     'nav': _AppWrapper(vehicle, {
        #         ''
        #     })
        # }

    def __getattr__(self, item):
        if hasattr(self._vehicle, item):
            return getattr(self._vehicle, item)
        try:
            return self.app_by_qualified_name(self.mapping[item])
        except KeyError:
            raise AttributeError('{!r} is not a mapped App of this '
                                 'vehicle.'.format(item))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # If the context manager exits without error, all good. Otherwise...
        if exc_type:
            if callable(self.failure_callback):
                self.failure_callback(self, (exc_type, exc_val, exc_tb))
            else:
                print('Error in execution. Returning to Launch.')
                self._vehicle.broadcast(Intent(action=ACTION_RTL))

    def app_by_qualified_name(self, qualified_name):
        return self._vehicle.apps[qualified_name].app

    def await_change(self, tlm, out=None, poll=1.0, timeout=360):
        """Block until the telemetry gets a new value.

        The new value does not have to be different than the old value.

        Args:
            tlm (str): The telemetry to monitor.
            out (Callable): If not None, call this every loop.
            poll (float): Check every `poll` seconds.
        """

        telemetry = self.telemetry(tlm)
        change = Event()
        telemetry.add_listener(lambda t: change.set())
        change.wait(poll)
        start_time = datetime.now()
        while not change.is_set():
            if (datetime.now() - start_time).seconds > timeout:
                return False
            if callable(out):
                out()
            elif isinstance(out, str):
                print(out)
            change.wait(poll)
        
        return True

    def telemetry(self, op_path):
        return self._vehicle.broadcast(Intent(
            action=ACTION_TELEM, data=op_path
        )).first().result
