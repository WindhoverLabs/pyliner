"""
The intent module provides all the classes and exceptions relevant to
broadcasting Intents across a Vehicle.
"""

from threading import Event

from pyliner.pyliner_error import PylinerError


class FutureTimeoutError(PylinerError):
    """Raised if an IntentFuture did not receive a reply within the
    specified timeout."""
    pass


class IntentExplicitFailure(PylinerError):
    """Raised if an explicit Intent was not broadcast because there was no App
    on-board the vehicle that matched the Intent.component."""


class IntentNoReceiverError(PylinerError):
    """Raised if an Intent was not broadcast because no Apps indicated they were
     capable of receiving the specified action."""
    pass


class Broadcaster(object):
    def broadcast(self, intent):
        """Broadcast an intent.

        Returns:
            An IntentFuture that holds the state of the broadcast intent.
        """
        raise NotImplementedError


class Intent(object):
    """An Intent is an abstraction of an operation to be performed.

    Intents are broadcast vehicle-wide or to single Apps to indicate that an
    action should be taken.

    See Also:
        https://developer.android.com/reference/android/content/Intent
    """
    def __init__(self, action, data=None, component=None):
        self.action = action
        """:type: str
        The action that this Intent would like to have performed. Handlers
        listen to certain kinds of actions that they can perform, and if there
        is a matching Intent.action broadcast the Intent is passed on to the
        handler.
        """
        self.component = component
        """:type: str
        If the Intent is explicit in its destination this is set to the 
        qualified name of the intended App.
        """
        self.data = data
        """:type: Any
        The data that this Intent is passing on to any handlers.
        """
        self.origin = None
        """:type: str
        If set, the qualified name of the App that generated this Intent.
        """

    def __repr__(self):
        s = '{}(origin={!r}, action={!r}, data={!r}'.format(
            self.__class__.__name__, self.origin, self.action, self.data)
        s += '' if not self.component else ' component=' + repr(self.component)
        s += ')'
        return s

    def is_explicit(self):
        return bool(self.component)


class IntentFilter(object):
    """Filters intents by matching actions from a list.

    Match using the in-operator `intent in IntentFilter()`.
    """
    def __init__(self, actions=None):
        self.actions = actions

    def __str__(self):
        return '{}(actions={}'.format(self.__class__.__name__, self.actions)
        # ', dynamic={}'.format(self.dynamic) if self.dynamic else '' + ')'

    def __contains__(self, intent):
        if not isinstance(intent, Intent):
            raise TypeError('IntentFilter can only contain Intents.')
        return intent.action in self.actions
        # or (self.dynamic(intent) if callable(self.dynamic) else False)


class IntentFuture(object):
    """Hold asynchronously-generated IntentResponses.

    When an Intent is broadcast the caller is given an IntentFuture which will
    be populated with IntentResponses as other Apps handle the Intent.
    """
    def __init__(self, caused_by):
        # type: (Intent) -> None
        self.caused_by = caused_by
        """:type: Intent
        The intent that generated this future. 
        """

        self.failure = None
        """:type: Exception
        Indicates a generic failure on Broadcasting an Intent. When possible
        this will be raised on the user thread but this is not guaranteed. The
        user should still explicitly check for this. 
        """

        self.responses = []
        """:type: list[IntentResponse]
        List of responses that this future has received. To avoid busy-loops
        while waiting for an initial response, the user should call .first() or 
        .recent() to block until a response has been received.
        """

        self._add_callback = None
        """:type: Callable[IntentFuture, Any]
        Called whenever a response has been received.
        """

        self._complete = Event()
        self._event_first = Event()

    def add(self, response):
        """Add a response to this Future. Call callback if set."""
        self.responses.append(response)
        self._event_first.set()
        if self._add_callback is not None:
            self._add_callback(self)

    def add_callback(self, callback):
        """Add a callback to be called whenever a new response is received.
        Callback is given this IntentFuture as its single argument.
        """
        if not callable(self._add_callback):
            raise TypeError('callback must be callable.')
        else:
            self._add_callback = callback

    @property
    def complete(self):
        """True if the future has been handled by every possible listener."""
        return self._complete.is_set()

    @complete.setter
    def complete(self, value):
        if value is not True:
            raise ValueError('IntentFuture.complete can only be set to True.')
        self._complete.set()

    def first(self, timeout=None):
        """Return the first response received.

        Raises:
            FutureTimeoutError: If no responses were received by the end
                of the timeout.
        """
        self._event_first.wait(timeout=timeout)
        try:
            return self.responses[0]
        except IndexError:
            raise self.failure or FutureTimeoutError()

    def recent(self, timeout=None):
        """Return the most recent response received.

        Raises:
            FutureTimeoutError: If no responses were received by the end
                of the timeout.
        """
        self._event_first.wait(timeout=timeout)
        try:
            return self.responses[-1]
        except IndexError:
            raise self.failure or FutureTimeoutError()

    def wait(self, timeout=None):
        """Block for timeout (or forever if None) until the IntentFuture is
        complete."""
        self._complete.wait(timeout)


class IntentResponse(object):
    """Wrapper for the response from an Intent.

    If the handling App processed the Intent properly, result will hold the
    returned value. If the handling App raised an exception during execution
    then exception will hold the exception that was raised.

    Attributes:
        result: The returned value from code that processed an Intent.
        exception: The exception (if any) that was raised while processing an
            Intent.
    """
    def __init__(self, origin=None, result=None, exception=None):
        self.exception = exception
        """:type: Exception
        The exception, if any, that the handling code for an Intent raised.
        """

        self.origin = origin
        """:type: str
        The qualified origin of the response.
        """

        self.result = result
        """:type: Any
        The returned result from the handling code for an Intent.
        """

    def __repr__(self):
        return '{}(origin={!r}, result={!r}, exception={!r})'.format(
            self.__class__.__name__, self.origin, self.result, self.exception)
