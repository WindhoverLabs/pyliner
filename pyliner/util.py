"""
The util module contains a lot of random code that doesn't belong anywhere else
yet. If enough code of a similar purpose is added here it should be moved into
its own package or module.

All code in util is self-sufficient, relying only on Python builtins or
documented dependencies to perform properly. No code in this module uses any
Pyliner-specific data.

Methods:
    enable_logging  Begins logging the current process.
    get_time  Get the current time.
    handler_factory  Create a UDP receive handler factory.
    indent  Yield prepended spaces from an iterable.
    init_socket  Create a UDP socket.
    query_yes_no  Ask the user a yes/no question.
    read_json  Read the json from the given file path into a dictionary.
    shifter  Yield an n-tuple of an iterable as it is left-shifted.

Classes:
     CallableDefaultDict  A reimplementation of defaultdict that calls a
        handler with the new key as an argument.
     Loggable  Mixin class that provides basic logging methods.
     OverlayDict  Dict where key lookups search multiple dicts.
     RealTimeOverrun  Raised when a RealTimeThread took too long to execute.
     RealTimeThread  Thread which calls its target periodically.
     StreamLogger  Intercepts IO to a stream and logs it.
     ThreadedUDPRequestHandler  Sends UDP requests to a custom callback.
"""

import json
import logging
import socket
import sys
import threading
import time
try:
    from collections.abc import Iterable  # python >= version 3.10
except ImportError:
    from collections import Iterable  # python < version 3.10
from datetime import datetime
from os import mkdir
from os.path import join, isdir, exists

# from orderedset import OrderedSet
import queue
import socketserver

from pyliner.pyliner_error import PylinerError


class CallableDefaultDict(dict):
    """A reimplementation of defaultdict that calls a handler with the
    new key as an argument.
    """

    def __init__(self, default_factory=None, **kwargs):
        super(CallableDefaultDict, self).__init__(**kwargs)
        self.default_factory = default_factory

    def __getitem__(self, item):
        if item not in self:
            self[item] = self.default_factory(item)
        return super(CallableDefaultDict, self).__getitem__(item)


class Loggable(object):
    """A mixin class that provides basic logging methods.

    Subclasses that mix this must set logger before calling any of the logging
    methods or they will get AttributeErrors.
    """

    def __init__(self, logger=None):
        self.logger = logger

    def critical(self, msg, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs)

    def exception(self, msg, *args, **kwargs):
        self.logger.exception(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs)


class OrderedSetQueue(queue.Queue):
    """Reimplementation of Queue with a backing OrderedSet.

    Allows use of the in-operator.
    """

    def __contains__(self, item):
        return item in self.queue

    def _init(self, maxsize):
        # FIXME: Implement with PriorityQueue
        pass
        # self.queue = OrderedSet()

    def _get(self):
        # FIXME: Implement with PriorityQueue
        # return self.queue.pop(last=False)
        pass

    def _put(self, item):
        # FIXME: Implement with PriorityQueue
        # self.queue.add(item)
        pass

    def remove(self, elem):
        # FIXME: Implement with PriorityQueue
        # self.queue.remove(elem)
        pass


class OverlayDict(object):
    """Limited dict implementation that allows for fallback on key lookup."""

    def __init__(self, *layers):
        self.layers = list(layers)

    def __getitem__(self, item):
        for layer in self.layers:
            if item in layer:
                return layer[item]
        raise KeyError(item)


class RealTimeOverrun(PylinerError):
    """Raised when a RealTimeThread took too long to execute."""


class RealTimeThread(threading.Thread):
    """Executes a callback function every x-seconds.

    Optionally takes callbacks for exception and last-pass handling.

    Exceptions may occur in Python 2.7 during interpreter shutdown, as threads
    are left alive until the process closes, during which time the thread is
    attempting to access garbage collected objects.
    """

    def __init__(self, target, every=1, args=(), kwargs=None, exception=None,
                 finalize=None, name=None, logger=None, daemon=True):
        """
        Args:
            target (Callable): This method will be called with no arguments
                continuously until stopped.
            every (Real): Seconds between calls to target.
            args (Optional[list]): If given, these args are passed to target.
            kwargs (Optional[dict]): If given, these kwargs are passed to
                target.
            exception (Callable): If an exception is raised this will
                be called with the exception as an argument.
            finalize (Callable): This method will be called with no arguments
                after the thread is stopped.
            name (str): The name of the thread. Default is autogenerated.
            logger (Logger): Logger to use. Default logging.getLogger(self.name)
            daemon (bool): Whether the thread is a daemon or not.
        """
        super(RealTimeThread, self).__init__(
            target=target, name=name, args=args, kwargs=kwargs)
        self.daemon = daemon  # Attribute of Thread

        # Instance Attributes
        self.exception = exception if exception else \
            lambda e: self.logger.exception(
                'Unhandled exception in thread %s', self.name)
        self.every = every
        self.finalize = finalize
        self.logger = logger if logger else logging.getLogger(self.name)
        self.running = False
        self.target = target
        self.args = args
        if kwargs is None:
            self.kwargs = dict()
        else:
            self.kwargs = kwargs

    def run(self):
        """Do not call this directly. Use PeriodicExecutor.start()"""
        self.logger.info('Thread %s starting', self.name)
        self.running = True
        start_time = time.time()
        try:
            while self.running:
                self.target(*self.args, **self.kwargs)
                # self._Thread__target(
                #     *self._Thread__args, **self._Thread__kwargs)
                start_time += self.every
                sleep_time = start_time - time.time()
                if sleep_time < 0:
                    raise RealTimeOverrun(
                        '{} took too long ({}s)'
                        .format(self.name, self.every - sleep_time))
                time.sleep(sleep_time)
        except Exception as e:
            if callable(self.exception):
                self.exception(e)
        finally:
            if callable(self.finalize):
                self.finalize()
            self.stop()

    def stop(self):
        """Stops the thread from continuing to loop.

        Thread will not exit immediately. When the thread wakes up next it will
        see that it has been stopped, will execute the finalize method if it was
        given, and will then exit.
        """
        self.logger.info('Thread %s stopping.', self.name)
        self.running = False


class StreamLogger(object):
    """Intercepts IO to a stream and logs it."""

    def __init__(self, name, stream, level=logging.INFO):
        self.level = level
        self.name = name
        self.stream = stream
        self.log = logging.getLogger(name)

    def flush(self):
        return self.stream.flush()

    def read(self, length):
        result = self.stream.read(length)
        self.log.log(self.level, '(read) %s', repr(result))
        return result

    def readline(self):
        result = self.stream.readline()
        self.log.log(self.level, '(readline) %s', repr(result))
        return result

    def write(self, s):
        if not s.isspace():
            self.log.log(self.level, '(write) %s', repr(s))
        self.stream.write(s)


class ThreadedUDPRequestHandler(socketserver.BaseRequestHandler):
    def __init__(self, callback, *args, **keys):
        self.callback = callback
        socketserver.BaseRequestHandler.__init__(self, *args, **keys)

    def handle(self):
        self.callback(self.request)


def enable_logging(log_dir=None, log_file=None, script=None, level=logging.INFO,
                   filemode='a', stdin=logging.INFO, stdout=logging.INFO,
                   stderr=logging.ERROR):
    """Enable the base logger, which all other logging bases off of.

    Logs stdin, stdout, and stderr if a logging level is passed to them.

    Args:
        log_dir (str): If None, defaults to 'logs'.
        log_file (str): If None, defaults to '<time>_pyliner_<script>.log'.
        script (str): Script name, only used if log_file is None.
        level: The minimum log level to capture.
        filemode: The filemode of the log file. Either 'a' or 'w'.
        stdin: Logging level or None to disable stdin logging.
        stdout: Logging level or None to disable stdout logging.
        stderr: Logging level or None to disable stderr logging.
    """
    if log_dir is None:
        log_dir = 'logs'
    if log_file is None:
        now = datetime.now()
        time_str = now.strftime('%Y-%m-%d_%H:%M:%S')
        log_file = '{}_pyliner_{}.log'.format(time_str, script)

    log_path = join(log_dir, log_file)
    if not isdir(log_dir):
        mkdir(log_dir)

    print('Log at {}'.format(log_path))
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - '
               '%(threadName)s - %(message)s',
        filename=log_path,
        filemode=filemode
    )
    if stdin is not None:
        sys.stdin = StreamLogger('stdin', sys.stdin, stdin)
    if stdout is not None:
        sys.stdout = StreamLogger('stdout', sys.stdout, stdout)
    if stderr is not None:
        sys.stderr = StreamLogger('stderr', sys.stderr, stderr)


def get_time():
    return int(
        (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds() * 10000)


def handler_factory(callback):
    """ Creates server object and sets the callback """
    return lambda *args, **kwargs: ThreadedUDPRequestHandler(callback, *args,
                                                             **kwargs)


def indent(level, iterable):
    """Indent every line in a string by `level` spaces."""
    for string in iterable:
        yield ' ' * level + str(string)


def init_socket():
    """ Creates a UDP socket object and returns it """
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def query_yes_no(question, default=None, **extra):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.

    Returns:
        True if yes, False if no.
    Adapted from: http://code.activestate.com/recipes/577058/
    """
    valid = {True: {'y', 'ye', 'yes'},
             False: {'n', 'no'}}
    valid.update(extra)
    if default is None:
        prompt = " [y/n] "
    elif default:
        prompt = " [Y/n] "
    else:
        prompt = " [y/N] "

    while True:
        sys.stdout.write(question + prompt)
        if sys.version_info.major < 3:
            choice = raw_input().lower()
        else:
            choice = input().lower()
        if default is not None and choice == '':
            return default
        for answer, options in valid.items():
            if choice in options:
                return answer
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")


def read_json(file_path):
    """Parses the required JSON input file containing Airliner mappings.

    Returns:
        dict
    """
    try:
        with open(file_path, 'r') as airliner_map:
            return json.load(airliner_map)
    except IOError:
        print("Specified input file (%s) does not exist" % file_path)
    except Exception as e:
        print(e)


def shifter(num, iterator):
    if not isinstance(iterator, Iterator) \
            and isinstance(iterator, Iterable):
        iterator = iter(iterator)
    output = tuple(next(iterator) for _ in range(num))
    yield output
    for item in iterator:
        output = tuple(output[index + 1] if index < num - 1 else item
                       for index in range(num))
        yield output
