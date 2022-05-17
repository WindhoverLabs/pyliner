"""
The Command module provides a simple process for defining new Command types
and setting arbitrary Command items.

Classes:
    Arm  Arm the vehicle.
    Disarm  A Disarm command.
    Command  Base class for any Command. A dictionary.
"""


# TODO Put into Communication (package)
# TODO Remove all of this, change to wrapper of python_pb protobuf objects.
from pyliner.message import MessageType


class Command(dict):
    """Subclass of dict representing FSW Command objects.

    Call to_json() to convert this object to a JSON formatted dictionary that
    can be sent to the FSW.

    Keys of the dictionary are argument names. The values can be either
    primitives (ex. int, float) or a callable that takes no arguments which
    will be invoked when to_json is called.
    """

    def __init__(self, name, **kwargs):
        super(Command, self).__init__(**kwargs)
        super(Command, self).msg_type = MessageType.COMMAND
        self.name = name

    def __repr__(self):
        return '{}({}, {})'.format(
            self.__class__.__name__, repr(self.name),
            ', '.join(('{}={}'.format(*item) for item in self.items())))

    def __setitem__(self, key, value):
        if key not in self:
            raise KeyError('Cannot add additional attributes to Command. '
                           'Failed to add {}'.format(key))
        super(Command, self).__setitem__(key, value)

    @property
    def has_args(self):
        return len(self) is not 0

    def to_dict(self):
        json = {'name': self.name}
        if len(self):
            json['args'] = [{'name': name,
                             'value': value() if callable(value) else value}
                            for name, value in self.items()]
        return json


class Arm(Command):
    def __init__(self, **kwargs):
        super(Arm, self).__init__(
            '/cfs/cpd/apps/vm/Arm',
        )
        for key, value in kwargs.items():
            self[key] = value


class Disarm(Command):
    def __init__(self, **kwargs):
        super(Disarm, self).__init__(
            '/cfs/cpd/apps/vm/Disarm',
        )
        for key, value in kwargs.items():
            self[key] = value
