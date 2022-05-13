from enum import Enum


class MessageType(Enum):
    COMMAND = 1
    TELEMETRY = 2


# TODO Put into Communication (package)
# TODO Remove all of this, change to wrapper of python_pb protobuf objects.
class Message(dict):
    """Subclass of dict representing FSW message objects.

    Call to_json() to convert this object to a JSON formatted dictionary that
    can be sent to the FSW.

    Keys of the dictionary are argument names. The values can be either
    primitives (ex. int, float) or a callable that takes no arguments which
    will be invoked when to_json is called.
    """

    msg_type: MessageType

    def __init__(self, name, **kwargs):
        super(Message, self).__init__(**kwargs)
        self.name = name

    def __repr__(self):
        return '{}({}, {})'.format(
            self.__class__.__name__, repr(self.name),
            ', '.join(('{}={}'.format(*item) for item in self.items())))

    def __setitem__(self, key, value):
        if key not in self:
            raise KeyError('Cannot add additional attributes to telemetry. '
                           'Failed to add {}'.format(key))
        super(Message, self).__setitem__(key, value)

    @property
    def message_type(self):
        return self.msg_type

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
