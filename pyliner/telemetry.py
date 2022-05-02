"""
The Telemetry module provides a simple process for defining new telemetry types
and setting arbitrary telemetry items.

Classes:
    ManualSetpoint  A ManualSetpoint command.
    SetpointTriplet  A SetpointTriplet command.
    Telemetry  Base class for any Telemetry. A dictionary.
"""

from pyliner import util


# TODO Put into Communication (package)
# TODO Remove all of this, change to wrapper of python_pb protobuf objects.
class Telemetry(dict):
    """Subclass of dict representing FSW telemetry objects.

    Call to_json() to convert this object to a JSON formatted dictionary that
    can be sent to the FSW.

    Keys of the dictionary are argument names. The values can be either
    primitives (ex. int, float) or a callable that takes no arguments which
    will be invoked when to_json is called.
    """

    def __init__(self, name, **kwargs):
        super(Telemetry, self).__init__(**kwargs)
        self.name = name

    def __repr__(self):
        return '{}({}, {})'.format(
            self.__class__.__name__, repr(self.name),
            ', '.join(('{}={}'.format(*item) for item in self.items())))

    def __setitem__(self, key, value):
        if key not in self:
            raise KeyError('Cannot add additional attributes to telemetry. '
                           'Failed to add {}'.format(key))
        super(Telemetry, self).__setitem__(key, value)

    @property
    def has_args(self):
        return len(self) is not 0

    def to_json(self):
        json = {'name': self.name}
        if len(self):
            json['args'] = [{'name': name,
                             'value': value() if callable(value) else value}
                            for name, value in self.items()]
        return json


class ManualSetpoint(Telemetry):
    def __init__(self, **kwargs):
        super(ManualSetpoint, self).__init__(
            '/Airliner/PX4/ManualSetpoint',
            Timestamp=util.get_time,
            X=0.0,
            Y=0.0,
            Z=0.0,
            R=0.0,
            Flaps=0.0,
            Aux1=0.0,
            Aux2=0.0,
            Aux3=0.0,
            Aux4=0.0,
            Aux5=0.0,
            ModeSwitch=0,
            ReturnSwitch=0,
            RattitudeSwitch=0,
            PosctlSwitch=0,
            LoiterSwitch=0,
            AcroSwitch=0,
            OffboardSwitch=0,
            KillSwitch=0,
            TransitionSwitch=0,
            GearSwitch=0,
            ArmSwitch=0,
            StabSwitch=0,
            ManSwitch=0,
            ModeSlot=0,
            DataSource=0)
        for key, value in kwargs.items():
            self[key] = value


class SetpointTriplet(Telemetry):
    def __init__(self, **kwargs):
        super(SetpointTriplet, self).__init__(
            '/Airliner/PX4/SetpointTriplet',
            Timestamp=util.get_time,
            Prev_Lat=0.0,
            Prev_Lon=0.0,
            Prev_X=0.0,
            Prev_Y=0.0,
            Prev_Z=0.0,
            Prev_VX=0.0,
            Prev_VY=0.0,
            Prev_VZ=0.0,
            Prev_Alt=0.0,
            Prev_Yaw=0.0,
            Prev_Yawspeed=0.0,
            Prev_LoiterRadius=0.0,
            Prev_PitchMin=0.0,
            Prev_AX=0.0,
            Prev_AY=0.0,
            Prev_AZ=0.0,
            Prev_AcceptanceRadius=0.0,
            Prev_CruisingSpeed=0.0,
            Prev_CruisingThrottle=0.0,
            Prev_Valid=0,
            Prev_Type=0,
            Prev_PositionValid=0,
            Prev_VelocityValid=0,
            Prev_YawValid=0,
            Prev_DisableMcYawControl=0,
            Prev_YawspeedValid=0,
            Prev_LoiterDirection=0,
            Prev_AccelerationValid=0,
            Prev_AccelerationIsForce=0,
            Prev_VelocityFrame=0,
            Cur_Lat=0.0,
            Cur_Lon=0.0,
            Cur_X=0.0,
            Cur_Y=0.0,
            Cur_Z=0.0,
            Cur_VX=0.0,
            Cur_VY=0.0,
            Cur_VZ=0.0,
            Cur_Alt=0.0,
            Cur_Yaw=0.0,
            Cur_Yawspeed=0.0,
            Cur_LoiterRadius=0.0,
            Cur_PitchMin=0.0,
            Cur_AX=0.0,
            Cur_AY=0.0,
            Cur_AZ=0.0,
            Cur_AcceptanceRadius=0.0,
            Cur_CruisingSpeed=0.0,
            Cur_CruisingThrottle=0.0,
            Cur_Valid=0,
            Cur_Type=0,
            Cur_PositionValid=0,
            Cur_VelocityValid=0,
            Cur_YawValid=0,
            Cur_DisableMcYawControl=0,
            Cur_YawspeedValid=0,
            Cur_LoiterDirection=0,
            Cur_AccelerationValid=0,
            Cur_AccelerationIsForce=0,
            Cur_VelocityFrame=0,
            Next_Lat=0.0,
            Next_Lon=0.0,
            Next_X=0.0,
            Next_Y=0.0,
            Next_Z=0.0,
            Next_VX=0.0,
            Next_VY=0.0,
            Next_VZ=0.0,
            Next_Alt=0.0,
            Next_Yaw=0.0,
            Next_Yawspeed=0.0,
            Next_LoiterRadius=0.0,
            Next_PitchMin=0.0,
            Next_AX=0.0,
            Next_AY=0.0,
            Next_AZ=0.0,
            Next_AcceptanceRadius=0.0,
            Next_CruisingSpeed=0.0,
            Next_CruisingThrottle=0.0,
            Next_Valid=0,
            Next_Type=0,
            Next_PositionValid=0,
            Next_VelocityValid=0,
            Next_YawValid=0,
            Next_DisableMcYawControl=0,
            Next_YawspeedValid=0,
            Next_LoiterDirection=0,
            Next_AccelerationValid=0,
            Next_AccelerationIsForce=0,
            Next_VelocityFrame=0)
        for key, value in kwargs.items():
            self[key] = value
