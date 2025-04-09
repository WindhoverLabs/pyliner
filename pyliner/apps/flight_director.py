"""
The Flight Director app provides a Flight Director Application to control a
vehicle's basic axes of motion.

Apps:
    FlightDirector  Grants the user control of a vehicle's basic axes.
"""

from pyliner.app import App
from pyliner.action import ACTION_AXIS_SET, ACTION_SEND_COMMAND, \
    ACTION_AXIS_ZERO, ACTION_SEND_TELEMETRY
from pyliner.intent import Intent, IntentFilter
from pyliner.telemetry import ManualSetpoint


class FlightDirector(App):
    """The FlightDirector allows for manual, direct control of a vehicle's axes.

    The X, Y, Z, and Yaw (R) axes are controlled via a ManualSetpoint command.

    Axis Set:
        This app listens to ACTION_AXIS_SET intents, where the data attribute is
        a dictionary with keys ('x', 'y', 'z', or 'r') that map to the desired
        value for each of the axes. A value of None or an unset key will not
        be changed.

    Axis Zero:
        This app listens to ACTION_AXIS_ZERO intents, and when one is received
        it sets all axes to zero.
    """
    def __init__(self, strict_set=False):
        super(FlightDirector, self).__init__()
        self._x = self._y = self._z = self._r = 0.0
        self.strict_set = strict_set

    def attach(self, vehicle_wrapper):
        super(FlightDirector, self).attach(vehicle_wrapper)
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_AXIS_SET]),
            lambda i: self.set(**i.data)
        )
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_AXIS_ZERO]),
            lambda i: self.zero()
        )

    def detach(self):
        self.vehicle.callback = None
        super(FlightDirector, self).detach()

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.apps.flight_director'

    def _send_telemetry(self):
        """Generate a ManualSetpoint and transmit it."""
        mod_z = self.z / 2.0 + 0.5  # [-1, 1] -> [0, 1]
        # self.debug('Sending Telemetry: {} {} {} {}'.format(
        #     self.x, self.y, mod_z, self.r))
        with self.control_block() as block:
            block.broadcast(Intent(
                action=ACTION_SEND_TELEMETRY,
                data=block.request(ManualSetpoint(
                    X=self._x, Y=self._y, Z=mod_z, R=self._r,
                    PosctlSwitch=1, GearSwitch=1, ArmSwitch=1)))).first()

    def set(self, x=None, y=None, z=None, r=None):
        """Set multiple axes at the same time."""
        if x is not None:
            self._x_set(x)
        if y is not None:
            self._y_set(y)
        if z is not None:
            self._z_set(z)
        if r is not None:
            self._r_set(r)
        self._send_telemetry()

    def _set_check(self, axis, value):
        """If a value is invalid raise an exception, otherwise do nothing."""
        if not isinstance(value, float):
            raise TypeError('May only assign float to {}.'.format(axis))
        if self.strict_set and (value > 1 or value < -1):
            raise ValueError('Cannot assign {} outside of [-1, 1].'
                             .format(axis))

    def zero(self):
        """Zero out all axes."""
        self._x = self._y = self._z = self._r = 0.0
        self._send_telemetry()

    # The methods below deal with the nuances of having each of x, y, z, and r
    # as properties. It's basically the same 3 methods copied for each one.

    @property
    def r(self):
        return self._r

    @r.setter
    def r(self, value):
        self._r_set(value)
        self._send_telemetry()

    def _r_set(self, value):
        self._set_check('r', value)
        self._r = value

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x_set(value)
        self._send_telemetry()

    def _x_set(self, value):
        self._set_check('x', value)
        self._x = value

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y_set(value)
        self._send_telemetry()

    def _y_set(self, value):
        self._set_check('y', value)
        self._y = value

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        self._z_set(value)
        self._send_telemetry()

    def _z_set(self, value):
        self._set_check('z', value)
        self._z = value
