"""
The controller module exposes a Controller App for performing high-level vehicle
state management.

Enums:
    FlightMode  Use to send the flight software a mode.

Apps:
    Controller  Sends high-level one-off vehicle management commands.

"""
import time

from enum import Enum

from pyliner.action import ACTION_RTL, ACTION_SEND_COMMAND, ACTION_ARM, \
    ACTION_DISARM, ACTION_TAKEOFF, ACTION_ATP, ACTION_SEND_TELEMETRY
from pyliner.app import App
from pyliner.command import Arm, Disarm, Takeoff, AutoRtl, PosCtl
from pyliner.intent import Intent, IntentFilter
from pyliner.pyliner_error import PylinerError
from pyliner.telemetry import ManualSetpoint
from pyliner.util import query_yes_no

_WAIT_TIME = 0.1


class UnauthorizedAtpError(PylinerError):
    """Raised if the user denies ATP."""
    pass


class FlightMode(Enum):
    """Use to select a mode for the flight software."""
    Manual = 1
    AltCtl = 2
    PosCtl = 3
    RTL = 4


class Controller(App):
    """Sends high-level one-off commands to the vehicle.

    Controller listens to a multiple intent actions. If it receives any of the
    following actions it will perform the appropriate action:
        ACTION_ARM
        ACTION_DISARM
        ACTION_RTL
        ACTION_TAKEOFF

    Each of these actions take no data and do not provide any feedback for when
    the action has completed.

    The only other action that is listens to is ACTION_ATP, which prompts the
    user over the command line whether to allow a certain action to proceed.
    The intent should provide a prompt to the user as a string in the intent's
    data attribute. This App will return True or False to the caller depending
    on the user's choice.
    """

    def __init__(self):
        super(Controller, self).__init__()
        self.atp_override = None

    def attach(self, vehicle_wrapper):
        super(Controller, self).attach(vehicle_wrapper)
        # TODO FLIGHT_MODE
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_ARM]), lambda i: self.arm())
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_ATP]),
            lambda i: self.atp(i.data, False))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_DISARM]), lambda i: self.disarm())
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_RTL]), lambda i: self.rtl())
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_TAKEOFF]), lambda i: self.takeoff())

    def detach(self):
        self.vehicle.callback = None
        super(Controller, self).detach()

    def arm(self):
        """Arm vehicle."""
        print("Arming vehicle")
        self.vehicle.info("Arming vehicle")
        with self.control_block() as block:
            block.broadcast(Intent(
                action=ACTION_SEND_COMMAND,
                data=block.request(Arm()))).first()

    def atp(self, text, error=True):
        """Collect authorization to proceed (ATP) from the user."""
        atp_auth = None
        while atp_auth is None:
            if callable(self.atp_override):
                atp_auth = self.atp_override()
            elif self.atp_override is not None:
                atp_auth = self.atp_override
            else:
                atp_auth = query_yes_no(
                    "Requires authorization for: {}".format(text))
            self.vehicle.info("ATP: {} Auth: {}".format(text, atp_auth))
            if error and atp_auth is False:
                raise UnauthorizedAtpError
            return atp_auth

    def disarm(self):
        print("Disarming vehicle")
        self.vehicle.info("Disarming vehicle")
        with self.control_block() as block:
            block.broadcast(Intent(
                action=ACTION_SEND_COMMAND,
                data=block.request(Disarm()))).first()

    def flight_mode(self, mode):
        if not mode:
            raise ValueError('Mode transition requires a passed mode.')
        if mode == FlightMode.Manual:
            raise NotImplemented()
        elif mode == FlightMode.AltCtl:
            raise NotImplemented()
        elif mode == FlightMode.PosCtl:
            self._mode_posctl()

    def _mode_posctl(self):
        print("Position control")
        self.vehicle.info("Position control")
        with self.control_block() as block:
            block.broadcast(Intent(
                action=ACTION_SEND_COMMAND,
                data=block.request(
                    PosCtl())
            )).first()

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.apps.controller'

    @classmethod
    def required_telemetry_paths(cls):
        return None

    def rtl(self):
        """Return to launch."""
        print("RTL")
        self.vehicle.info("RTL")
        with self.control_block() as block:
            block.broadcast(Intent(
                action=ACTION_SEND_TELEMETRY,
                data=block.request(
                    AutoRtl())
            )).first()

    def takeoff(self):
        """Takeoff"""
        print("Auto takeoff")
        self.vehicle.info("Auto takeoff")
        with self.control_block() as block:
            block.broadcast(Intent(
                action=ACTION_SEND_COMMAND,
                data=block.request(
                    Takeoff())
            )).first()
        time.sleep(5)

    def takeoff(self):
        """Takeoff"""
        print("Auto takeoff")
        self.vehicle.info("Auto takeoff")
        with self.control_block() as block:
            block.broadcast(Intent(
                action=ACTION_SEND_COMMAND,
                data=block.request(
                    Takeoff())
            )).first()
        time.sleep(5)
