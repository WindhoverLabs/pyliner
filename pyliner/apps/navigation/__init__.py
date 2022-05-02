"""
The navigation package holds the builtin code for performing basic vehicle
navigation. Four types of navigation are provided by default, as well as a
Navigation app that users may attach to a vehicle.

Notes:
    Most navigation methods block until they are complete.

Apps:
    Navigation  Takes control of the vehicle to perform movements.

Navigation Helpers:
    Goto  Move the vehicle to a waypoint or along a series of waypoints.
    Lnav  Perform a lateral movement.
    Rotate  Rotate the vehicle.
    Vnav  Perform a vertical movement.

The following modules are also available:
    control  Collection of useful navigation PID controllers.
    heading  Implementation of custom float that wraps [0-360).
    navigation_factory  Base class for navigation helpers.
    position  2D and 3D point classes.
    waypoint  Waypoint used in navigation.
"""
import math

from pyliner.action import ACTION_TELEM, ACTION_GOTO
from pyliner.app import App
from pyliner.apps.navigation.goto import Goto
from pyliner.heading import Heading
from pyliner.apps.navigation.lnav import Lnav
from pyliner.apps.navigation.rotate import Rotate
from pyliner.apps.navigation.vnav import Vnav
from pyliner.position import Waypoint
from pyliner.intent import Intent, IntentFilter
from pyliner.conversions import hertz


class Navigation(App):
    """Navigation app. Contains all navigation features."""
    # TODO: In a future version a more integrated 3d-space will be implemented.
    # TODO Look into using decimal library for precision at any lat/lon

    def __init__(self):
        super(Navigation, self).__init__()

        self.defaults = {}
        self.sleep_time = hertz(10.0)
        self.telemetry = None

    def attach(self, vehicle_wrapper):
        super(Navigation, self).attach(vehicle_wrapper)
        intent = self.vehicle.broadcast(Intent(
            action=ACTION_TELEM,
            data={
                'altitude': '/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Alt',
                'latitude': '/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Lat',
                'longitude': '/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Lon',
                'yaw': '/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Yaw'
            })).first()
        self.telemetry = intent.result
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_GOTO]),
            lambda i: self.goto()(**i.data))

    def detach(self):
        self.telemetry = None
        super(Navigation, self).detach()

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.apps.navigation'

    def goto(self, **kwargs):
        return Goto(self, **kwargs)

    def lnav(self, **kwargs):
        return Lnav(self, **kwargs)

    def rotate(self, **kwargs):
        return Rotate(self, **kwargs)

    def vnav(self, **kwargs):
        return Vnav(self, **kwargs)

    @property
    def altitude(self):
        """meters"""
        return self.telemetry['altitude'].value

    @property
    def heading(self):
        """Degrees"""
        return Heading(math.degrees(self.yaw))

    @property
    def latitude(self):
        """Degrees"""
        return self.telemetry['latitude'].value

    @property
    def longitude(self):
        """Degrees"""
        return self.telemetry['longitude'].value

    @property
    def position(self):
        """The spatial state of the vehicle."""
        return Waypoint(
            self.latitude, self.longitude, self.altitude, self.heading)

    @property
    def yaw(self):
        """The vehicle yaw in clockwise radians from north [-Pi, Pi]."""
        return self.telemetry['yaw'].value
