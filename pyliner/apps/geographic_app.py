"""
The Geographic App module provides a app that can be used to calculate
information about two Coordinates on Earth.

Sensors:
    GeographicApp  Produces relevant world-based calculations.
"""

from copy import copy
from numbers import Real

from geographiclib.geodesic import Geodesic

from pyliner.action import ACTION_CALC_BEARING, ACTION_CALC_DISTANCE, \
    ACTION_CALC_PBD
from pyliner.intent import IntentFilter
from pyliner.position import Coordinate
from pyliner.app import App


class GeographicApp(App):
    """An App that produces relevant world-based calculations.

    This App listens to ACTION_CALC_BEARING, ACTION_CALC_DISTANCE, and
    ACTION_CALC_PBD. For BEARING or DISTANCE, data must be populated with an
    iterable of two Coordinates. For PDB, data must be an iterable representing
    an origin Coordinate, and two real numbers representing a desired bearing
    and a desired distance.
    """

    def __repr__(self):
        return 'GeographicApp()'

    def __str__(self):
        return 'WGS84'

    def attach(self, vehicle_wrapper):
        super(GeographicApp, self).attach(vehicle_wrapper)
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_CALC_BEARING]),
            lambda i: GeographicApp.bearing(*i.data))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_CALC_DISTANCE]),
            lambda i: GeographicApp.distance(*i.data))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_CALC_PBD]),
            lambda i: GeographicApp.pbd(*i.data)
        )

    def detach(self):
        self.vehicle.clear_filter()
        super(GeographicApp, self).detach()

    @staticmethod
    def bearing(a, b):
        # type: (Coordinate, Coordinate) -> Real
        """Calculate the bearing from a to b in degrees."""
        return GeographicApp._inverse(a, b)['azi1'] % 360

    @staticmethod
    def _direct(a, azim, dist):
        # type: (Coordinate, Real, Real) -> dict
        # Possibility of LRU cache here.
        # noinspection PyUnresolvedReferences
        return Geodesic.WGS84.Direct(a.latitude, a.longitude, azim, dist)

    @staticmethod
    def distance(a, b):
        # type: (Coordinate, Coordinate) -> Real
        """Calculate the distance between two points on the globe.

        Args:
            a (Coordinate): Point A
            b (Coordinate): Point B

        Returns:
            float: Distance in meters.
        """
        return GeographicApp._inverse(a, b)['s12']

    @staticmethod
    def _inverse(a, b):
        # type: (Coordinate, Coordinate) -> dict
        # Possibility of LRU cache here.
        # noinspection PyUnresolvedReferences
        return Geodesic.WGS84.Inverse(a.latitude, a.longitude,
                                      b.latitude, b.longitude)

    @staticmethod
    def pbd(a, bearing, distance):
        # type: (Coordinate, Real, Real) -> Coordinate
        """Calculate a Place-Bearing-Distance (PBD) coordinate.

        Returns:
            (Coordinate): A copy of `a` with updated latitude and longitude.

        Args:
            a (Coordinate): Point A
            bearing (Real): Direction in degrees [0, 360)
            distance (Real): Distance in meters
        """
        direct = GeographicApp._direct(a, bearing, distance)
        b = copy(a)
        b.latitude = direct['lat2']
        b.longitude = direct['lon2']
        return b

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.apps.geographic'
