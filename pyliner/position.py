"""
Container module for the basic 2, 3, and 4-dimensional position classes.

Airspace is inherently 3-dimensional, favor Position over Coordinate when an
altitude is available. When a direction is available, use a Waypoint.
"""
import math


class Coordinate(object):
    """A 2-dimensional point on the Earth's surface.

    Represented by latitude and longitude in degrees.
    """
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__,
                                   self.latitude, self.longitude)

    @property
    def elevation(self):
        """Polar angle in radians from the equator going north."""
        return math.radians(self.latitude)

    @property
    def phi(self):
        """Polar angle in radians from the north pole going south."""
        return math.pi/2 - self.elevation

    @property
    def theta(self):
        """Azimuth in radians from the prime meridian. Positive = East"""
        return math.radians(self.longitude)


class Position(Coordinate):
    """A 3-dimensional point above the Earth.

    Represented by latitude and longitude in degrees, and altitude in meters.
    """
    def __init__(self, latitude, longitude, altitude):
        super(Position, self).__init__(latitude, longitude)
        self.altitude = altitude

    def __repr__(self):
        return('{}({}, {}, {})'.format(self.__class__.__name__, self.latitude,
                                       self.longitude, self.altitude))

    @classmethod
    def from_coordinate(cls, coordinate, altitude):
        return Position(coordinate.latitude, coordinate.longitude, altitude)


class Waypoint(Position):
    """A 4-dimensional coordinate, Position + heading

    May be used as part of the arguments to a triplet command.
    """

    def __init__(self, latitude, longitude, altitude, heading):
        super(Waypoint, self).__init__(latitude, longitude, altitude)
        self.heading = heading

    def __repr__(self):
        return '{}({}, {}, {}, {})'.format(
            self.__class__.__name__, self.latitude, self.longitude,
            self.altitude, self.heading)

    @property
    def yaw(self):
        if self.heading is None:
            return None
        return math.radians(
            self.heading if self.heading < 180 else self.heading - 360)
