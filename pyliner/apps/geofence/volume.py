"""
Provides base Volume and a few simple concrete classes.

Volumes:
    Volume  Base Volume, all others should subclass.
    CompositeVolume  Volume made of a set of other volumes.
    Box  A box aligned along latitude and longitude
    VerticalCylinder  A cylinder around a Coordinate.
"""

import itertools
import warnings
from abc import ABCMeta, abstractmethod
from collections import Container
from numbers import Real

from pyliner.apps.geographic_app import GeographicApp
from pyliner.position import Position, Coordinate


class Volume(Container):
    """A generic volume class. Extend to create concrete volumes.

    By default, `c = a | b` will return a new CompositeVolume made of a and b.
    """
    __metaclass__ = ABCMeta

    def __init__(self, geographic):
        self.geographic = geographic

    def __or__(self, other):
        return CompositeVolume(self.geographic, {self, other})

    @property
    @abstractmethod
    def bounding_box(self):
        # type: () -> Box
        """The min and max values for latitude, longitude, and altitude."""
        raise NotImplementedError


class CompositeVolume(Volume, set):
    """A volume directly composed of the union of multiple other volumes."""

    def __init__(self, geographic, volumes=None):
        Volume.__init__(self, geographic)
        set.__init__(self)
        if volumes:
            self.update(volumes)

    def __contains__(self, x):
        """Test whether a volume or a point are contained in this volume.

        If x is a Volume, test if the volume is a member of this
        CompositeVolume. If x is anything else, test if x is contained in any of
        the volumes that make up this CompositeVolume.

        Notes:
            Does not test if an arbitrary volume is entirely contained within
            this CompositeVolume.
        """
        return any(x in volume for volume in self) \
            if not isinstance(x, Volume) \
            else super(CompositeVolume, self).__contains__(x)

    def add(self, volume):
        self._check_geographic(volume)
        super(CompositeVolume, self).add(volume)

    @property
    def bounding_box(self):
        boxes = filter(lambda bb: bb is not None,
                       (x.bounding_box for x in self))
        if not boxes:
            return None
        return Box(
            Position(
                min(x.min_latitude for x in boxes),
                min(x.min_longitude for x in boxes),
                min(x.min_altitude for x in boxes)
            ), Position(
                max(x.max_latitude for x in boxes),
                max(x.max_longitude for x in boxes),
                max(x.max_altitude for x in boxes)
            )
        )

    def _check_geographic(self, other):
        if self.geographic and other.geographic \
                and other.geographic is not self.geographic:
            warnings.warn('Volume geographics do not match.')

    def flatten(self):
        """Return a flattened CompositeVolume."""
        todo = list(self)
        flat = set()
        for volume in todo:
            if isinstance(volume, set):  # CompositeVolumes are sets
                todo.extend(volume)
            else:
                flat.add(volume)
        return CompositeVolume(self.geographic, flat)

    def update(self, *s):
        for volume in itertools.chain(s):
            self._check_geographic(volume)
        super(CompositeVolume, self).update(*s)


class Box(Volume):
    """A simple box. Aligned along latitude, longitude, and vertical."""

    def __init__(self, corner_1, corner_2):
        # type: (Position, Position) -> None
        super(Box, self).__init__(None)
        self.min_latitude = min(corner_1.latitude, corner_2.latitude)
        self.max_latitude = max(corner_1.latitude, corner_2.latitude)
        self.min_longitude = min(corner_1.longitude, corner_2.longitude)
        self.max_longitude = max(corner_1.longitude, corner_2.longitude)
        self.min_altitude = min(corner_1.altitude, corner_2.altitude)
        self.max_altitude = max(corner_1.altitude, corner_2.altitude)

    def __contains__(self, other):
        return self.min_latitude <= other.latitude <= self.max_latitude \
               and self.min_longitude <= other.longitude <= self.max_longitude \
               and self.min_altitude <= other.altitude <= self.max_altitude

    def __str__(self):
        return 'Box(latitude=[{}, {}], longitude=[{}, {}], altitude=[{}, {}' \
               '])'.format(self.min_latitude, self.max_latitude,
                           self.min_longitude, self.max_longitude,
                           self.min_altitude, self.max_altitude)

    @property
    def bounding_box(self):
        return self


class VerticalCylinder(Volume):
    """A vertical right circular cylinder.

    Defined in a geographic system by a 2d coordinate, bounding altitudes and a
    radius in meters.
    """

    def __init__(self, geographic, center, radius, low, high):
        # type: (GeographicApp, Coordinate, Real, Real, Real) -> None
        super(VerticalCylinder, self).__init__(geographic)
        self.center = center
        self.high = high
        self.low = low
        self.radius = radius

    def __contains__(self, other):
        return self.low <= other.altitude <= self.high and \
               self.geographic.distance(self.center, other) <= self.radius

    def __repr__(self):
        return 'VerticalCylinder({}, {}, {}, {}, {})'.format(
            self.geographic, self.center, self.radius, self.low, self.high)

    @property
    def bounding_box(self):
        # Check for poles
        distance = self.geographic.distance
        pbd = self.geographic.pbd

        # If the cylinder intersects a pole the latitude caps and the longitude
        # is the full range.
        min_lat = max_lat = min_lon = max_lon = None
        if distance(self.center, Coordinate(90, 0)) <= self.radius:
            max_lat = 90
        if distance(self.center, Coordinate(-90, 0)) <= self.radius:
            min_lat = -90
        if max_lat or min_lat:
            min_lon = 0
            max_lon = 360

        return Box(Position(
            min_lat or pbd(self.center, 180, self.radius).latitude,
            min_lon or pbd(self.center, 270, self.radius).longitude,
            self.low
        ), Position(
            max_lat or pbd(self.center, 0, self.radius).latitude,
            max_lon or pbd(self.center, 90, self.radius).longitude,
            self.high
        ))


class LayerCake(CompositeVolume):
    """Define a layer-cake volume.

    Airspace around major airports is often described as an "upside-down layered
    cake", with larger rings at higher altitudes. Each ring is additive to the
    whole volume, and rings may intersect in altitude.
    """

    def __init__(self, geographic, center, rings=None):
        super(LayerCake, self).__init__(geographic, rings)
        self.center = center

    def add_ring(self, radius, low, high):
        self.add(
            VerticalCylinder(self.geographic, self.center, low, high, radius))
