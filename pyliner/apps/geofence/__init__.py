"""
The geofence package provides sample volumes and a Geofence application.

Apps:
    Geofence  Maintains a volumetric space of operations for the vehicle and
        commands an RTL in the event of a violation.

Volumes:
    Layer  Primarily used by the geofence, layers of volumes.
    LayerCake  A set of cylinders around a single Coordinate.

Modules:
    volume  Collection of base and simple Volume classes.

Notes:
    This package is not set up to handle boxes crossing the anti-meridian.
    Currently you will get a large bounding box around the entire planet.
    TODO: How to identify bounding boxes that cross? Take smaller box?
        Possible solution: If max_longitude < min_longitude, then the box
            crosses the anti-meridian.
"""
import warnings
from functools import partial

from enum import Enum
from sortedcontainers import SortedDict

from pyliner.action import ACTION_RTL, ACTION_TELEM
from pyliner.app import App
from pyliner.apps.geofence.volume import Volume, CompositeVolume
from pyliner.intent import Intent
from pyliner.position import Position
from pyliner.util import indent, RealTimeThread

FENCE_SLEEP = 1.0


class FenceGenerator(object):
    """Curry a bunch of Volumes with the same Geographic."""

    def __init__(self, geographic, *classes):
        self.geographic = geographic
        self.curry(*classes)

    def curry(self, *classes):
        for c in classes:
            if not issubclass(c, Volume):
                warnings.warn('Curry was expecting a subclass of Volume and '
                              'received a {}'.format(c))
            setattr(self, c.__name__, partial(c, geographic=self.geographic))


class LayerKind(Enum):
    """Whether a layer is additive or subtractive."""
    ADDITIVE = True
    SUBTRACTIVE = False


class Layer(CompositeVolume):
    """A layer of volumes in a geofence."""

    def __init__(self, name, kind, geographic=None):
        super(Layer, self).__init__(geographic)
        self.name = name
        self.kind = kind

    def __contains__(self, other):
        return other in self.bounding_box \
               and super(Layer, self).__contains__(other)

    def __str__(self):
        return '{}\n'.format(self.name) + '\n'.join(
            '{}'.format(volume) for volume in indent(4, self))


class Geofence(App):
    """A Geofence defines the space a vehicle is allowed to operate within.

    A geofence is constructed by layering additive and subtractive geometry
    to construct a 3-dimensional space of operations that a drone is allowed to
    fly in.

    Within a layer, a point is determined to be inside as if all the volumes in
    that layer were taken as a union.
    """

    # TODO Use a small memory database (like TinyDB) to handle layer mapping.
    #   Added benefit of allowing both name and order mapping to layer at once.

    def __init__(self):
        super(Geofence, self).__init__()
        self.enabled = False
        self.layers = SortedDict()
        """:type: dict[Any, _Layer]"""

        self._check_thread = None
        self._fence_violation = False
        self._telemetry = None

    def __contains__(self, other):
        """True if the given other is contained within the Geofence."""
        contained = False
        for layer in self.layers.values():
            if other in layer:
                contained = layer.kind is LayerKind.ADDITIVE
        return contained

    def __str__(self):
        return 'Geofence{\n' + '\n'.join(' {}{}: {}'.format(
            '+' if layer.kind is LayerKind.ADDITIVE else '-',
            order, layer) for order, layer in self.layers.items()) + '\n}'

    def attach(self, vehicle):
        super(Geofence, self).attach(vehicle)
        self._telemetry = self.vehicle.broadcast(Intent(
            action=ACTION_TELEM,
            data={
                'latitude': '/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Lat',
                'longitude': '/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Lon',
                'altitude': '/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Alt'})
        ).first().result

        self._check_thread = RealTimeThread(
            self._check_fence, every=FENCE_SLEEP,
            logger=self.vehicle.logger, name='FenceCheck',
            exception=lambda e: self.vehicle.exception('Geofence Exception'))
        self._check_thread.start()

    def detach(self):
        self._check_thread.stop()
        self._telemetry = None
        super(Geofence, self).detach()

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.apps.geofence'

    def add_layer(self, layer_position, layer_name, layer_kind):
        if layer_position in self.layers:
            raise KeyError('This layer already exists.')
        if not isinstance(layer_kind, LayerKind):
            raise TypeError('layer_kind must be of type LayerKind.')
        layer = Layer(name=layer_name, kind=layer_kind)
        self.layers[layer_position] = layer
        return layer

    def _check_fence(self):
        old = self._fence_violation
        self._fence_violation = self._fence_violation or \
                                (self.enabled and self.position not in self)
        if not old and self._fence_violation:
            self.vehicle.error('Encountered Fence Violation at %s',
                               self.position)
            self.vehicle.broadcast(Intent(action=ACTION_RTL))
            print('Encountered fence violation. Press Ctrl-C exit.')

    def layer_by_name(self, name):
        for layer in self.layers.values():
            if layer.name == name:
                return layer

    @property
    def position(self):
        return Position(
            self._telemetry['latitude'].value,
            self._telemetry['longitude'].value,
            self._telemetry['altitude'].value)

    def remove_layer(self, position):
        del self.layers[position]
