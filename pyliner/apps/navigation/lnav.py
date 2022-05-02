import re
import time
from datetime import datetime
from numbers import Real

from pyliner.action import ACTION_CALC_DISTANCE, ACTION_AXIS_SET
from pyliner.conversions import seconds
from pyliner.intent import Intent
from pyliner.apps.navigation.navigation_factory import NavigationFactory, NotSet
from pyliner.apps.navigation.command_timeout import CommandTimeout


class Lnav(NavigationFactory):
    """Perform a lateral movement."""

    def __call__(self, distance, axis=NotSet, method=NotSet, tolerance=NotSet,
                 timeout=seconds(30)):
        """Block until the distance traveled is within the tolerance of the
        target distance.

        Args:
            distance (Real): Distance in meters to move.
            axis (str): Must be one of the lateral movement axes (x, y, or z),
                and may be prefixed by an optional "-" for negative movement.
            method (Callable[[Real, Real], Real]): Callable takes two arguments,
                the distance traveled and the target distance, and returns an
                axis control value within [-1, 1].
            tolerance (Real): The allowable error in movement before the method
                sets the control to 0 and returns.
        """
        # NotSet resolution
        axis = self.resolve(axis, 'axis')
        method = self.resolve(method, 'method')
        tolerance = self.resolve(tolerance, 'tolerance')
        timeout = self.resolve(timeout, 'timeout')

        # Sanity checks
        if not method or not callable(method):
            raise ValueError('Must have a callable navigation method.')
        if not isinstance(tolerance, Real) or tolerance <= 0:
            raise ValueError('Tolerance must be set to a positive real number.')

        # Match axis value
        valid = '(-)?([xyz])'
        axis_match = re.match(valid, axis)
        if not axis_match:
            raise ValueError('Axis must match "{}".'.format(valid))
        neg, axis = axis_match.groups()

        # Timeout
        # TODO Use vehicle time not local
        timeout = datetime.max if timeout is None else timeout + datetime.now()

        original = self.nav.position
        while datetime.now() < timeout:
            delta = self.broadcast(Intent(
                action=ACTION_CALC_DISTANCE, data=(original, self.nav.position)
            )).first().result
            if (distance - delta) < tolerance:
                self.info('lnav expected %s actual %s (%s < %s m)',
                          distance, delta, distance - delta, tolerance)
                self.broadcast(Intent(action=ACTION_AXIS_SET, data={axis: 0.0}))
                return self
            control = method(delta, distance)
            control = -control if neg else control
            self.debug('lnav toward %.3f actual %.3f (%.3f < %.3f m) %.3f',
                       distance, delta, distance - delta, tolerance, control)
            self.broadcast(Intent(action=ACTION_AXIS_SET, data={axis: control}))
            time.sleep(self.nav.sleep_time)
        raise CommandTimeout('lnav exceeded timeout')

    def backward(self, distance, **kwargs):
        """Move backward by a distance. See __call__ for full documentation."""
        return self(distance, '-x', **kwargs)

    def forward(self, distance, **kwargs):
        """Move forward by a distance. See __call__ for full documentation."""
        return self(distance, 'x', **kwargs)

    def left(self, distance, **kwargs):
        """Move left by a distance. See __call__ for full documentation."""
        return self(distance, '-y', **kwargs)

    def right(self, distance, **kwargs):
        """Move right by a distance. See __call__ for full documentation."""
        return self(distance, 'y', **kwargs)
