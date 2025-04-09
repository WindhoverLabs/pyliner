import time
from datetime import timedelta, datetime
from numbers import Real

from pyliner.action import ACTION_AXIS_SET
from pyliner.intent import Intent
from pyliner.heading import Heading, Direction
from pyliner.apps.navigation.navigation_factory import NavigationFactory, NotSet
from pyliner.apps.navigation.command_timeout import CommandTimeout


class Rotate(NavigationFactory):
    """Rotate the vehicle."""

    def __call__(self, by=None, to=None, method=NotSet, tolerance=NotSet,
                 direction=Direction.NEAREST, timeout=NotSet, underflow=NotSet):
        """Block until the vehicle is within tolerance of the target heading.

        Args:
            by (Real): Change heading by this amount. May be negative. Cannot
                be set at the same time as `to`.
            to (Real): Rotate to this heading. Cannot be set at the same
                time as `by`.
            method (Callable[[Real, Real], Real]): Take two arguments, the
                current heading and the target heading, and return an
                axis control value within [-1, 1].
            tolerance (Real): The allowable error in altitude before the method
                sets the z-axis to 0 and returns.
            direction (Direction): The direction to rotate.
            timeout (Optional[timedelta]): If not None, the amount of time the
                method has to complete an operation before raising
                CommandTimeout. If None the timeout is datetime.max.
            underflow (Real): If the vehicle over-rotates and is outside of
                tolerance this is how far the vehicle is allowed to correct
                the error before resetting to follow the set direction.
        """
        # NotSet resolution
        method = self.resolve(method, 'method')
        tolerance = self.resolve(tolerance, 'tolerance')
        direction = self.resolve(direction, 'direction')
        timeout = self.resolve(timeout, 'timeout')
        underflow = self.resolve(underflow, 'underflow')

        # Sanity checks
        if not by and not to:
            raise ValueError('Must set one of by or to.')
        elif by and to:
            raise ValueError('Must not set both by and to.')
        if not method or not callable(method):
            raise ValueError('Must have a callable navigation method.')
        if not isinstance(tolerance, Real) or tolerance < 0.0:
            raise ValueError('Tolerance must be a non-negative real number.')
        if direction not in Direction:
            raise TypeError('Direction must be a valid Direction.')
        if timeout is not None and not isinstance(timeout, timedelta):
            raise ValueError('Timeout must be None or a timedelta.')
        if not isinstance(underflow, Real) or underflow < 0.0:
            raise ValueError('Underflow must be a non-negative real number.')

        # Timeout
        # TODO Use vehicle time not local
        timeout = datetime.max if timeout is None else datetime.now() + timeout

        original = self.nav.heading
        target = original + by if by else Heading(to)
        tol_range = target.range(tolerance)

        while datetime.now() < timeout:
            current = self.nav.heading
            if current in tol_range:
                self.info('rotate expected %s actual %s (in %s)',
                          target, current, tol_range)
                self.broadcast(Intent(action=ACTION_AXIS_SET, data={'r': 0.0}))
                return self
            distance = Heading.distance(current, target, direction, underflow)
            control = method(0.0, distance)
            self.debug('rotate toward %.3f current %.3f (%.3f < %.3f) %.3f',
                       target, current, abs(distance), tolerance, control)
            self.broadcast(Intent(action=ACTION_AXIS_SET, data={'r': control}))
            time.sleep(self.nav.sleep_time)
        raise CommandTimeout('rotate exceeded timeout')

    def clockwise(self, degrees, **kwargs):
        """Rotate clockwise by a number of degrees."""
        self(by=degrees, direction=Direction.CLOCKWISE, **kwargs)

    def counterclockwise(self, degrees, **kwargs):
        """Rotate counterclockwise by a number of degrees."""
        self(by=-degrees, direction=Direction.COUNTERCLOCKWISE, **kwargs)
