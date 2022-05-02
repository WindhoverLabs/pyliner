import time
from datetime import datetime
from numbers import Real

from pyliner.action import ACTION_AXIS_SET
from pyliner.conversions import seconds
from pyliner.intent import Intent
from pyliner.apps.navigation.navigation_factory import NavigationFactory, NotSet
from pyliner.apps.navigation.command_timeout import CommandTimeout


class Vnav(NavigationFactory):
    """Perform a vertical navigation."""

    def __call__(self, by=None, to=None, method=NotSet, tolerance=NotSet,
                 timeout=seconds(30)):
        """Block until the vehicle is within tolerance of the target altitude.

        Args:
            by (Real): Change altitude by this amount. May be negative. Cannot
                be set at the same time as `to`.
            to (Real): Ascend or descend to this altitude. Cannot be set at the
                same time as `by`.
            method (Callable[[Real, Real], Real]): Take two arguments, the
                current altitude and the target altitude, and return a z-axis
                control value [-1, 1].
            tolerance (Real): The allowable error in altitude before the method
                sets the z-axis to 0 and returns.
            timeout (Optional[timedelta]): If not None, the amount of time the
                method has to complete an operation before raising
                CommandTimeout. If None the timeout is datetime.max.
        """
        # NotSet resolution
        method = self.resolve(method, 'method')
        timeout = self.resolve(timeout, 'timeout')
        tolerance = self.resolve(tolerance, 'tolerance')

        # Sanity checks
        if not by and not to:
            raise ValueError('Must set either by or to.')
        elif by and to:
            raise ValueError('Must not set both by and to.')
        if not method or not callable(method):
            raise ValueError('Must have a callable navigation method.')
        if not isinstance(tolerance, Real) or tolerance <= 0:
            raise ValueError('Tolerance must be set to a positive real number.')

        # Timeout
        # TODO Use vehicle time not local
        timeout = datetime.max if timeout is None else timeout + datetime.now()

        target_altitude = (self.nav.altitude + by) if by else to

        while datetime.now() < timeout:
            difference = abs(target_altitude - self.nav.altitude)
            if difference <= tolerance:
                self.info('vnav expected %s actual %s (%s < %s m)',
                          target_altitude, self.nav.altitude,
                          difference, tolerance)
                self.broadcast(Intent(action=ACTION_AXIS_SET, data={'z': 0.0}))
                return self
            control = method(self.nav.altitude, target_altitude)
            self.debug('vnav toward %.3f actual %.3f (%.3f < %.3f m) %.3f',
                       target_altitude, self.nav.altitude,
                       difference, tolerance, control)
            self.broadcast(Intent(action=ACTION_AXIS_SET, data={'z': control}))
            time.sleep(self.nav.sleep_time)
        raise CommandTimeout('vnav exceeded timeout')

    def down(self, distance, **kwargs):
        """Move down by a distance. See vnav for full documentation."""
        return self(by=-distance, **kwargs)

    def to(self, altitude, **kwargs):
        """Climb or descend to altitude. See vnav for full documentation."""
        return self(to=altitude, **kwargs)

    def up(self, distance, **kwargs):
        """Move up by a distance. See vnav for full documentation."""
        return self(by=distance, **kwargs)
