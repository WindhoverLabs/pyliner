"""
Provides shorthands for a handful of useful conversions to SI units.
"""

from datetime import timedelta


# Length
def feet(ft):
    """Convert from given feet to meters."""
    return ft * 0.3048


def meters(m):
    """Return m, here for completeness."""
    return m


# Time
def hours(hrs):
    """Timedelta with hrs as hours."""
    return timedelta(hours=hrs)


def minutes(min):
    """Timedelta with min as minutes."""
    return timedelta(minutes=min)


def seconds(sec):
    """Timedelta with sec as seconds."""
    return timedelta(seconds=sec)


def hertz(hz):
    """Convert from hz to period/rate. Simply inverse of hz."""
    return 1.0 / float(hz)
