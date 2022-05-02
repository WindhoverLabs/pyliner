"""
The control module provides simple control mixin functions.

Decorators:
    even  Decorate if function should be even.
    odd  Decorate if function should be odd.

Controllers:
    adder  Sum the result of all given functions.
    constant  Give a single value no matter the input.
    integral  Standard integral controller.
    limiter  Limit the output of a function.
    pi  Proportional-Integral controller.
    proportional  Standard proportional controller.
    scale  Scale an input by a factor.
"""


# Function Decorators
def even(f):
    """Decorate if f(x)==f(-x) but f is only defined for x>=0."""
    def wrapper(x):
        return f(x) if x >= 0 else f(-x)
    return wrapper


def odd(f):
    """Decorate if f(x)==-f(-x) but f is only defined for x>=0."""
    def wrapper(x):
        return f(x) if x >= 0 else -f(-x)
    return wrapper


# Zero (or any) input
def adder(*processes):
    """Sum the output of processes when called with the same arguments."""
    return lambda *args, **kwargs: sum(p(*args, **kwargs) for p in processes)


def constant(value):
    """Return a constant value regardless of input."""
    return lambda *args, **kwargs: value


# One input
def limiter(min_val=float('-inf'), max_val=float('inf')):
    """Bound the output of a function to a range."""
    return lambda val: max(min(val, max_val), min_val)


def scale(factor):
    """Linearly scale a single input."""
    return lambda val: val * factor


# Two inputs: (current, target)
def integral(gain, t_sample, t_integral):
    """Generate an integral controller with a gain, sample and integral times.

    Controller returns a control value when called with two args, the current
    value and the target setpoint.
    """
    factor = gain * t_sample / t_integral

    def _integral():
        result = 0.0
        while True:
            current, target = yield result
            result += factor * (target - current)
    gen = _integral()
    next(gen)
    return lambda current, target: gen.send((current, target))


def pi(p_gain, i_gain, t_sample, t_integral, p_bias=0.0):
    """Generate a proportional-integral controller.

    Controller returns a control value when called with two args, the current
    value and the target setpoint. In this setup, the input differential is
    fed to the proportional controller, which feeds the integral controller.
    The output is a sum of the proportional and integral controllers.

    Args:
        p_gain (Real): Proportional gain between the current and target value.
        p_bias (Real): Constant bias to proportional output.
        i_gain (Real): Integral gain between the current and target value.
        t_sample (Real): Sampling time, smaller = slower response
        t_integral (Real): Integral time, smaller = faster response
    """
    def _pi():
        p = proportional(p_gain, p_bias)
        i = integral(i_gain, t_sample, t_integral)
        p_out = i_out = 0.0
        while True:
            current, target = yield p_out + i_out
            p_out = p(current, target)
            i_out = i(0.0, p_out)
    gen = _pi()
    next(gen)
    return lambda current, target: gen.send((current, target))


def proportional(gain, bias=0.0):
    """Generate a proportional controller with a gain and a bias.

    Controller returns a control value when called with two args, the current
    value and the target setpoint.
    """
    return lambda current, target: gain * (target - current) + bias
