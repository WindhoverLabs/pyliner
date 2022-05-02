"""
Pyliner

Pyliner is a Python-based drone control scripting interface from Windhover Labs.
It uses a custom flight software called Airliner, developed by Windhover Labs,
to send commands and receive telemetry from the actual vehicle. Pyliner presents
Airliner functionality through a combination of Pyliner Apps which the user may
choose to enable or disable at will.

Pyliner by itself is a flexible, open-ended Intent-passing framework that gives
users the flexibility to choose how their platform behaves, and what features
it presents. When combined with the Windhover Labs Pyliner Apps that are built-
in to the Vehicle class or available by themselves from the pyliner.apps
package, Pyliner becomes a feature rich drone flight software controller.

Assuming that Airliner is running and listening on the appropriate ports,
controlling a vehicle from the scripting interface is super easy!

    >>> from pyliner import Vehicle, enable_logging
    >>> from pyliner.apps.communication import Communication
    >>> from pyliner.scripting_wrapper import ScriptingWrapper
    >>> from pyliner.util import read_json
    >>> enable_logging()
    >>> vehicle = Vehicle(
    ...     vehicle_id='example',
    ...     communication=Communication(
    ...         airliner_map=read_json('airliner.json'),
    ...         address='localhost', ci_port=5009, to_port=5012))
    >>> with ScriptingWrapper(vehicle) as v:
    ...     v.await_change('/Airliner/CNTL/VehicleGlobalPosition/Alt',
    ...                    'Waiting for telemetry downlink')
    ...     v.ctrl.atp('Begin Script?')
    ...     v.ctrl.arm()
    ...     v.ctrl.takeoff()
    ...     v.ctrl.atp('Return?')
    ...     v.ctrl.rtl()


"""

# from pyliner.util import enable_logging
# from pyliner.vehicle import Vehicle
#
# __version__ = '1.0'
# __all__ = ['Vehicle', 'enable_logging']
