"""
Fly a heading and to a waypoint.

Requirements Fulfilled:
    PYLINER001
    PYLINER002
    PYLINER003
    PYLINER004
    PYLINER005
    PYLINER006
    PYLINER010
    PYLINER011
    PYLINER012
    PYLINER013
    PYLINER014
    PYLINER016
"""
from time import sleep

from pyliner import Vehicle
from pyliner.apps.communication import Communication
from pyliner.apps.navigation.control import limiter, proportional
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json


def range_limit(current, target):
    return limiter(-0.2, 0.2)(proportional(0.1 / 50.0)(current, target))


rky = Vehicle(
    vehicle_id='rocky',
    communication=Communication(
        airliner_map=read_json("airliner.json"),
        address="192.168.1.2",
        ci_port=5009,
        to_port=5012)
)

with ScriptingWrapper(rky) as rocky:
    while rocky.nav.altitude == "NULL":
        sleep(1)
        print "Waiting for telemetry downlink..."

    rocky.ctrl.atp('Arm')
    rocky.ctrl.arm()
    rocky.ctrl.atp('Takeoff')
    rocky.ctrl.takeoff()
    # rocky.ctrl.flight_mode(FlightMode.PosCtl)

    rocky.ctrl.atp('Goto')
    goto = rocky.nav.goto(tolerance=1)

    home = rocky.nav.position
    new = rocky.geographic.pbd(home, 90, 20)
    new.altitude = rocky.nav.altitude + 10

    goto(new)
    goto(rocky.geographic.pbd(new, 270, 40))

    rocky.ctrl.atp('Return')
    rocky.ctrl.rtl()
