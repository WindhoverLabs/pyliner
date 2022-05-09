"""
Flies forward, backward, left, and right.

Requirements Fulfilled:
    PYLINER001
    PYLINER002
    PYLINER003
    PYLINER010
    PYLINER011
    PYLINER012
    PYLINER013
    PYLINER014
    PYLINER016
"""
from pathlib import Path
from time import sleep

from xtce.xtce_msg_parser import XTCEParser

from pyliner.vehicle import Vehicle
from pyliner.apps.communication import Communication, ParseMode
from pyliner.apps.controller import FlightMode
from pyliner.apps.navigation.control import proportional
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json

ppd = Path('../mdb/ppd.xml').resolve()
cpd = Path('../mdb/cpd.xml').resolve()
simlink = Path('../mdb/simlink.xml').resolve()
ccscds = Path('../mdb/cfs-ccsds.xml').resolve()
registry = '../mdb/registry.yaml'

parser = XTCEParser([str(ppd), str(cpd), str(simlink)], str(ccscds), registry)

rky = Vehicle(
    vehicle_id='rocky',
    communication=Communication(read_json('airliner.json'),
                                ParseMode.XTCE,
                                parser,
                                to_port=5111)
)

with ScriptingWrapper(rky) as rocky:
    while rocky.nav.altitude is None:
        sleep(1)
        print("Waiting for telemetry downlink...")

    rocky.ctrl.atp('Arm')
    rocky.ctrl.arm()
#     rocky.ctrl.atp('Takeoff')
#     rocky.ctrl.takeoff()
#     rocky.ctrl.flight_mode(FlightMode.PosCtl)
#
#     rocky.ctrl.atp('Move Up')
#     rocky.nav.vnav(method=proportional(0.25), tolerance=0.5) \
#         .up(10)
#
#     rocky.ctrl.atp('First')
#     lnav = rocky.nav.lnav(method=proportional(0.20), tolerance=0.5)
#     lnav.forward(15)
#     lnav.right(15)
#     lnav.backward(15)
#     lnav.left(15)
#
#     rocky.ctrl.atp('Second')
#     lnav.forward(15)
#     lnav.left(15)
#     lnav.backward(15)
#     lnav.right(15)
#
#     rocky.ctrl.atp('Third')
#     rocky.nav.lnav(method=proportional(0.20), tolerance=0.5) \
#         .backward(15) \
#         .left(15) \
#         .forward(15) \
#         .right(15)
#
#     rocky.ctrl.atp('Fourth')
#     rocky.nav.lnav(method=proportional(0.20), tolerance=0.5) \
#         .backward(15).right(15).forward(15).left(15)
#
#     rocky.ctrl.atp('Return')
#     rocky.ctrl.rtl()
