"""
Fly in a rising spiral.

Requirements Fulfilled:
    PYLINER001
    PYLINER003
    PYLINER004
    PYLINER009
    PYLINER010
    PYLINER011
    PYLINER012
    PYLINER013
    PYLINER014
    PYLINER016
"""
import time
from pathlib import Path

from xtce.xtce_msg_parser import XTCEParser

from pyliner.apps.communication import Communication, ParseMode
from pyliner.apps.controller import FlightMode
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json
from pyliner.vehicle import Vehicle

ppd = Path('../mdb/ppd.xml').resolve()
cpd = Path('../mdb/cpd.xml').resolve()
simlink = Path('../mdb/simlink.xml').resolve()
ccscds = Path('../mdb/cfs-ccsds.xml').resolve()
registry = '../mdb/registry.yaml'

parser = XTCEParser([str(ppd), str(cpd), str(simlink)], str(ccscds), registry)

rky = Vehicle(
    vehicle_id='rocky',
    communication=Communication(
    read_json("airliner.json"),
        ParseMode.XTCE,
        parser)
)
with ScriptingWrapper(rky) as rocky:
    while rocky.nav.altitude is None:
        time.sleep(1)
        print("Waiting for telemetry downlink...")
    
    rocky.ctrl.atp('Arm')
    rocky.ctrl.arm()
    rocky.ctrl.atp('Takeoff')
    rocky.ctrl.takeoff()
    rocky.ctrl.flight_mode(FlightMode.PosCtl)

    start_altitude = rocky.nav.altitude

    rocky.ctrl.atp('Start Spiral')
    rocky.fd.y = 0.75
    rocky.fd.z = 0.5
    rocky.fd.r = -0.2

    while rocky.nav.altitude < start_altitude + 10:
        time.sleep(1)
    rocky.fd.zero()

    rocky.ctrl.atp('Return')
    rocky.ctrl.rtl()
