from pathlib import Path
from time import sleep

from xtce.xtce_msg_parser import XTCEParser

from pyliner.apps.communication import Communication, ParseMode
from pyliner.apps.geofence import Geofence
from pyliner.apps.geofence.volume import VerticalCylinder
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
    communication=Communication(read_json('airliner.json'),
                                ParseMode.XTCE,
                                parser,
                                to_port=5111)
)

with ScriptingWrapper(rky) as rocky:
    while rocky.nav.altitude == "NULL":
        sleep(1)
        print("Waiting for telemetry downlink...")
    
    rocky.ctrl.atp('Arm')
    rocky.ctrl.arm()
    rocky.ctrl.atp('Takeoff')
    rocky.ctrl.takeoff()
    # rocky.ctrl.flight_mode(FlightMode.PosCtl)

    rocky.ctrl.atp('Enable Fence')
    fence = rocky.fence  # type: Geofence
    base = fence.layers[0]
    base.add(VerticalCylinder(
        rocky.geographic, rocky.nav.position,
        low=rocky.nav.altitude - 10, high=rocky.nav.altitude + 100, radius=20))
    fence.enabled = True

    rocky.ctrl.atp('Start mission')
    home = rocky.nav.position
    new = rocky.geographic.pbd(home, 90, 15)
    new.altitude = rocky.nav.altitude + 10

    goto = rocky.nav.goto(tolerance=0.5)
    goto(new)
    new = rocky.geographic.pbd(new, 0, 50)
    goto(new)

    rocky.ctrl.atp('Return (if you got here the fence failed)')
    rocky.ctrl.rtl()
