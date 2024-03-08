from pyliner.apps.communication import Communication, ParseMode
from pyliner.apps.controller import FlightMode
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json
from pyliner.command import Command
from pyliner.vehicle import Vehicle

import time
from pathlib import Path

from xtce.xtce_msg_parser import XTCEParser

ppd = Path('../mdb/ppd.xml').resolve()
cpd = Path('../mdb/cpd.xml').resolve()
simlink = Path('../mdb/simlink.xml').resolve()
ccscds = Path('../mdb/cfs-ccsds.xml').resolve()
registry = '../mdb/registry.yaml'

print(f"ppd-->{ppd}")

parser = XTCEParser([str(ppd), str(cpd), str(simlink)], str(ccscds), registry)

comms = Communication(ParseMode.XTCE,
                      parser,
                      to_port=6011)
rky = Vehicle(
    vehicle_id='rocky',
    communication=comms
)

with ScriptingWrapper(rky) as rocky:
    while rocky.nav.altitude is None:
        time.sleep(1)
        print ("Waiting for telemetry downlink...")

    rocky.ctrl.atp('Arm')
    # rocky.ctrl.arm()
    rky.logger.setLevel(40)

    # rky.ctrl.atp('Arm')

    # comms.send_message(Command("/cfs/cpd/apps/vm/Arm"))


    def watch_gpio_status(tlm):
        print("Watch GPIO:", tlm) 

    comms.subscribe('/cfs/cpd/apps/gpio/GPIO_STATUS_TLM_MID.Status', callback=watch_gpio_status)

# comms.send_message(Command("/cfs/cpd/apps/vm/Arm"))