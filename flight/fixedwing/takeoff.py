import time
from pathlib import Path

from xtce.xtce_msg_parser import XTCEParser

from pyliner.apps.communication import Communication, ParseMode
from pyliner.apps.controller import FlightMode
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json
from pyliner.command import Command
from pyliner.vehicle import Vehicle

ppd = Path('../../mdb/ppd.xml').resolve()
cpd = Path('../../mdb/cpd.xml').resolve()
simlink = Path('../../mdb/simlink.xml').resolve()
ccscds = Path('../../mdb/cfs-ccsds.xml').resolve()
registry = '../../mdb/registry.yaml'

print(f"ppd-->{ppd}")

parser = XTCEParser([str(ppd), str(cpd), str(simlink)], str(ccscds), registry)

comms = Communication(ParseMode.XTCE,
                      parser,
                      to_port=5111)
rky = Vehicle(
    vehicle_id='rocky',
    communication=comms
)

rky.logger.setLevel(40)

comms.send_message(Command("/cfs/cpd/apps/vm/AutoTakeOff"))