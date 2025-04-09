# from pyliner.util import enable_logging
# from pyliner.apps.xtce_communication import XTCE_Communication
import time

from pyliner.apps.xtce_msg_parser import XTCEParser
# from pyliner.vehicle import Vehicle
# from pyliner.yamcs_vehicle import YamcsVehicle
# from pyliner.apps.yamcs_communication import YamcsCommunication
# from pyliner.scripting_wrapper import ScriptingWrapper
# from yamcs.client import YamcsClient
# from pyliner.util import read_json
from pathlib import Path
import xtce.xtce
# enable_logging()
# from pyliner.apps.xtce_communication import XTCE_Communication
from pyliner.apps.communication import Communication, ParseMode
from pyliner.util import read_json
from pyliner.vehicle import Vehicle

ppd = Path('./mdb/ppd.xml').resolve()
cpd = Path('./mdb/cpd.xml').resolve()
simlink = Path('./mdb/simlink.xml').resolve()
ccscds = Path('./mdb/cfs-ccsds.xml').resolve()

# parser = XTCEParser([str(ppd), str(cpd), str(simlink)], str(ccscds))
parser = XTCEParser([str(cpd)], str(ccscds))

connection = Communication(read_json('airliner.json'),
                           ParseMode.XTCE,
                           parser,
                           to_port=5111)

# while True:
#     time.sleep(5)
# vehicle = Vehicle(vehicle_id='example', communication=connection)
# from pyliner.apps.xtce_msg_parser import XTCEParser
# from pyliner.scripting_wrapper import ScriptingWrapper
#
# # Update with PV
# with ScriptingWrapper(vehicle) as v:
#     v.await_change('/Airliner/CNTL/VehicleGlobalPosition/Alt',
#                    'Waiting for telemetry downlink')
#     v.ctrl.atp('Begin Script?')
#     v.ctrl.arm()
#     v.ctrl.takeoff()
#     v.ctrl.atp('Return?')
#     v.ctrl.rtl()
