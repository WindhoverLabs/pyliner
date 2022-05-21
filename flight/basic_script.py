from pathlib import Path
from time import sleep

from xtce.xtce_msg_parser import XTCEParser

from pyliner.command import Arm, Disarm, Takeoff, PosCtl
from pyliner.telemetry import ManualSetpoint
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

tlm_cmd_manager = Communication(read_json('airliner.json'),
                                ParseMode.XTCE,
                                parser,
                                to_port=5230)

def Throttle(comm: Communication):
    comm.send_command(ManualSetpoint(Z=0.2, X=0.75))

def Down(comm: Communication):
    comm.send_command(ManualSetpoint(X=-1))

tlm_val_alt = None
tlm_val_armed = None

# class MyVal():
#     def __init__(self):
#         self.val = None
#
#
# def update_tlm_val(val):
#     # FIXME:The global is temporary
#     global tlm_val_alt
#     tlm_val_alt = val
#     print(f"val-->{val.value}")
#
#
# def update_tlm_armed(val):
#     # FIXME:The global is temporary
#     global tlm_val_armed
#     tlm_val_armed = val
#     print(f"val-->{val.value}")
#
# tlm_cmd_manager.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_GLOBAL_POSITION_MID.Alt', callback=update_tlm_val)
#
# while True:
#     sleep(1)
#     if tlm_val_alt is not None:
#         break
#
# arm = Arm()
# tlm_cmd_manager.send_command(arm)
#
# tlm_cmd_manager.subscribe('/cfs/cpd/apps/vm/VM_HK_TLM_MID.ArmingState', callback=update_tlm_armed)
#
# while True:
#     sleep(1)
#     if tlm_val_armed is not None:
#         if tlm_val_armed.value == 2:
#             break
#
# takeoff = Takeoff()
# tlm_cmd_manager.send_command(takeoff)
#
# while True:
#     sleep(1)
#     if tlm_val_alt is not None:
#         if tlm_val_alt.value > 495:
#             break

mode = PosCtl()

tlm_cmd_manager.send_command(mode)

# Up
# Up()
for i in range(512):
    Throttle(tlm_cmd_manager)
#
#
# while True:
#     sleep(1)
#     if tlm_val_alt is not None:
#         if tlm_val_alt.value > 540:
#             break

print('Done.')
