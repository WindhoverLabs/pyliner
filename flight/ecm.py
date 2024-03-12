"""
ECM - Engine Control Management
"""

from flight.nav import Nav
from flight.px4_lib import PX4_HomePositionMsg_t
from pyliner.apps.communication import Communication, ParseMode
from pyliner.apps.controller import FlightMode
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json
from pyliner.command import Command
from pyliner.vehicle import Vehicle

import time
from pathlib import Path

from xtce.xtce_msg_parser import XTCEParser




# GPIO definitions.
G_PIN_GPIO_6  =  (0b0000000010000000000)
G_PIN_GPIO_7  =  (0b0000000100000000000)
G_PIN_GPIO_0  =  (0b0000001000000000000)
G_PIN_GPIO_1  =  (0b0000010000000000000)
G_PIN_GPIO_2  =  (0b0000100000000000000)
G_PIN_GPIO_3  =  (0b0001000000000000000)
G_PIN_GPIO_4  =  (0b0010000000000000000)
G_PIN_GPIO_5  =  (0b0100000000000000000)

ppd = Path('../mdb/ppd.xml').resolve()
cpd = Path('../mdb/cpd.xml').resolve()
simlink = Path('../mdb/simlink.xml').resolve()
ccscds = Path('../mdb/cfs-ccsds.xml').resolve()
registry = '../mdb/registry.yaml'

print(f"Parsing XTCE...")

parser = XTCEParser([str(ppd), str(cpd), str(simlink)], str(ccscds), registry)

# comms = Communication(ParseMode.XTCE,
#                       parser,
#                       to_port=6011,
#                       address="172.16.30.3")

comms = Communication(ParseMode.XTCE,
                      parser,
                      to_port=6011,
                      address="127.0.0.1")
rky = Vehicle(
    vehicle_id='rocky',
    communication=comms
)

# with ScriptingWrapper(rky) as rocky:
#     rocky.ctrl.atp('Arm')
#     while rocky.nav.altitude is None:
#         time.sleep(1)
#         print ("Waiting for telemetry downlink...")

#     # rocky.ctrl.atp('Arm')
#     # rocky.ctrl.arm()
#     rky.logger.setLevel(40)



class ECMConfig():
    def __init__(self) -> None:
        self.ECM_MAX_OUTPUT_COMMANDS = 8
        self.ReleasePin: int  = 0
        self.ArmPins = [0 for i in range(self.ECM_MAX_OUTPUT_COMMANDS)]
        self.ArmPinCount = 0
        self.CommandPins = [0 for i in range(self.ECM_MAX_OUTPUT_COMMANDS)]
        self.CommandPinCount = 0
        self.CommandDelayUs = [0 for i in range(self.ECM_MAX_OUTPUT_COMMANDS)]
    

class GPIO_StateCmd():
    def __init__(self) -> None:
        self.gpioNumber = 0

    

# comms.send_message(Command("/cfs/cpd/apps/vm/Arm"))
class ECM_HkTlm_t():
    def __init__(self) -> None:
        self.ECM_MAX_OUTPUT_COMMANDS = 8
        self.Armed: bool = False
        self.Released: bool = False
        self.StartTime: int = 0
        self.EndTime: int = 0
        self.SequenceStep: int = 0
        self.ExecutionTimes: list = [0 for i in range(self.ECM_MAX_OUTPUT_COMMANDS)]
    


def PX4LIB_GetPX4TimeUs():
    nanos = time.monotonic_ns() 
    return nanos / 1000


class ECM():
    def __init__(self, config: ECMConfig, new_comms: Communication):
        self.HkTlm: ECM_HkTlm_t = ECM_HkTlm_t()
        self.Config: ECMConfig = config
        self.ArmCommand = Command("/cfs/cpd/apps/gpio/Arm")
        self.EngageCommand = Command("/cfs/cpd/apps/gpio/Engage")
        self.comms: Communication = new_comms
        self.gpio_status = 0
        self.actuator_armed = False
        self.nav = Nav(self.comms)
        self.nav.LoadJSON("/home/lgomez/projects/pyliner/pyliner/flight/mission_items.json")
        
        # ATP will be Pyliner's own internal command, as opposed to NAV
        # self.AtpCommand = Command("/cfs/cpd/apps/gpio/Engage")

    def IsVehicleReleased(self):
        return bool(self.gpio_status & ecm_app.Config.ReleasePin)
    
    def IsActuatorArmed(self):
        return self.actuator_armed
    
    def ECM_ArmGpioPins(self):
        for pin in self.Config.ArmPins:
            self.comms.send_message(Command("/cfs/cpd/apps/gpio/Arm", args={"gpioNumber": pin}) )
            # If I go any faster than this, SB starts dropping messages. See cfe _sb_api.c, Line 1291
            time.sleep(0.02)


    def ECM_SendNavAtp(self):
        # pass
        # TODO: Internal command as part of Pyliner NAV Mission sequence
        self.comms.send_message(Command("/cfs/cpd/apps/nav/ATP", args={}) )

    
    def ECM_RunMissonController(self):
        # if (!CVT.VehicleGlobalPosition.Timestamp == 0):
        self.nav.Execute()
    
    def ECM_RunController(self):
        self.ECM_RunMissonController()
        Released = False
        # print(f"self.Config.ArmPins:{self.Config.ArmPins}")
    # /* Check if actuator armed. */
        self.HkTlm.Armed = self.IsActuatorArmed()
        if self.HkTlm.Armed:
            Released = self.IsVehicleReleased()
            if Released:
                if False == self.HkTlm.Released:
                
                    # /* Latch. */
                    self.HkTlm.Released = True
                    self.HkTlm.StartTime = PX4LIB_GetPX4TimeUs()
                    # Internal Pyliner's NAV sequence ATP
                    self.ECM_SendNavAtp()
                    self.nav.ATP()
                    self.ECM_ArmGpioPins()
                    # (void) CFE_EVS_SendEvent(ECM_SEQUENCE_START_INF_EID, CFE_EVS_INFORMATION,
                    #                 "Sequence started %llu", ECM_AppData.HkTlm.StartTime)
                

            if self.HkTlm.Released:
                i: int        = 0
                TimeNow: int  = 0
                Delta: int    = 0

                # /* Get the current delta time. */
                TimeNow = PX4LIB_GetPX4TimeUs()
                # TimeNow = time.clock_gettime_ns(time.time.CLOCK_MONOTONIC)
                Delta = TimeNow - self.HkTlm.StartTime

                # /* Loop through all possible commands. */
                for i in range(self.Config.CommandPinCount):
                
                    # /* 
                    # * If delta time is greater than the delay and the command 
                    # * has not been executed. 
                    # */
                    if(Delta >= self.Config.CommandDelayUs[i] and
                    self.HkTlm.ExecutionTimes[i] == 0):
                        # /* Engage that pin. */
                        self.ECM_EngageGpioPin(self.Config.CommandPins[i])
                        # /* Increment the step. */
                        self.HkTlm.SequenceStep += 1
                        # /* Record the execution time. */
                        self.HkTlm.ExecutionTimes[i] = TimeNow
                

                # /* If all commands have been executed record the end time. */
                if 0 == self.HkTlm.EndTime and  self.HkTlm.SequenceStep >= (self.Config.CommandPinCount - 1):
                    self.HkTlm.EndTime = PX4LIB_GetPX4TimeUs()
                #     (void) CFE_EVS_SendEvent(ECM_SEQUENCE_END_INF_EID, CFE_EVS_INFORMATION,
                #                 "Sequence ended %llu", ECM_AppData.HkTlm.EndTime)

    def ECM_EngageGpioPin(self, number: int):
            self.comms.send_message(Command("/cfs/cpd/apps/gpio/Engage", args={"gpioNumber": number} ) )
            # If I go any faster than this, SB starts dropping messages. See cfe _sb_api.c, Line 1291
            time.sleep(0.02)
        
            

config = ECMConfig()
config.ECM_MAX_OUTPUT_COMMANDS = 8
config.ReleasePin  = G_PIN_GPIO_0
config.ArmPins = [1,2,4]
config.ArmPinCount = 3
config.CommandPinCount = 3
config.CommandPins = [1,2,4]
config.CommandDelayUs = [5966000, 5966000, 7966000]
    

ecm_app: ECM = ECM(config, comms)


# /* ArmPins (arming is completed at the start of the sequence). */
# {
#     /* GPIO pin 1 */
#     1,
#     /* GPIO pin 2 */
#     2,
#     /* GPIO pin 4 */
#     4
# },


def watch_gpio_status(tlm):
    # print(f"tlm.value************************:{tlm.value}")
    # print(f"tlm name: {tlm.name}")
    ecm_app.gpio_status = tlm.value

def watch_actuator_status(tlm):
    ecm_app.actuator_armed = tlm.value

def watch_PX4_VEHICLE_STATUS_MID(tlm):
    field_name = tlm.name.split(".")[1]
    # print(f"field_name:{field_name}")
    ecm_app.nav.CVT.VehicleStatusMsg.__setattr__(field_name, tlm.value)



home: PX4_HomePositionMsg_t = PX4_HomePositionMsg_t()
print(home)

# getattr(home)

comms.subscribe('/cfs/cpd/apps/gpio/GPIO_STATUS_TLM_MID.Status', callback=watch_gpio_status)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_ACTUATOR_ARMED_MID.Armed', callback=watch_actuator_status)

# PX4_VEHICLE_STATUS_MID
# TODO:Eventually will be a single subscription to the aggregate
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.Timestamp', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.SystemID', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.ComponentID', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.OnboardControlSensorsPresent', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.OnboardControlSensorsEnabled', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.OnboardControlSensorsHealth', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.NavState', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.ArmingState', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.HilState', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.Failsafe', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.SystemType', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.IsRotaryWing', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.IsVtol', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.VtolFwPermanentStab', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.InTransitionMode', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.RcSignalLost', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.RcInputMode', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.DataLinkLost', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.DataLinkLostCounter', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.EngineFailure', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.EngineFailureCmd', callback=watch_PX4_VEHICLE_STATUS_MID)
comms.subscribe('/cfs/cpd/apps/px4lib/PX4_VEHICLE_STATUS_MID.MissionFailure', callback=watch_PX4_VEHICLE_STATUS_MID)


# case PX4_HOME_POSITION_MID:
#     CFE_PSP_MemCpy(&CVT.HomePositionMsg, MsgPtr, sizeof(CVT.HomePositionMsg));
#     break;

# case PX4_MISSION_MID:
#     CFE_PSP_MemCpy(&CVT.MissionMsg, MsgPtr, sizeof(CVT.MissionMsg));
#     break;

# case PX4_VEHICLE_GLOBAL_POSITION_MID:
#     CFE_PSP_MemCpy(&CVT.VehicleGlobalPosition, MsgPtr,
#             sizeof(CVT.VehicleGlobalPosition));
#     break;

# case PX4_VEHICLE_STATUS_MID:
#     CFE_PSP_MemCpy(&CVT.VehicleStatusMsg, MsgPtr, sizeof(CVT.VehicleStatusMsg));
#     break;

# case PX4_VEHICLE_LAND_DETECTED_MID:
#     CFE_PSP_MemCpy(&CVT.VehicleLandDetectedMsg, MsgPtr,
#             sizeof(CVT.VehicleLandDetectedMsg));
#     break;

# case PX4_VEHICLE_LOCAL_POSITION_MID:
#     CFE_PSP_MemCpy(&CVT.VehicleLocalPositionMsg, MsgPtr,
#             sizeof(CVT.VehicleLocalPositionMsg));
#     break;

# case PX4_VEHICLE_COMMAND_MID:
#     NewCommandArrived = TRUE;
#     CFE_PSP_MemCpy(&CVT.VehicleCommandMsg, MsgPtr,
#             sizeof(CVT.VehicleCommandMsg));
#     break;




comms.send_message(Command("/cfs/cpd/apps/vm/Arm", args={}))

while True:
    ecm_app.ECM_RunController()
    # ecm_app.nav.Execute()
    time.sleep(0.01) # 100HZ


# comms.send_message(Command("/cfs/cpd/apps/gpio/Arm", args={"gpioNumber": 1}) )
# time.sleep(0.02)
# comms.send_message(Command("/cfs/cpd/apps/gpio/Arm", args={"gpioNumber": 2}) )
# time.sleep(0.02)
# comms.send_message(Command("/cfs/cpd/apps/gpio/Arm", args={"gpioNumber": 4}) )

