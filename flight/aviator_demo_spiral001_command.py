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

print(f"ppd-->{ppd}")

parser = XTCEParser([str(ppd), str(cpd), str(simlink)], str(ccscds), registry)

comms = Communication(ParseMode.XTCE,
                      parser,
                      to_port=6011)
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

    

# comms.send_message(Command("/cfs/cpd/apps/vm/Arm"))
class ECM_HkTlm_t():
    ECM_MAX_OUTPUT_COMMANDS = 8
    def __init__(self) -> None:
        self.Armed = False
        self.Released = False
        self.StartTime = 0
        self.EndTime = 0
        self.SequenceStep = 0
        self.ExecutionTimes = []


ecm: ECM_HkTlm_t = ECM_HkTlm_t()



def IsVehicleReleased(status):
    return status == G_PIN_GPIO_0


def IsActuatorArmed(Armed):
    isArmed = False

    if Armed:
        isArmed = True

    return isArmed



def ECM_SendNavAtp():
    comms.send_message(Command("/cfs/cpd/apps/nav/ATP"))

def ECM_ArmGpioPins():  
    comms.send_message(Command("/cfs/cpd/apps/gpio"), {"gpioNumber:": 1})
    comms.send_message(Command("/cfs/cpd/apps/gpio"), {"gpioNumber:": 2})
    comms.send_message(Command("/cfs/cpd/apps/gpio"), {"gpioNumber:": 4})

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
    # print("Watch GPIO:", tlm.value)
    ecm.Armed = IsActuatorArmed()
    if ecm.Armed:
         ecm.Released = IsVehicleReleased(tm.value)
        if IsVehicleReleased(tlm.value):
            ECM_SendNavAtp()
            # Pin was
            ECM_ArmGpioPins()
            print("Released************")

comms.subscribe('/cfs/cpd/apps/gpio/GPIO_STATUS_TLM_MID.Status', callback=watch_gpio_status)
while True:
    pass
# comms.send_message(Command("/cfs/cpd/apps/vm/Arm"))