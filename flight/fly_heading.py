from pyliner import Vehicle
from pyliner.apps.communication import Communication
from pyliner.apps.controller import FlightMode
from pyliner.apps.navigation.control import limiter, proportional
from pyliner.position import Waypoint
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json
from pyliner.telemetry import SetpointTriplet

from math import radians
from time import sleep

def range_limit(current, target):
    return limiter(-0.2, 0.2)(proportional(0.1 / 40.0)(current, target))

vehicle = Vehicle(
    vehicle_id='rocky',
    communication=Communication(
        airliner_map=read_json('airliner.json'),
        ci_port=5009, to_port=5012
    )
)

with ScriptingWrapper(vehicle) as rocky:
    geo = rocky.geographic

    # Takeoff
    rocky.ctrl.arm()
    rocky.ctrl.takeoff()
    
    cur_alt = rocky.nav.position.altitude + 10
    triplet = SetpointTriplet(
                Cur_VX=5.0,
                Cur_CruisingSpeed=5.0,
                Cur_Alt=cur_alt,
                Cur_Valid=True,
                Cur_VelocityValid=True,
                Cur_VelocityFrame=8,
                Cur_Type=1,
                Cur_Yaw=rocky.fd.r,
                Next_Yaw=rocky.fd.r
            )
    
#    heading = rocky.nav.heading
#    target_heading = 0
#    yaw = rocky.nav.rotate(method=range_limit, tolerance=2)
#    if heading > target_heading:
#        yaw.counterclockwise(heading - target_heading)
#    else:
#        yaw.clockwise(target_heading - heading)
        
    # Goto
    
    rocky.com.send_command(triplet)
    sleep(20)
    rocky.ctrl.rtl()

