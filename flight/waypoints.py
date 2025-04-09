from pyliner import Vehicle
from pyliner.apps.communication import Communication
from pyliner.position import Waypoint
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json

from time import sleep

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
    
    cur_alt = rocky.nav.position.altitude
    
    # Generate waypoints
    wpts = []
    wpts.append(Waypoint(47.3977271, 8.5455870, cur_alt+10, 0))
    wpts.append(Waypoint(47.3977379, 8.5455979, cur_alt+10, 90))
    wpts.append(Waypoint(47.3978289, 8.5456889, cur_alt+10, 180))
    wpts.append(Waypoint(47.3979299, 8.5457899, cur_alt+10, 270))
    wpts.append(Waypoint(47.3977271, 8.5455870, cur_alt+10, 360))

    # Goto
    rocky.nav.goto(tolerance=0.75)(wpts)
    rocky.ctrl.rtl()

