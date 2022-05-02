from pyliner import Vehicle
from pyliner.apps.communication import Communication
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json

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
    rocky.ctrl.atp('Begin Mission')
    rocky.ctrl.arm()
    rocky.ctrl.takeoff()

    # Generate waypoints
    last = rocky.nav.position
    last.altitude += 10
    wpts = []
    motion = [(0, 30), (90, 10), (180, 30), (90, 10),
              (0, 30), (90, 10), (180, 30), (90, 10),
              (0, 30), (90, 10), (180, 30)]
    for brng, dist in motion:
        last = geo.pbd(last, brng, dist)
        wpts.append(last)

    # Goto
    rocky.nav.goto(tolerance=0.75)(wpts)
    rocky.ctrl.rtl()

