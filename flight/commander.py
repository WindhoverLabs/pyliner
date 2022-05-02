import time

from pyliner import Vehicle
from pyliner.apps.commander_app import CommanderApp
from pyliner.apps.communication import Communication
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json

rky = Vehicle(
    vehicle_id='rocky',
    communication=Communication(
        airliner_map=read_json("airliner.json"),
        ci_port=5009,
        to_port=5012)
)

with ScriptingWrapper(rky) as rocky:
    rocky.await_change('/Airliner/CNTL/VehicleGlobalPosition/Alt',
                       'Waiting for telemetry downlink...')

    commander_app = CommanderApp(3396)
    rky.attach_app(commander_app)

    rocky.ctrl.atp('Arm')
    rocky.ctrl.arm()
    rocky.ctrl.atp('Takeoff')
    rocky.ctrl.takeoff()

    print('Waiting on port 3396')
    while True:
        time.sleep(1)
