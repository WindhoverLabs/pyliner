from time import sleep

from pyliner import Vehicle
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
    while rocky.nav.altitude == "NULL":
        sleep(1)
        print "Waiting for telemetry downlink..."

    home = rocky.nav.position

    while True:
        sleep(1.0)
        # raw_input('Press Enter To Log: ')
        position = rocky.nav.position
        print('Distance: {}'.format(rocky.geographic.distance(home, position)))
