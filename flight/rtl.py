import time

from pyliner import Vehicle
from pyliner.action import ACTION_RTL
from pyliner.apps.communication import Communication
from pyliner.intent import Intent
from pyliner.util import read_json

vehicle = Vehicle(
    vehicle_id='rocky',
    communication=Communication(
        ci_port=5009, to_port=5012
    )
)

vehicle.broadcast(Intent(action=ACTION_RTL))
time.sleep(1)
