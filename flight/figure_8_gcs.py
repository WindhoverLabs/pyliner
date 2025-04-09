"""
Fly in a figure-8. Fly at full-speed during approach and retreat, and slow down
during the turn to allow for a smaller turn radius.

Requirements Fulfilled:
    PYLINER001
    PYLINER003
    PYLINER004
    PYLINER006
    PYLINER009
    PYLINER010
    PYLINER011
    PYLINER012
    PYLINER013
    PYLINER014
    PYLINER016
"""
import thread
import threading
from time import sleep

from pyliner import Vehicle
from pyliner.apps.communication import Communication
from pyliner.apps.controller import FlightMode
from pyliner.apps.navigation.control import constant, limiter, proportional
from pyliner.scripting_wrapper import ScriptingWrapper
from pyliner.util import read_json

FAST = 0.75
SLOW = 0.50
SLEEP = 0.1


def range_limit(current, target):
    return limiter(0, 0.25)(proportional(0.1 / 50.0)(current, target))


rky = Vehicle(
    vehicle_id='rocky',
    communication=Communication(
        airliner_map=read_json("airliner.json"),
        address="192.168.1.2",
        ci_port=5009,
        to_port=5012)
)

with ScriptingWrapper(rky) as rocky:
    while rocky.nav.altitude == "NULL":
        sleep(1)
        print "Waiting for telemetry downlink..."

    rocky.ctrl.atp('Arm')
    rocky.ctrl.arm()
    rocky.ctrl.atp('Takeoff')
    rocky.ctrl.takeoff()
    rocky.ctrl.flight_mode(FlightMode.PosCtl)

    rocky.ctrl.atp('Move Up')
    rocky.nav.vnav(method=constant(0.5), tolerance=0.5).up(10)
    home = rocky.nav.position

    # Bearing to home
    def home_bear():
        return rocky.geographic.bearing(rocky.nav.position, home)

    def figure8():
        def check():
            if not running:
                thread.exit()

        direction = -1
        while True:
            rocky.fd.x = FAST
            # Wait until past home
            print('Approach')
            old_dist = float('inf')
            while True:
                dist = rocky.geographic.distance(home, rocky.nav.position)
                if dist > old_dist:
                    break
                old_dist = dist
                sleep(SLEEP)
                check()

            # Wait until away from homeposition
            print('Retreat')
            distance = rocky.geographic.distance(home, rocky.nav.position)
            while distance < 7:
                distance = rocky.geographic.distance(home, rocky.nav.position)
                print('Retreat {} {} {}'.format(round(rocky.fd.x, 3), distance, rocky.nav.position))
                sleep(SLEEP)
                check()

            # Flip directions
            rocky.fd.x = SLOW
            direction *= -1

            print('Rotate')
            diff = float('inf')
            while diff > 4:
                bear = home_bear()
                diff = rocky.nav.heading - bear
                diff = diff % 360
                diff = diff if diff < 180 else -(diff - 360)
                rocky.fd.r = direction * abs(limiter(-0.25, 0.25)(diff / 100))
                sleep(SLEEP)
                check()

            # Stop rotating
            rocky.fd.r = 0.0


    fig = threading.Thread(target=figure8)
    running = True
    try:
        fig.start()
        raw_input('Press enter to stop\n')
    except KeyboardInterrupt:
        pass
    running = False

    fig.join()
    rocky.ctrl.rtl()
