import json

from ..action import ACTION_GOTO
from ..position import Waypoint
from ..apps.socket_app import SocketApp
from pyliner.intent import Intent


class CommanderApp(SocketApp):
    """Receives a simple JSON-formatted list of waypoints and flies to them.

    This is a demo App that demonstrates the capability of the SocketApp for
    handling generic socket data.

    CommanderApp does not listen to any intents.
    """
    JSON_MAX_SIZE = 1024

    def handle(self, request, client_address):
        data = request.recv(CommanderApp.JSON_MAX_SIZE)
        j = json.loads(data)
        self.info('JSON from {}: {}'.format(client_address, j))
        if j['command'] == 'goto':
            waypoints = [Waypoint(
                latitude=w['lat'], longitude=w['lon'], altitude=w['alt'],
                heading=None) for w in j['waypoints']]
            goto = {'waypoints': waypoints, 'tolerance': j['tolerance']}
            self.vehicle.broadcast(Intent(action=ACTION_GOTO, data=goto))
        else:
            print('CommanderApp unable to route JSON.')

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.apps.commander'
