from socketserver import TCPServer
from abc import abstractmethod
from threading import Thread

from pyliner.app import App


class SocketApp(App):
    """The SocketApp listens to a TCP port for vehicle commands or other data.

    Implementing classes must define a handle method that takes the request and
    does something with it. See CommanderApp for an example.
    """

    def __init__(self, port):
        super(SocketApp, self).__init__()
        self.port = port
        self.server = None
        """:type: TCPServer"""

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.port)

    @abstractmethod
    def handle(self, request, client_address):
        raise NotImplementedError()

    def attach(self, vehicle_wrapper):
        super(SocketApp, self).attach(vehicle_wrapper)
        self.info('Starting SocketService on port {}'.format(self.port))
        self.server = TCPServer(
            ('', self.port),
            lambda request, client_address, _self:
                self.handle(request, client_address))
        self.server.service = self

        listen_thread = Thread(
            name=self.__class__.__name__ + 'Thread',
            target=self.server.serve_forever)
        listen_thread.daemon = True
        listen_thread.start()

    def detach(self):
        self.info('Stopping SocketService on port {}'.format(self.port))
        self.server.shutdown()
        super(SocketApp, self).detach()
