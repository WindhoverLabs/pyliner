import socket
import socketserver


class MyUDPHandler(socketserver.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        data = self.request[0].strip()
        in_socket = self.request[1]
        # print("{} wrote:".format(self.client_address[0]))
        # print(data)
        # Send out data to all output ports

        UDP_IP = "127.0.0.1"
        out_ports = [5311, 5411]
        for port in out_ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # print("port:", port)
            sock.sendto(data, (UDP_IP, port))
        # socket.sendto(data.upper(), self.client_address)


def main():
    UDP_IP = "127.0.0.1"
    out_ports = [5311, 5411]
    MESSAGE = "Hello, World!"


    print("UDP target IP:", UDP_IP)
    print("UDP target port:", out_ports)
    print("message:", MESSAGE)

    HOST, in_port = "localhost", 5011

    while True:

    with socketserver.UDPServer((HOST, in_port), MyUDPHandler) as server:
        server.serve_forever()


main()