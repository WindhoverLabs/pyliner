"""

   Copyright (c) 2017 Windhover Labs, L.L.C. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
 3. Neither the name Windhover Labs nor the names of its 
    contributors may be used to endorse or promote products derived 
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

"""
import socket
from arte_ccsds import *


class ArteClient(object):
    """ArteClient for communicating with ArteServer over TCP.
    
    Args:
        ip (str): The IP address of the server.
        port (unsigned int): The port number of the server.
        
    Attributes:
        socket (:obj:`socket`): The instantiated socket object.
        sequence (uint): The current sequence count as received
            from ArteServer.
        frame (uint): The current frame count as received from 
            ArteServer.
        telemetry_packet (:obj:`CCSDS_TlmPkt_t`): A CCSDS telemetry 
            packet used for all communication to ArteServer.
        command_packet (:obj:`CCSDS_CmdPkt_t`): A CCSDS command packet
            used for all communication from ArteServer.
    """
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, port))
        self.sequence = 0
        self.frame = 0
        # Set up telemetry packet
        self.telemetry_packet = CCSDS_TlmPkt_t()
        self.telemetry_packet.clear_packet()
        self.telemetry_packet.init_packet()
        self.telemetry_packet.set_user_data_length(0)
        self.telemetry_packet_size = self.telemetry_packet.get_packet_size()
        # Set up command  packet
        self.command_packet = CCSDS_CmdPkt_t()
        self.command_packet.clear_packet()
        self.command_packet.init_packet()
        self.command_packet.set_user_data_length(0)

    def send_ready(self):
        """Sends a ready notification to ArteServer."""
        #TODO add try catch
        print("sending ready to ARTE server")
        self.telemetry_packet.set_current_time()
        self.sock.sendall(self.telemetry_packet.get_encoded())
        
    def send_shutdown(self, successBool):
        """Sends a shutdown notification to ArteServer.
        
        Args:
            successBool: (boolean): Notify ArteServer that that test(s)
                completed with success or failure.
        """
        #TODO add try catch
        print("sending shutdown to ARTE server")
        if successBool == True:
            self.telemetry_packet.PriHdr.StreamId.bits.app_id = 1
        elif successBool == False:
            self.telemetry_packet.PriHdr.StreamId.bits.app_id = 2
        self.telemetry_packet.set_current_time()
        self.sock.sendall(self.telemetry_packet.get_encoded())
        self.telemetry_packet.PriHdr.StreamId.bits.app_id = 0
    
    def receive_response(self):
        """Receive a response from ArteServer.

        Note:
            Currently pends forever. 
        """
        #TODO add timeout to the recv or use select().
        print("waiting for response from ARTE server")
        response = self.sock.recv(self.command_packet.get_packet_size())
        self.command_packet.set_decoded(response)
        self.decode_sequence()
    
    def decode_sequence(self):
        """Get and set the sequence count from a received ArteServer 
            command packet.
        """
        self.sequence = self.command_packet.PriHdr.Sequence.bits.count
        print("sequence count from ARTE server = ", self.sequence)
        
    def decode_frame(self):
        """Get and set the frame count from a received ArteServer 
            command packet.
        """
        self.frame = self.command_packet.SecHdr.Command.bits.code
        print("frame count from ARTE server = ", self.frame)
        
    def close_conn(self):
        """Close the TCP socket object."""
        self.sock.close()
