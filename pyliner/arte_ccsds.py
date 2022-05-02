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
import ctypes
import struct 
import time


"""CCSDS Space Packet Protocol ctype structure definitions.

Note: ctypes.BigEndianStructure seems to have a bug bit fields defined 
with bigendianstructure seem to create fields in random order. Using 
ctypes Structure for now. ctypes uses the native byte order for 
Structures and Unions. For issues with alignment use the pragma pack
ctype equivalent _pack_ = n.

"""


class stream_id_bits(ctypes.Structure):
    """Primary header packet identifier word (stream ID) bit fields.

    Application ID
    Secondary header flag (0 = absent, 1 = present)
    Packet type (0 = TLM, 1 = CMD)
    CCSDS version (always set to 0)
    """
    _fields_ = [("app_id", ctypes.c_uint16, 11),
                ("shdr_flag", ctypes.c_uint16, 1),
                ("type", ctypes.c_uint16, 1),
                ("version", ctypes.c_uint16, 3)]


class sequence_bits(ctypes.Structure):
    """Primary header packet sequence word bit fields.

    Sequence count
    segmentation flags: (3 = complete packet)
    """
    _fields_ = [("count", ctypes.c_uint16, 14),
                ("seq_flags", ctypes.c_uint16, 2)]


class command_bits(ctypes.Structure):
    """Command secondary header bit fields.

    Checksum, calculated by ground system
    command function code
    reserved, set to 0
    """
    _fields_ = [("code", ctypes.c_uint16, 7),
                ("checksum", ctypes.c_uint16, 8),
                ("reserved", ctypes.c_uint16, 1)]


class StreamId(ctypes.Union):
    """Primary header packet identifier word (stream ID) union."""
    _fields_ = [("bits", stream_id_bits),
                ("data", ctypes.c_uint16)]


class Sequence(ctypes.Union):
    """Primary header packet sequence word."""
    _fields_ = [("bits", sequence_bits),
                ("data", ctypes.c_uint16)]


class Command(ctypes.Union):
    """Secondary command header packet sequence word union."""
    _fields_ = [("bits", command_bits),
                ("data", ctypes.c_uint16)]


class CCSDS_PriHdr_t(ctypes.Structure):
    """Primary header 
    
    Stream ID
    Sequence
    Length (total packet length)
    """
    _fields_ = [("StreamId", StreamId),
                ("Sequence", Sequence),
                ("Length", ctypes.c_uint16)]
                        
    def clear_header(self):
        """Clear the header contents."""
        self.StreamId.bits.app_id = 0
        self.StreamId.bits.shdr_flag = 0
        self.StreamId.bits.type = 0
        self.StreamId.bits.version = 0
        self.Sequence.bits.count = 0
        self.Sequence.bits.seq_flags = 0
        self.Length = 0
        
    def print_base2(self):
        """Print the primary header in binary."""
        print("CCSDS primary header :", struct.pack('>HHH', self.StreamId.data, self.Sequence.data, self.Length))
        print("StreamId :", bin(self.StreamId.data))
        print("    app_id    :", bin(self.StreamId.bits.app_id))
        print("    shdr_flag :", bin(self.StreamId.bits.shdr_flag))
        print("    type      :", bin(self.StreamId.bits.type))
        print("    version   :", bin(self.StreamId.bits.version))
        print("Sequence :", bin(self.Sequence.data))
        print("    count     :", bin(self.Sequence.bits.count))
        print("    seq_flags :", bin(self.Sequence.bits.seq_flags))
        print("Length        :", bin(self.Length))

    def print_base10(self):
        """Print the primary header in decimal."""
        print("CCSDS primary header :", struct.pack('>HHH', self.StreamId.data, self.Sequence.data, self.Length))
        print("StreamId :", self.StreamId.data)
        print("    app_id    :", self.StreamId.bits.app_id)
        print("    shdr_flag :", self.StreamId.bits.shdr_flag)
        print("    type      :", self.StreamId.bits.type)
        print("    version   :", self.StreamId.bits.version)
        print("Sequence :", self.Sequence.data)
        print("    count     :", self.Sequence.bits.count)
        print("    seq_flags :", self.Sequence.bits.seq_flags)
        print("Length        :", self.Length)

    def print_base16(self):
        """Print the primary header in hexidecimal."""
        print("CCSDS primary header :", struct.pack('>HHH', self.StreamId.data, self.Sequence.data, self.Length))
        print("StreamId :", hex(self.StreamId.data))
        print("    app_id    :", hex(self.StreamId.bits.app_id))
        print("    shdr_flag :", hex(self.StreamId.bits.shdr_flag))
        print("    type      :", hex(self.StreamId.bits.type))
        print("    version   :", hex(self.StreamId.bits.version))
        print("Sequence :", hex(self.Sequence.data))
        print("    count     :", hex(self.Sequence.bits.count))
        print("    seq_flags :", hex(self.Sequence.bits.seq_flags))
        print("Length        :", hex(self.Length))

    def get_encoded(self):
        """Encode the primary header.
        
        Returns:
            bytes: the primary header as a bytes object.
        """
        return struct.pack('>HHH', self.StreamId.data, self.Sequence.data, self.Length)

    def set_decoded(self, header):
        """Decode and set the values of the primary header.

        Note:
            unpack always returns a tuple.

        Args:
            header (bytes): The encoded primary header to decode and set.
        """
        self.StreamId.data, self.Sequence.data, self.Length = struct.unpack('>HHH', header)
        
    def set_app_id(self):
        """Not implemented."""
        pass
        
    def get_app_id(self):
        """Not implemented."""
        pass
    
    def set_shdr_flag(self):
        """Not implemented."""
        pass
    
    def get_shdr_flag(self):
        """Not implemented."""
        pass
    
    def set_type(self):
        """Not implemented."""
        pass
    
    def get_type(self):
        """Not implemented."""
        pass

    def set_version(self):
        """Not implemented."""
        pass

    def get_version(self):
        """Not implemented."""
        pass
    
    def set_count(self):
        """Not implemented."""
        pass
        
    def get_count(self):
        """Not implemented."""
        pass
    
    def set_seq_flags(self):
        """Not implemented."""
        pass

    def set_length(self, value):
        """Set the packet data length
        
        Per CCSDS 133.0-B-1 Space Packet Protocol 
        The length count C shall be expressed as:
        C = (Total Number of Octets in the Packet Data Field) - 1
        
        Plus the length of the primary header - 6 bytes
        
        Args:
            value (int): Total packet length in bytes
        """
        self.Length = value - 7

    def get_length(self):
        """Get the packet data length.
        
        Per CCSDS 133.0-B-1 Space Packet Protocol 
        The length count C shall be expressed as:
        C = (Total Number of Octets in the Packet Data Field) - 1
        
        Plus the length of the primary header - 6 bytes
        
        Returns:
            int: Total packet length in bytes
        """
        return self.Length + 7
    
    def get_header_size(self):
        """Returns the length of the primary header using ctypes.sizeof.
        
        Returns:
            int: Total primary header size in bytes.
        """
        return ctypes.sizeof(self)


class CCSDS_CmdSecHdr_t(ctypes.Structure):
    """Secondary command header."""
    _fields_ = [("Command", Command)]

    def clear_header(self):
        """Clear the header contents."""
        self.Command.bits.checksum = 0
        self.Command.bits.code = 0
        self.Command.bits.reserved = 0

    def print_base2(self):
        """Print the secondary command header in binary."""
        print("CCSDS command secondary header ", struct.pack('>H', self.Command.data))
        print("Command :", bin(self.Command.data))
        print("    checksum :", bin(self.Command.bits.checksum))
        print("    code     :", bin(self.Command.bits.code))
        print("    reserved :", bin(self.Command.bits.reserved))

    def print_base10(self):
        """Print the secondary command header in decimal."""
        print("CCSDS command secondary header :", struct.pack('>H', self.Command.data))
        print("Command :", self.Command.data)
        print("    checksum :", self.Command.bits.checksum)
        print("    code     :", self.Command.bits.code)
        print("    reserved :", self.Command.bits.reserved)

    def print_base16(self):
        """Print the secondary command header in hexidecimal."""
        print("CCSDS command secondary header :", struct.pack('>H', self.Command.data))
        print("Command :", hex(self.Command.data))
        print("    checksum :", hex(self.Command.bits.checksum))
        print("    code     :", hex(self.Command.bits.code))
        print("    reserved :", hex(self.Command.bits.reserved))

    def get_encoded(self):
        """Encode the secondary command header.
        
        Returns:
            bytes: the secondary command header as a bytes object.
        """
        return struct.pack('>H', self.Command.data)

    def set_decoded(self, sec_header):
        """Decode and set the values of the secondary command header.
        
        Note:
            unpack always returns a tuple.
        
        Args:
            header (bytes): The encoded secondary header to decode and 
            set.
        """
        self.Command.data, = struct.unpack('>H', sec_header)

    def get_header_size(self):
        """Returns the length of the command secondary header using 
        ctypes.sizeof.
        
        Returns:
            int: Total primary header size in bytes.
        """
        return ctypes.sizeof(self)


class Time(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("Seconds", ctypes.c_uint32),
                ("Subseconds", ctypes.c_uint16)]


# for CFE_SB_PACKET_TIME_FORMAT == CFE_SB_TIME_32_16_SUBS
# CCSDS_TIME_SIZE 6
class CCSDS_TlmSecHdr_t(ctypes.Structure):
    """Secondary telemetry header."""
    _fields_ = [("Time", Time)]
    
    def clear_header(self):
        """Clear the header contents."""
        self.Time.Seconds = 0
        self.Time.Subseconds = 0

    def print_base2(self):
        """Print the secondary telemetry header in binary."""
        print("CCSDS telemetry secondary header ", struct.pack('>iH', self.Time.Seconds, self.Time.Subseconds))
        print("Time Seconds    :", bin(self.Time.Seconds))
        print("Time Subseconds :", bin(self.Time.Subseconds))

    def print_base10(self):
        """Print the secondary telemetry header in decimal."""
        print("CCSDS telemetry secondary header ", struct.pack('>iH', self.Time.Seconds, self.Time.Subseconds))
        print("Time Seconds    :", self.Time.Seconds)
        print("Time Subseconds :", self.Time.Subseconds)

    def print_base16(self):
        """Print the secondary telemetry header in hexidecimal."""
        print("CCSDS telemetry secondary header ", struct.pack('>iH', self.Time.Seconds, self.Time.Subseconds))
        print("Time Seconds    :", hex(self.Time.Seconds))
        print("Time Subseconds :", hex(self.Time.Subseconds))
        
    def get_encoded(self):
        """Encode the secondary telemetry header.
        
        Returns:
            bytes: the secondary telemetry header as a bytes object.
        """
        return struct.pack('>iH', self.Time.Seconds, self.Time.Subseconds)

    def set_decoded(self, sec_header):
        """Decode and set the values of the secondary telemetry header.
        
        Note:
            unpack always returns a tuple.
        
        Args:
            header (bytes): The encoded secondary header to decode and 
            set.
        """
        self.Time.Seconds, self.Time.Subseconds = struct.unpack('>iH', sec_header)
        
    def set_time(self, seconds, sub_seconds):
        """Set the time field.

        Note:
            CFS has other time formats besides 32 bits seconds + 16 bits
            subseconds.

        Args:
            Seconds (int32): Time seconds since epoch.
            Subseconds (int16): Time subseconds since epoch.  
        """
        self.Time.Seconds = seconds
        self.Time.Subseconds = sub_seconds
    
    def get_time(self):
        """Get the time field.
        
        Returns:
            tuple: Seconds (int32) and subseconds (int16)
        """
        return self.Time.Seconds, self.Time.Subseconds

    def get_header_size(self):
        """Returns the length of the telemetry secondary header using 
        ctypes.sizeof.
        
        Returns:
            int: Total primary header size in bytes.
        """
        return ctypes.sizeof(self)


class CCSDS_CmdPkt_t(ctypes.Structure):
    """Generic combined command header."""
    _fields_ = [("PriHdr", CCSDS_PriHdr_t),
                ("SecHdr", CCSDS_CmdSecHdr_t)]

    def init_packet(self):
        """Initialize the packet with the proper values for a command.
        
        Note:
            Only initializes values that apply to all command packets.
        """
        #self.PriHdr.StreamId.bits.app_id = 0
        self.PriHdr.StreamId.bits.shdr_flag = 1
        self.PriHdr.StreamId.bits.type = 1
        #self.PriHdr.StreamId.bits.version = 0
        #self.PriHdr.StreamId.bits.count = 0
        #self.PriHdr.StreamId.bits.seq_flags = 0
        #self.PriHdr.Length = 0
        #self.SecHdr.Command.bits.checksum = 0
        #self.SecHdr.Command.bits.code = 0
        #self.SecHdr.Command.bits.reserved = 0

    def clear_packet(self):
        """Clear the packet contents."""
        self.PriHdr.clear_header()
        self.SecHdr.clear_header()

    def print_base2(self):
        """Print the packet in binary."""
        self.PriHdr.print_base2()
        self.SecHdr.print_base2()

    def print_base10(self):
        """Print the packet in decimal."""
        self.PriHdr.print_base10()
        self.SecHdr.print_base10()

    def print_base16(self):
        """Print the packet in hexidecimal"""
        self.PriHdr.print_base16()
        self.SecHdr.print_base16()

    def get_encoded(self):
        """Encode the command packet.
        
        Returns:
            bytes: the command packet as a bytes object.
        """
        return self.PriHdr.get_encoded() + self.SecHdr.get_encoded()

    def set_decoded(self, cmd_packet):
        """Decode and set the values of the command packet.
        
        Args:
            cmd_packet (bytes): The encoded command packet to decode 
                and set.
        """
        self.PriHdr.set_decoded(cmd_packet[0:6])
        self.SecHdr.set_decoded(cmd_packet[6:8])

    def set_length(self, value):
        """Set the packet data length.

        Args:
            value (int): Total payload length in bytes.
        """
        self.PriHdr.set_length(value)

    def get_length(self):
        """Get the packet data length.
    
        Returns:
            int: Total payload length in bytes.
        """
        return self.PriHdr.get_length()

    def get_packet_size(self):
        """Returns the length of the combined command packet using 
        ctypes.sizeof.
        
        Returns:
            int: Total combined command packet size in bytes.
        """
        return self.PriHdr.get_header_size() + self.SecHdr.get_header_size()

    def set_user_data_length(self, value):
        """Set the length of the user payload. 
        
        Note:
            This function automatically sets the length of the packet.
            
        Args:
            The length of the user payload in bytes.
        """
        self.set_length(self.get_packet_size() + value)

    def get_user_data_length(self):
        """Get the length of the user payload. 
        
        Note:
            This function automatically subtracts the length of the 
            packet.
            
        Returns:
            The length of the user payload in bytes.
        """
        return self.get_length() - self.get_packet_size()

    def compute_checksum(self, payload):
        """Compute the checksum for a command packet. The checksum 
        is the XOR of all bytes in the packet. A valid checksum is zero.
        
        Note:
            pass 0 as an argument if there is no payload.

        Args:
            payload (bytes): The encoded payload.

        Returns:
            int: The checksum.
        """
        check_sum = 0xFF
        encoded = self.get_encoded()

        for i in range(self.get_packet_size()):
            check_sum ^= int(encoded[i].encode('hex'), 16)

        for j in range(self.get_user_data_length()):
            check_sum ^= int(payload[j].encode('hex'), 16)
            
        return check_sum

    def set_checksum(self, payload):
        """Set the checksum for a command packet. 

        Note:
            pass 0 as an argument if there is no payload.

        Args:
            payload (bytes): The encoded payload.
        """
        computed = self.compute_checksum(payload)
        self.SecHdr.Command.bits.checksum = computed

    def validate_checksum(self, payload):
        """Validate checksum for a command packet. 

        Note:
            pass 0 as an argument if there is no payload.

        Returns:
            (bool): True if checksum of packet is valid; False if not.
        """
        validated = self.compute_checksum(payload)
        
        if validated == 0:
            return True
        else:
            return False


class CCSDS_TlmPkt_t(ctypes.Structure):
    """Generic combined telemetry header."""
    _fields_ = [("PriHdr", CCSDS_PriHdr_t),
                ("SecHdr", CCSDS_TlmSecHdr_t)]

    def init_packet(self):
        """Initialize the packet with the proper values for telemetry.
        
        Note:
            Only initializes values that apply to all telemetry packets.
        """
        #self.PriHdr.StreamId.bits.app_id = 0
        self.PriHdr.StreamId.bits.shdr_flag = 1
        self.PriHdr.StreamId.bits.type = 0
        #self.PriHdr.StreamId.bits.version = 0
        #self.PriHdr.StreamId.bits.count = 0
        #self.PriHdr.StreamId.bits.seq_flags = 0
        #self.PriHdr.Length = 0
        #self.SecHdr.Time.Seconds = 0
        #self.SecHdr.Time.Subseconds = 0

    def clear_packet(self):
        """Clear the packet contents."""
        self.PriHdr.clear_header()
        self.SecHdr.clear_header()

    def print_base2(self):
        """Print the packet in binary."""
        self.PriHdr.print_base2()
        self.SecHdr.print_base2()

    def print_base10(self):
        """Print the packet in decimal."""
        self.PriHdr.print_base10()
        self.SecHdr.print_base10()

    def print_base16(self):
        """Print the packet in hexidecimal."""
        self.PriHdr.print_base16()
        self.SecHdr.print_base16()

    def get_encoded(self):
        """Encode the telemetry packet.
        
        Returns:
            bytes: the telemetry packet as a bytes object.
        """
        return self.PriHdr.get_encoded() + self.SecHdr.get_encoded()

    def set_decoded(self, telemetry_packet):
        """Decode and set the values of the telemetry packet.
        
        Args:
            cmd_packet (bytes): The encoded telemetry packet to decode 
                and set.
        """
        self.PriHdr.set_decoded(telemetry_packet[0:6])
        self.SecHdr.set_decoded(telemetry_packet[6:12])

    def set_length(self, value):
        """Set the packet data length.

        Args:
            value (int): Total payload length in bytes.
        """
        self.PriHdr.set_length(value)

    def get_length(self):
        """Get the packet data length.

        Returns:
            int: Total payload length in bytes.
        """
        return self.PriHdr.get_length() 
        
    def set_current_time(self):
        """Set the time field to the current system time.

        Note:
            CFS has other time formats besides 32 bits seconds + 16 bits
            subseconds. time.time() returns time in seconds since the
            epoch as a floating point number. A conversion from a 
            floating point fraction to binary fraction i.e. MSB = 2^-1.
        """
        float_time = time.time()
        int_time = int(float_time)
        fraction_time = float_time - int_time
        subseconds = 0
        
        for n in range(16):
            if (fraction_time/(2**-(n+1))) >= 1:
                subseconds |= 1 << (16 - (n+1)) 
                fraction_time -= 2**-(n+1)
        self.SecHdr.set_time(int_time, subseconds)

    def get_time(self):
        """Get the time field.

        Returns:
            tuple: Seconds (int32) and subseconds (int16) converted to
            a fraction.
        """
        bin_subseconds = self.SecHdr.Time.Subseconds
        fraction_subseconds = 0
        for n in range(16):
            if(bin_subseconds & 1 << (16 - (n+1))):
                fraction_subseconds += 2**-(n+1)
        return self.SecHdr.Time.Seconds, fraction_subseconds

    def get_packet_size(self):
        """Returns the length of the combined telemetry packet using 
        ctypes.sizeof.

        Returns:
            int: Total combined telemetry packet size in bytes.
        """
        return self.PriHdr.get_header_size() + self.SecHdr.get_header_size()

    def set_user_data_length(self, value):
        """Set the length of the user payload. 

        Note:
            This function automatically sets the length of the packet.

        Args:
            The length of the user payload in bytes.
        """
        self.set_length(self.get_packet_size() + value)

    def get_user_data_length(self):
        """Get the length of the user payload. 

        Note:
            This function automatically subtracts the length of the 
            packet.
            
        Returns:
            The length of the user payload in bytes.
        """
        return self.get_length() - self.get_packet_size()


#class float_bits(ctypes.BigEndianStructure):
    #_fields_ = [("mantisa", ctypes.c_ulonglong, 52),
                #("exponent", ctypes.c_ulonglong, 11),
                #("sign", ctypes.c_ulonglong, 1)]


#class Double_Float(ctypes.Union):
    #_fields_ = [("bits", float_bits),
                #("data", ctypes.c_ulonglong)]
        
    #def print_base2(self):
        #print("sign     :", bin(self.bits.sign))
        #print("exponent :", bin(self.bits.exponent))
        #print("mantisa  :", bin(self.bits.mantisa))


#class test_bits(ctypes.BigEndianStructure):
    #_fields_ = [("bit1", ctypes.c_uint16, 1),
                #("bit2", ctypes.c_uint16, 1),
                #("bit3", ctypes.c_uint16, 1),
                #("bit4", ctypes.c_uint16, 1),
                #("bit5", ctypes.c_uint16, 1),
                #("bit6", ctypes.c_uint16, 1),
                #("bit7", ctypes.c_uint16, 1),
                #("bit8", ctypes.c_uint16, 1),
                #("bit9", ctypes.c_uint16, 1),
                #("bit10", ctypes.c_uint16, 1),
                #("bit11", ctypes.c_uint16, 1),
                #("bit12", ctypes.c_uint16, 1),
                #("bit13", ctypes.c_uint16, 1),
                #("bit14", ctypes.c_uint16, 1),
                #("bit15", ctypes.c_uint16, 1),
                #("bit16", ctypes.c_uint16, 1)]
                
#class testing(ctypes.Union):
    #_fields_ = [("bits", test_bits),
                #("data", ctypes.c_uint16)]

