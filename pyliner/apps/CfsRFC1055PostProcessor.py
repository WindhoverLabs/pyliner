import struct

from xtce.msg_post_processor import MsgPostProcessor

from pyliner.message import MessageType

END = 0xc0
ESC = 0xdb
ESC_END = 0xdc
ESC_ESC = 0xdd


class CfsRFC1055PostProcessor(MsgPostProcessor):
    def __init__(self, msg_type: MessageType):
        self.type = msg_type

    def process(self, msg: bytes) -> bytes:
        msg_bytes = bytearray(msg)
        # Make this configurable as this depends on Cfs versions
        tmp = msg_bytes[6]
        msg_bytes[7] = tmp
        msg_bytes[6] = 0
        payload = bytearray()
        header_size = None
        if self.type == MessageType.COMMAND:
            header_size = 8
        elif self.type == MessageType.TELEMETRY:
            header_size = 12
        for character in msg_bytes:
            if len(payload) < header_size:
                value = struct.unpack('>B', character.to_bytes(1, "big"))[0]  # big-endian
            else:
                # Make this endian an argument
                value = struct.unpack('B', character.to_bytes(1, "little"))[0]  # little-endian
            if value == END:
                value = ESC
                payload.append(value)
                value = ESC_END
                payload.append(value)
            elif value == ESC:
                value = ESC
                payload.append(value)
                value = ESC_ESC
                payload.append(value)
                payload.append(character)
            else:
                payload.append(character)

        payload.append(END)
        return bytes(payload)
