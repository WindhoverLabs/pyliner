import struct
import time
from xtce.msg_post_processor import MsgPostProcessor

from pyliner.message import MessageType

MIN_CMD_LENGTH = 7  # Assuming MIN_CMD_LENGTH is defined elsewhere
CHECKSUM_OFFSET = 7  # Assuming CHECKSUM_OFFSET is defined elsewhere
FC_OFFSET = 6  # Assuming FC_OFFSET is defined elsewhere

class CfsPostProcessor(MsgPostProcessor):
    def __init__(self, msg_type: MessageType):
        self.type = msg_type



    def preprocess_command(self, binary):
        # print("preprocess_command")
        if len(binary) < MIN_CMD_LENGTH:
            msg = f"Short command received, length: {len(binary)}, expected minimum length: {MIN_CMD_LENGTH}"
            print(msg)  # Assuming log.warn() is replaced with print() for simplicity
            t = int(time.time())  # Assuming TimeEncoding.getWallclockTime() returns current Unix timestamp

            # # Assuming these functions are defined elsewhere
            # publish_ack(pc.get_command_id(), "AcknowledgeSent_KEY", t, "NOK", msg)
            # command_failed(pc.get_command_id(), t, msg)
            return None

        # set packet length
        binary_length = len(binary) - 7
        binary[4:6] = (binary_length >> 8 & 0xFF, binary_length & 0xFF)

        # fill sequence
        # seq_count = seq_filler(binary)

        # set the checksum
        binary[CHECKSUM_OFFSET] = 0
        # checksum = 0xFF
        # for byte_val in binary:
        #     checksum ^= byte_val
        # binary[CHECKSUM_OFFSET] = checksum & 0xFF

        swap_checksum_fc = True
        # # Swap checksum and FC if needed
        if swap_checksum_fc:
            binary[CHECKSUM_OFFSET], binary[FC_OFFSET] = binary[FC_OFFSET], binary[CHECKSUM_OFFSET]

        # # Assuming these functions are defined elsewhere
        # # publish(pc.get_command_id(), "CNAME_BINARY", binary)
        return binary

    def process(self, msg: bytes) -> bytes:
        msg_bytes = bytearray(msg)
        return self.preprocess_command(msg_bytes)
