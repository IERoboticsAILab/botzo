from dataclasses import dataclass
from typing import Optional

from client.protocol.types import CmdType

START_BYTE = 0x3C  # 0x3C = '<'
END_BYTE = 0x3E  # 0x3E = '>'
MAX_PAYLOAD_SIZE = 255  # bytes


@dataclass
class Packet:
    """This defines the packet that is sent between Host and MCU.

    NOTE: The packet is defined as follows:
        [START_BYTE] [LENGTH] [COMMAND_ID] [PAYLOAD...] [CHECKSUM] [END_BYTE]
    """

    cmd_type: CmdType
    payload: bytes = b""

    def to_bytes(self) -> bytes:
        length = len(self.payload)
        if length > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {length} > {MAX_PAYLOAD_SIZE}")

        packet = bytearray([START_BYTE, length, self.cmd_type.value])
        packet.extend(self.payload)

        # Calculate checksum
        checksum = length ^ self.cmd_type.value
        for b in self.payload:
            checksum ^= b

        packet.append(checksum)
        packet.append(END_BYTE)
        return bytes(packet)

    @staticmethod
    def from_bytes(data: bytes) -> Optional["Packet"]:
        if len(data) < 5:
            return None

        if data[0] != START_BYTE or data[-1] != END_BYTE:
            return None

        length = data[1]
        cmd_value = data[2]
        payload = data[3 : 3 + length]
        checksum = data[3 + length]

        # Verify checksum
        calc_checksum = length ^ cmd_value
        for b in payload:
            calc_checksum ^= b

        if calc_checksum != checksum:
            return None

        try:
            cmd_type = CmdType(cmd_value)
        except ValueError:
            return None

        return Packet(cmd_type=cmd_type, payload=payload)
