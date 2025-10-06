from dataclasses import dataclass
from typing import Optional

from botzo_sdk.protocol.types import CMDType

START_BYTE = 0x3C  # 0x3C = '<'
"""Packet start delimiter byte."""

END_BYTE = 0x3E  # 0x3E = '>'
"""Packet end delimiter byte."""

MAX_PAYLOAD_SIZE = 254  # bytes
"""Maximum allowed payload size in bytes."""


@dataclass
class Packet:
    """Data packet for bidirectional Host-MCU communication.

    Implements a framed binary protocol with checksum verification for reliable
    serial communication. Each packet contains a command type and optional payload.

    Packet Structure:
        [START_BYTE] [LENGTH] [CMD_TYPE] [PAYLOAD...] [CHECKSUM] [END_BYTE]

        - START_BYTE (1 byte): Packet start delimiter (0x3C, '<')
        - LENGTH (1 byte): Payload length in bytes (0-254)
        - CMD_TYPE (1 byte): Command type identifier from CMDType enum
        - PAYLOAD (0-254 bytes): Command-specific data
        - CHECKSUM (1 byte): XOR checksum of LENGTH, CMD_TYPE, and PAYLOAD
        - END_BYTE (1 byte): Packet end delimiter (0x3E, '>')

    Minimum packet size: 5 bytes (empty payload)
    Maximum packet size: 259 bytes (255-byte payload)

    Attributes:
        cmd_type (CMDType): The command type for this packet.
        payload (bytes): Optional command-specific data. Defaults to empty bytes.

    Examples:
        >>> # Create a PING packet with no payload
        >>> packet = Packet(cmd_type=CMDType.PING)
        >>> data = packet.serialize()

        >>> # Create a packet with payload
        >>> packet = Packet(cmd_type=CMDType.MOVE_SERVO, payload=b'\\x01\\x5A')
        >>> data = packet.serialize()

        >>> # Parse received bytes
        >>> received_packet = Packet.deserialize(data)
        >>> if received_packet:
        ...     print(f"Received: {received_packet.cmd_type.name}")
    """

    cmd_type: CMDType
    payload: bytes = b""

    def serialize(self) -> bytes:
        """Serialize the packet into bytes for transmission.

        Constructs a framed packet with start/end delimiters and XOR checksum
        for error detection.

        Returns:
            bytes: The complete serialized packet ready for transmission.

        Raises:
            ValueError: If payload size exceeds MAX_PAYLOAD_SIZE (255 bytes).

        Note:
            Checksum is calculated as: LENGTH XOR CMD_TYPE XOR payload[0] XOR ... XOR payload[n]
        """
        length = len(self.payload)
        if length > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {length} > {MAX_PAYLOAD_SIZE}")

        packet = bytearray([START_BYTE, length, self.cmd_type.value])
        packet.extend(self.payload)

        # Calculate checksum (XOR of length, command, and payload bytes)
        checksum = length ^ self.cmd_type.value
        for b in self.payload:
            checksum ^= b

        packet.append(checksum)
        packet.append(END_BYTE)

        return bytes(packet)

    @staticmethod
    def deserialize(data: bytes) -> Optional["Packet"]:
        """Deserialize bytes into a Packet object with validation.

        Parses received bytes, validates packet structure, verifies checksum,
        and constructs a Packet object if all checks pass.

        Args:
            data (bytes): Raw bytes received from serial port or buffer.

        Returns:
            Optional[Packet]: A valid Packet object if parsing and validation
                succeed, None otherwise.

        Validation Checks:
            - Minimum length (5 bytes)
            - Correct START_BYTE and END_BYTE delimiters
            - Valid checksum
            - Valid command type (exists in CMDType enum)

        Examples:
            >>> raw_data = b'<\\x02\\x01\\x05\\x0A\\x08>'
            >>> packet = Packet.deserialize(raw_data)
            >>> if packet:
            ...     print(f"Valid packet: {packet.cmd_type.name}")
            ... else:
            ...     print("Invalid or corrupted packet")
        """
        # Minimum packet: [START][LEN][CMD][CHECKSUM][END] = 5 bytes
        if len(data) < 5:
            return None

        # Verify delimiters
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

        # Validate command type
        try:
            cmd_type = CMDType(cmd_value)
        except ValueError:
            return None

        return Packet(cmd_type=cmd_type, payload=payload)
