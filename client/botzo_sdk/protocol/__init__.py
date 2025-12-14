import struct
from typing import Optional

from botzo_sdk.protocol.packet import Packet
from botzo_sdk.protocol.types import CMDType

from .packet import END_BYTE, START_BYTE, Packet
from .types import CMDType


class Protocol:
    """High-level protocol interface for creating and parsing MCU packets.

    Provides factory methods for constructing command packets with proper
    payload encoding and validation. Also provides parsers for extracting
    structured data from response packets.

    All methods use little-endian byte order ('<') for multi-byte values in
    commands, to match ESP32 endianness.
    """

    @staticmethod
    def create_ping() -> Packet:
        """Create a connectivity check packet.

        Sends a PING command to verify MCU connectivity. Typically expects
        an ACK response.

        Returns:
            Packet: A PING packet with no payload.

        Example:
            >>> ping = Protocol.create_ping()
            >>> pong = await interface.send_and_wait(ping, CMDType.ACK)
        """
        return Packet(cmd_type=CMDType.PING)

    @staticmethod
    def create_move_servo(servo_id: int, position: int) -> Packet:
        """Create a servo movement command packet.

        Commands a single servo to move to the specified position.

        Args:
            servo_id (int): Servo identifier (0-255).
            position (int): Target position (0-1000), where 0 is minimum
                and 1000 is maximum servo range.

        Returns:
            Packet: A MOVE_SERVO packet with encoded servo ID and position.

        Raises:
            ValueError: If servo_id is not in range 0-255 or position is
                not in range 0-1000.

        Payload Format:
            [SERVO_ID: 1 byte][POSITION: 2 bytes, little-endian]

        Example:
            >>> # Move servo 3 to middle position
            >>> packet = Protocol.create_move_servo(servo_id=3, position=500)
            >>> await interface.send(packet)
        """
        if not (0 <= servo_id <= 255):
            raise ValueError(f"Invalid servo_id: {servo_id}")
        if not (0 <= position <= 1000):
            raise ValueError(f"Invalid position: {position}")

        payload = struct.pack("<BH", servo_id, position)
        return Packet(cmd_type=CMDType.MOVE_SERVO, payload=payload)

    @staticmethod
    def create_set_gpio(pin: int, state: bool) -> Packet:
        """Create a GPIO pin state command packet.

        Sets the digital state of a GPIO pin to high or low.

        Args:
            pin (int): GPIO pin number (0-255).
            state (bool): Desired pin state - True for HIGH, False for LOW.

        Returns:
            Packet: A SET_GPIO packet with encoded pin and state.

        Raises:
            ValueError: If pin number is not in range 0-255.

        Payload Format:
            [PIN: 1 byte][STATE: 1 byte (0=LOW, 1=HIGH)]

        Example:
            >>> # Set GPIO pin 5 to HIGH
            >>> packet = Protocol.create_set_gpio(pin=5, state=True)
            >>> await interface.send(packet)
        """
        if not (0 <= pin <= 255):
            raise ValueError(f"Invalid pin: {pin}")

        payload = struct.pack(">BB", pin, 1 if state else 0)
        return Packet(cmd_type=CMDType.SET_GPIO, payload=payload)

    @staticmethod
    def create_get_imu() -> Packet:
        """Create a single IMU reading request packet.

        Requests a one-time reading from the Inertial Measurement Unit.
        Response will be an IMU_DATA packet.

        Returns:
            Packet: A GET_IMU packet with no payload.

        Example:
            >>> packet = Protocol.create_get_imu()
            >>> response = await interface.send_and_wait(packet, CMDType.IMU_DATA)
            >>> imu_data = Protocol.parse_imu_data(response)
        """
        return Packet(cmd_type=CMDType.GET_IMU)

    @staticmethod
    def create_start_imu_stream(rate_hz: int) -> Packet:
        """Create a command to start continuous IMU data streaming.

        Begins periodic transmission of IMU_DATA packets at the specified rate.

        Args:
            rate_hz (int): Streaming frequency in Hz (1-200).

        Returns:
            Packet: A START_IMU_STREAM packet with encoded sampling rate.

        Raises:
            ValueError: If rate_hz is not in range 1-200.

        Payload Format:
            [RATE_HZ: 1 byte]

        Example:
            >>> # Stream IMU data at 50 Hz
            >>> packet = Protocol.create_start_imu_stream(rate_hz=50)
            >>> await interface.send(packet)
            >>>
            >>> # Register callback for incoming data
            >>> interface.on(CMDType.IMU_DATA, handle_imu)
        """
        if not (1 <= rate_hz <= 200):
            raise ValueError(f"Invalid rate: {rate_hz}")

        payload = struct.pack("<B", rate_hz)
        return Packet(cmd_type=CMDType.START_IMU_STREAM, payload=payload)

    @staticmethod
    def create_stop_imu_stream() -> Packet:
        """Create a command to stop continuous IMU data streaming.

        Stops the periodic transmission of IMU_DATA packets started by
        create_start_imu_stream().

        Returns:
            Packet: A STOP_IMU_STREAM packet with no payload.

        Example:
            >>> packet = Protocol.create_stop_imu_stream()
            >>> await interface.send(packet)
        """
        return Packet(cmd_type=CMDType.STOP_IMU_STREAM)

    @staticmethod
    def parse_imu_data(packet: Packet) -> Optional[dict]:
        """Parse IMU sensor data from a packet.

        Extracts accelerometer and gyroscope readings from an IMU_DATA packet.

        Args:
            packet (Packet): The packet to parse. Must be of type IMU_DATA.

        Returns:
            Optional[dict]: Dictionary containing IMU readings with structure:
                {
                    "accel": {"x": float, "y": float, "z": float},
                    "gyro": {"x": float, "y": float, "z": float}
                }
                Returns None if packet type is incorrect or payload is malformed.

        Payload Format (24 bytes, little-endian):
            [ACCEL_X: 4 bytes float][ACCEL_Y: 4 bytes float][ACCEL_Z: 4 bytes float]
            [GYRO_X: 4 bytes float][GYRO_Y: 4 bytes float][GYRO_Z: 4 bytes float]

        Example:
            >>> response = await interface.send_and_wait(
            ...     Protocol.create_get_imu(),
            ...     CMDType.IMU_DATA
            ... )
            >>> if response:
            ...     data = Protocol.parse_imu_data(response)
            ...     if data:
            ...         print(f"Accel X: {data['accel']['x']} m/s²")
            ...         print(f"Gyro Z: {data['gyro']['z']} rad/s")
        """
        if packet.cmd_type != CMDType.IMU_DATA:
            return None

        if len(packet.payload) != 24:  # 6 floats × 4 bytes
            return None

        # ESP32 is little-endian
        values = struct.unpack("<6f", packet.payload)

        return {
            "accel": {"x": values[0], "y": values[1], "z": values[2]},
            "gyro": {"x": values[3], "y": values[4], "z": values[5]},
        }


__all__ = ["Protocol", "Packet", "CMDType", "START_BYTE", "END_BYTE"]
