import struct
from typing import Optional

from .packet import END_BYTE, START_BYTE, Packet
from .types import CmdType


class Protocol:
    @staticmethod
    def create_ping() -> Packet:
        """Create ping packet"""
        return Packet(cmd_type=CmdType.PING)

    @staticmethod
    def create_move_servo(servo_id: int, position: int) -> Packet:
        """Create move servo packet (position: 0-1000)"""
        if not (0 <= servo_id <= 255):
            raise ValueError(f"Invalid servo_id: {servo_id}")
        if not (0 <= position <= 1000):
            raise ValueError(f"Invalid position: {position}")

        payload = struct.pack(">BH", servo_id, position)
        return Packet(cmd_type=CmdType.MOVE_SERVO, payload=payload)

    @staticmethod
    def create_set_gpio(pin: int, state: bool) -> Packet:
        """Create GPIO set packet"""
        if not (0 <= pin <= 255):
            raise ValueError(f"Invalid pin: {pin}")

        payload = struct.pack(">BB", pin, 1 if state else 0)
        return Packet(cmd_type=CmdType.SET_GPIO, payload=payload)

    @staticmethod
    def create_get_imu() -> Packet:
        """Create get IMU packet"""
        return Packet(cmd_type=CmdType.GET_IMU)

    @staticmethod
    def create_start_imu_stream(rate_hz: int) -> Packet:
        """Create start IMU stream packet"""
        if not (1 <= rate_hz <= 200):
            raise ValueError(f"Invalid rate: {rate_hz}")

        payload = struct.pack(">B", rate_hz)
        return Packet(cmd_type=CmdType.START_IMU_STREAM, payload=payload)

    @staticmethod
    def create_stop_imu_stream() -> Packet:
        """Create stop IMU stream packet"""
        return Packet(cmd_type=CmdType.STOP_IMU_STREAM)

    @staticmethod
    def parse_imu_data(packet: Packet) -> Optional[dict]:
        """Parse IMU data from packet"""
        if packet.cmd_type != CmdType.IMU_DATA:
            return None

        if len(packet.payload) != 24:  # 6 floats
            return None

        values = struct.unpack(">6f", packet.payload)
        return {
            "accel": {"x": values[0], "y": values[1], "z": values[2]},
            "gyro": {"x": values[3], "y": values[4], "z": values[5]},
        }


__all__ = ["Protocol", "Packet", "CmdType", "START_BYTE", "END_BYTE"]
