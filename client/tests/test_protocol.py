import struct

import pytest

# Assuming your protocol files are importable
from client.protocol import CmdType, Packet, Protocol
from client.protocol.packet import END_BYTE, START_BYTE


class TestProtocol:
    """Test protocol packet creation and parsing"""

    def test_packet_serialization(self):
        """Test packet to bytes conversion"""
        packet = Packet(cmd_type=CmdType.PING)
        data = packet.to_bytes()

        assert data[0] == START_BYTE
        assert data[1] == 0  # No payload
        assert data[2] == CmdType.PING.value
        assert data[-1] == END_BYTE

    def test_packet_deserialization(self):
        """Test bytes to packet conversion"""
        original = Packet(cmd_type=CmdType.PING)
        data = original.to_bytes()
        parsed = Packet.from_bytes(data)

        assert parsed is not None
        assert parsed.cmd_type == CmdType.PING
        assert parsed.payload == b""

    def test_packet_with_payload(self):
        """Test packet with payload"""
        payload = struct.pack(">BH", 5, 500)
        packet = Packet(cmd_type=CmdType.MOVE_SERVO, payload=payload)
        data = packet.to_bytes()
        parsed = Packet.from_bytes(data)

        assert parsed is not None
        assert parsed.cmd_type == CmdType.MOVE_SERVO
        assert parsed.payload == payload

    def test_checksum_validation(self):
        """Test checksum rejects corrupted data"""
        packet = Packet(cmd_type=CmdType.PING)
        data = bytearray(packet.to_bytes())

        # Corrupt checksum
        data[-2] ^= 0xFF

        parsed = Packet.from_bytes(bytes(data))
        assert parsed is None

    def test_protocol_create_ping(self):
        """Test Protocol.create_ping()"""
        packet = Protocol.create_ping()
        assert packet.cmd_type == CmdType.PING
        assert len(packet.payload) == 0

    def test_protocol_create_move_servo(self):
        """Test Protocol.create_move_servo()"""
        packet = Protocol.create_move_servo(servo_id=3, position=750)
        assert packet.cmd_type == CmdType.MOVE_SERVO

        servo_id, position = struct.unpack(">BH", packet.payload)
        assert servo_id == 3
        assert position == 750

    def test_protocol_set_gpio(self):
        """Test Protocol.create_set_gpio()"""
        packet = Protocol.create_set_gpio(5, True)
        assert packet.cmd_type == CmdType.SET_GPIO
        assert len(packet.payload) == 2

        pin, state = struct.unpack(">BB", packet.payload)
        assert pin == 5
        assert state

    def test_protocol_get_imu(self):
        """Test Protocol.create_get_imu()"""
        packet = Protocol.create_get_imu()
        assert packet.cmd_type == CmdType.GET_IMU
        assert len(packet.payload) == 0

    def test_protocol_start_imu_stream(self):
        """Test Protocol.create_get_imu()"""
        packet = Protocol.create_start_imu_stream(10)
        assert packet.cmd_type == CmdType.START_IMU_STREAM
        assert len(packet.payload) == 1

        rate = struct.unpack(">B", packet.payload)[0]
        assert rate == 10

    def test_protocol_stop_imu_stream(self):
        """Test Protocol.create_get_imu()"""
        packet = Protocol.create_stop_imu_stream()
        assert packet.cmd_type == CmdType.STOP_IMU_STREAM
        assert len(packet.payload) == 0

    def test_protocol_parse_imu_data(self):
        """Test Protocol.parse_imu_data()"""
        imu_data = struct.pack(">6f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        packet = Packet(cmd_type=CmdType.IMU_DATA, payload=imu_data)

        parsed = Protocol.parse_imu_data(packet)
        assert parsed is not None
        assert parsed["accel"]["x"] == 1.0
        assert parsed["accel"]["y"] == 2.0
        assert parsed["accel"]["z"] == 3.0
        assert parsed["gyro"]["x"] == 4.0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
