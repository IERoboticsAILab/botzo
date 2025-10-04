import struct

from client.protocol import CmdType, Packet

from .mock_serial import MockSerial


class MockMCU:
    """Simulates MCU behavior for testing"""

    def __init__(self, mock_serial: MockSerial):
        self.serial = mock_serial
        self.servo_positions = {}
        self.gpio_states = {}
        self.imu_streaming = False
        self.imu_rate = 0

    def process_commands(self):
        """Process commands from write buffer and generate responses"""
        data = self.serial.get_written_data()
        self.serial.clear_write_buffer()

        if not data:
            return

        # Parse packet
        packet = Packet.from_bytes(data)
        if not packet:
            return

        # Handle commands
        if packet.cmd_type == CmdType.PING:
            self._respond_ack()

        elif packet.cmd_type == CmdType.MOVE_SERVO:
            if len(packet.payload) == 3:
                servo_id = packet.payload[0]
                position = struct.unpack(">H", packet.payload[1:3])[0]
                self.servo_positions[servo_id] = position

        elif packet.cmd_type == CmdType.SET_GPIO:
            if len(packet.payload) == 2:
                pin = packet.payload[0]
                state = packet.payload[1]
                self.gpio_states[pin] = bool(state)

        elif packet.cmd_type == CmdType.GET_IMU:
            self._send_imu_data()

        elif packet.cmd_type == CmdType.START_IMU_STREAM:
            if len(packet.payload) == 1:
                self.imu_streaming = True
                self.imu_rate = packet.payload[0]

        elif packet.cmd_type == CmdType.STOP_IMU_STREAM:
            self.imu_streaming = False

    def _respond_ack(self):
        """Send ACK response"""
        ack_packet = Packet(cmd_type=CmdType.ACK)
        self.serial.inject_data(ack_packet.to_bytes())

    def _send_imu_data(self):
        """Send IMU data response"""
        # Fake IMU data: 6 floats (accel_xyz, gyro_xyz)
        imu_data = struct.pack(">6f", 0.1, 0.2, 9.8, 0.01, 0.02, 0.03)
        imu_packet = Packet(cmd_type=CmdType.IMU_DATA, payload=imu_data)
        self.serial.inject_data(imu_packet.to_bytes())

    def simulate_imu_stream(self, count: int = 1):
        """Simulate streaming IMU data"""
        if self.imu_streaming:
            for _ in range(count):
                self._send_imu_data()
