from io import BytesIO


class MockSerial:
    """Mock serial port for testing"""

    def __init__(self):
        self.read_buffer = BytesIO()
        self.write_buffer = BytesIO()
        self.is_open = True
        self.in_waiting = 0

    def write(self, data: bytes) -> int:
        """Simulate writing to serial"""
        self.write_buffer.write(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        """Simulate reading from serial"""
        data = self.read_buffer.read(size)
        self.in_waiting = max(0, self.in_waiting - len(data))
        return data

    def reset_input_buffer(self):
        """Clear input buffer"""
        self.read_buffer = BytesIO()
        self.in_waiting = 0

    def close(self):
        """Close serial port"""
        self.is_open = False

    def set_low_latency_mode(self, enabled: bool):
        """Mock method"""
        pass

    def inject_data(self, data: bytes):
        """Helper to inject data for reading"""
        current_pos = self.read_buffer.tell()
        self.read_buffer.seek(0, 2)  # End of buffer
        self.read_buffer.write(data)
        self.read_buffer.seek(current_pos)  # Restore position
        self.in_waiting += len(data)

    def get_written_data(self) -> bytes:
        """Get data written to serial"""
        pos = self.write_buffer.tell()
        self.write_buffer.seek(0)
        data = self.write_buffer.read()
        self.write_buffer.seek(pos)
        return data

    def clear_write_buffer(self):
        """Clear write buffer"""
        self.write_buffer = BytesIO()
