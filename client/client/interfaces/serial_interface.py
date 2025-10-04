import asyncio
from typing import Callable, Dict, Optional

from serial import Serial

from client.logger import get_logger
from client.protocol import CmdType, Packet
from client.protocol.packet import END_BYTE

from .interface import Interface


class SerialInterface(Interface):
    def __init__(self, port: str, baudrate: int = 115200):
        super().__init__(get_logger("Interface"))
        self._port = port
        self._baudrate = baudrate
        self._serial: Optional[Serial] = None

        # Callback registry
        self._callbacks: Dict[CmdType, Callable] = {}

        # Response queue for request-response
        self._response_queue: asyncio.Queue = asyncio.Queue()

        # Background reader
        self._reader_task: Optional[asyncio.Task] = None
        self._running = False

    async def connect(self, low_latency: bool = False) -> bool:
        """Connect to serial port and start reader"""
        if self._serial:
            self._logger.info("Already connected")
            return True

        try:
            self._serial = Serial(self._port, self._baudrate, timeout=0.1)
            self._serial.set_low_latency_mode(low_latency)

            await asyncio.sleep(1)  # MCU boot time
            self._serial.reset_input_buffer()

            # Start background reader
            self._running = True
            self._reader_task = asyncio.create_task(self._read_loop())

            self._logger.info(f"Connected to {self._port}:{self._baudrate}")
            return True

        except Exception as e:
            self._logger.error(f"Connection failed: {e}")
            return False

    async def disconnect(self):
        """Disconnect and cleanup"""
        self._running = False

        if self._reader_task:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except asyncio.CancelledError:
                pass

        if self._serial:
            self._serial.close()
            self._serial = None
            self._logger.info("Disconnected")

    def on(self, cmd_type: CmdType, callback: Callable):
        """Register callback for packet type"""
        self._callbacks[cmd_type] = callback

    async def send(self, packet: Packet) -> bool:
        """Send a packet"""
        if not self._serial or not self._serial.is_open:
            self._logger.error("Not connected")
            return False

        try:
            data = packet.to_bytes()
            written = self._serial.write(data)
            self._logger.debug(f"Sent: {packet.cmd_type.name} ({written} bytes)")
            return written == len(data)
        except Exception as e:
            self._logger.error(f"Send error: {e}")
            return False

    async def send_and_wait(
        self, packet: Packet, expected_response: CmdType, timeout: float = 1.0
    ) -> Optional[Packet]:
        """Send packet and wait for response"""
        # Clear queue
        while not self._response_queue.empty():
            self._response_queue.get_nowait()

        if not await self.send(packet):
            return None

        try:
            response = await asyncio.wait_for(
                self._response_queue.get(), timeout=timeout
            )

            if response.cmd_type == expected_response:
                return response
            else:
                self._logger.warning(f"Unexpected response: {response.cmd_type.name}")
                return None

        except asyncio.TimeoutError:
            self._logger.warning("Response timeout")
            return None

    async def _read_loop(self):
        """Background task to read and dispatch packets"""
        buffer = bytearray()

        while self._running:
            try:
                if self._serial and self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    buffer.extend(data)

                    # Parse packets from buffer
                    while END_BYTE in buffer:
                        end_idx = buffer.index(END_BYTE) + 1
                        packet_data = bytes(buffer[:end_idx])
                        buffer = buffer[end_idx:]

                        packet = Packet.from_bytes(packet_data)
                        if packet:
                            await self._handle_packet(packet)
                        else:
                            self._logger.warning(f"Invalid packet: {packet_data.hex()}")

                await asyncio.sleep(0.001)

            except Exception as e:
                self._logger.error(f"Read loop error: {e}")
                await asyncio.sleep(0.1)

    async def _handle_packet(self, packet: Packet):
        """Dispatch packet to callback or queue"""
        self._logger.debug(f"Received: {packet.cmd_type.name}")

        # Call registered callback if exists
        if packet.cmd_type in self._callbacks:
            callback = self._callbacks[packet.cmd_type]
            if asyncio.iscoroutinefunction(callback):
                await callback(packet)
            else:
                callback(packet)

        # Also queue for request-response
        await self._response_queue.put(packet)
