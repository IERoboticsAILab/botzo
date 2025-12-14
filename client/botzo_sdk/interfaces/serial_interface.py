import asyncio
from typing import Callable, Dict, Optional

from serial import Serial

from botzo_sdk.logger import get_logger
from botzo_sdk.protocol import END_BYTE, START_BYTE, CMDType, Packet

from .interface import Interface


class SerialInterface(Interface):
    """Serial communication implementation of the Interface abstract class.

    This class provides serial port communication with an MCU, handling
    asynchronous reading, writing, and packet parsing. It uses a producer-consumer
    pattern with separate tasks for reading serial data and parsing packets.

    Attributes:
        _port (str): Serial port identifier (e.g., 'COM3' or '/dev/ttyUSB0').
        _baudrate (int): Communication speed in bits per second.
        _serial (Optional[Serial]): PySerial connection object.
        _callbacks (Dict[CmdType, Callable]): Registry mapping command types to callbacks.
        _response_queue (asyncio.Queue): Queue for packets awaiting processing.
        _buffer_queue (asyncio.Queue): Queue for raw bytes from serial port.
        _reader_task (Optional[asyncio.Task]): Background task reading from serial.
        _parser_task (Optional[asyncio.Task]): Background task parsing packets.
        _running (bool): Flag indicating if the interface is active.
    """

    def __init__(self, port: str, baudrate: int = 115200):
        """Initialize the serial interface.

        Args:
            port (str): Serial port identifier (e.g., 'COM3', '/dev/ttyUSB0').
            baudrate (int, optional): Communication speed in bits per second.
                Defaults to 115200.
        """
        super().__init__(get_logger("Interface"))
        self._port = port
        self._baudrate = baudrate
        self._serial: Optional[Serial] = None

        # Callback registry
        self._callbacks: Dict[CMDType, Callable[[Packet], None]] = {}

        self._response_queue: asyncio.Queue = asyncio.Queue()
        self._buffer_queue: asyncio.Queue = asyncio.Queue()

        self._reader_task: Optional[asyncio.Task] = None
        self._parser_task: Optional[asyncio.Task] = None
        self._running = False

    async def connect(self, low_latency: bool = False) -> bool:
        """Establish serial connection with the MCU.

        Opens the serial port, optionally enables low-latency mode, waits for
        MCU boot, and starts background tasks for reading and parsing data.

        Args:
            low_latency (bool, optional): If True, enables low-latency mode on
                the serial port for reduced transmission delays. Defaults to False.

        Returns:
            bool: True if connection was successful, False otherwise.

        Note:
            This method includes a 1-second delay to allow the MCU to boot.
            If already connected, returns True without reconnecting.
        """
        if self._serial:
            self._logger.info("Already connected")
            return True

        try:
            self._serial = Serial(self._port, self._baudrate, timeout=0.1)
            self._serial.set_low_latency_mode(low_latency)

            await asyncio.sleep(1)  # MCU boot time
            self._serial.reset_input_buffer()

            self._running = True
            self._reader_task = asyncio.create_task(self._reader())
            self._parser_task = asyncio.create_task(self._parser())

            self._logger.info(f"Connected to {self._port}:{self._baudrate}")
            return True

        except Exception as e:
            self._logger.error(f"Connection failed: {e}")
            return False

    async def disconnect(self):
        """Disconnect from the serial port and cleanup resources.

        Stops background tasks, cancels pending operations, and closes the
        serial port connection. Safe to call even if not connected.
        """
        self._running = False

        if self._reader_task:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except asyncio.CancelledError:
                pass

        if self._parser_task:
            self._parser_task.cancel()
            try:
                await self._parser_task
            except asyncio.CancelledError:
                pass

        if self._serial:
            self._serial.close()
            self._serial = None
            self._logger.info("Disconnected")

    def on(self, cmd_type: CMDType, callback: Callable[[Packet], None]):
        """Register a callback for incoming packets of a specific command type.

        When a packet with the specified command type is received, the registered
        callback will be invoked. The callback can be either synchronous or
        asynchronous (coroutine).

        Args:
            cmd_type (CmdType): The command type to listen for.
            callback (Callable[[Packet], None]): Function to call when a matching
                packet is received. Can be sync or async.

        Note:
            When a callback is registered, matching packets are consumed by the
            callback and NOT added to the response queue.
        """
        self._callbacks[cmd_type] = callback

    async def send(self, packet: Packet) -> bool:
        """Send a packet to the MCU via serial port.

        Args:
            packet (Packet): The packet to transmit.

        Returns:
            bool: True if all bytes were written successfully, False otherwise.

        Note:
            Returns False if not connected or if the serial port is not open.
        """
        if not self._serial or not self._serial.is_open:
            self._logger.error("Not connected")
            return False

        try:
            data = packet.serialize()
            written = self._serial.write(data)
            self._logger.debug(
                f"Sent: {packet.cmd_type.name} [{data.hex()}] ({written} bytes)"
            )
            return written == len(data)
        except Exception as e:
            self._logger.error(f"Send error: {e}")
            return False

    async def send_and_wait(
        self, packet: Packet, expected_response: CMDType, timeout: float = 1.0
    ) -> Optional[Packet]:
        """Send a packet and wait for a specific response from the MCU.

        Clears the response queue, sends the packet, and blocks until either
        the expected response arrives or the timeout expires.

        Args:
            packet (Packet): The packet to transmit.
            expected_response (CmdType): The command type of the expected response.
            timeout (float, optional): Maximum seconds to wait for response.
                Defaults to 1.0.

        Returns:
            Optional[Packet]: The response packet if received within timeout and
                matching expected type, None otherwise.

        Warning:
            This method clears the response queue before sending, which may
            discard previously received packets.
        """
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

    async def _reader(self):
        """Producer task: continuously read bytes from serial port.

        Runs in a background task, reading available bytes from the serial port
        and enqueueing them for processing by the parser. Implements a small
        sleep to prevent busy-waiting.

        Note:
            This is an internal method that runs as a background asyncio task.
            It continues until self._running is set to False.
        """
        while self._running:
            try:
                if self._serial and self._serial.in_waiting > 0:
                    # NOTE: self._serial.read() is synchronous. Sending this to
                    # a thread to prevent stuttering.
                    data = await asyncio.to_thread(
                        self._serial.read, self._serial.in_waiting
                    )
                    for b in data:
                        await self._buffer_queue.put(b)
                await asyncio.sleep(0.001)
            except Exception as e:
                self._logger.error(f"Reader error: {e}")
                await asyncio.sleep(0.1)

    async def _parser(self):
        """Consumer task: parse bytes into complete packets.

        Runs in a background task, consuming bytes from the buffer queue and
        assembling them into complete packets. Packets are delimited by START_BYTE
        and END_BYTE markers. Successfully parsed packets are dispatched to
        the handler.

        Note:
            This is an internal method that runs as a background asyncio task.
            It continues until self._running is set to False. Invalid packets
            are logged and discarded.
        """
        buffer = bytearray()

        while self._running:
            try:
                b = await self._buffer_queue.get()

                # Wait until start byte found
                if not buffer and b != START_BYTE:
                    continue

                buffer.append(b)

                # Check for complete packet
                if b == END_BYTE:
                    packet_data = bytes(buffer)
                    buffer.clear()

                    packet = Packet.deserialize(packet_data)

                    self._logger.debug(f"Detected valid packet: {packet_data.hex()}")

                    if packet:
                        await self._handle_packet(packet)
                    else:
                        self._logger.warning(f"Invalid packet: {packet_data.hex()}")

            except asyncio.CancelledError:
                break
            except Exception as e:
                self._logger.error(f"Parser error: {e}")
                buffer.clear()
                await asyncio.sleep(0.01)

    async def _handle_packet(self, packet: Packet):
        """Dispatch received packet to callback or response queue.

        If a callback is registered for the packet's command type, the callback
        is invoked (supports both sync and async callbacks). Otherwise, the
        packet is added to the response queue for send_and_wait.

        Args:
            packet (Packet): The received packet to handle.

        Warning:
            Packets handled by callbacks are NOT added to the response queue,
            which means they won't be available for send_and_wait calls.
        """
        self._logger.debug(
            f"Received: {packet.cmd_type.name} ({len(packet.payload)} bytes)"
        )

        # Call registered callback if exists
        if packet.cmd_type in self._callbacks:
            callback = self._callbacks[packet.cmd_type]
            if asyncio.iscoroutinefunction(callback):
                await callback(packet)
            else:
                callback(packet)
            # WARNING: Dispatched packet to a CB will is not queued. Queuing
            # would imply that send_and_wait could receive the wrong response.
            # At the same time, say a callback is registered for ACK, there would
            # be no response.
            # TODO: This needs to be better thought. Possible solution is ACK cannot be registered
            return

        await self._response_queue.put(packet)
