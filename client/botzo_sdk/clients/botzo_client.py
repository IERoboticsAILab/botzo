from botzo_sdk.interfaces import SerialInterface
from botzo_sdk.logger import get_logger
from botzo_sdk.protocol import Protocol
from botzo_sdk.protocol.types import CMDType

# TODO: What is still missing
# - [ ] Make the client take a configuration
# - [ ] Add reconnect attempts on connect method
# - [ ] Add possibility to register callbacks/events
# - [ ] Add new methods for new functionality as needed


class BotzoClient:
    def __init__(self, port: str, baudrate: int):
        self._interface = SerialInterface(port, baudrate)
        self._protocol = Protocol()
        self._logger = get_logger("Client")

    def print_data(self, packet):
        print(packet)
        print(self._protocol.parse_imu_data(packet))

    async def connect(self) -> bool:
        if not await self._interface.connect():
            return False

        return await self.is_connected()

    async def disconnect(self):
        await self.stop_imu_streaming()
        await self._interface.disconnect()

    async def is_connected(self) -> bool:
        packet = self._protocol.create_ping()
        resp = await self._interface.send_and_wait(packet, CMDType.ACK)
        return resp is not None

    async def set_gpio(self, pin: int, state: bool) -> bool:
        packet = self._protocol.create_set_gpio(pin, state)
        resp = await self._interface.send_and_wait(packet, CMDType.ACK)
        return resp is not None

    async def start_imu_streaming(self) -> bool:
        packet = self._protocol.create_start_imu_stream(100)
        resp = await self._interface.send_and_wait(packet, CMDType.ACK)
        return resp is not None

    async def stop_imu_streaming(self) -> bool:
        packet = self._protocol.create_stop_imu_stream()
        resp = await self._interface.send_and_wait(packet, CMDType.ACK)
        return resp is not None
