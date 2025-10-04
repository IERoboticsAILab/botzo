from client.interfaces import SerialInterface
from client.logger import get_logger
from client.protocol import Protocol
from client.protocol.types import CmdType

# TODO:(pedro) Client should maybe take a config


class BotzoClient:
    def __init__(self, port: str, baudrate: int):
        self._interface = SerialInterface(port, baudrate)
        self._protocol = Protocol()
        self._logger = get_logger("Client")

    async def connect(self) -> bool:
        if not await self._interface.connect():
            return False
        return await self.is_connected()

    async def disconnect(self):
        await self._interface.disconnect()

    async def is_connected(self) -> bool:
        packet = self._protocol.create_ping()
        resp = await self._interface.send_and_wait(packet, CmdType.ACK)
        return resp is not None

    async def set_gpio(self, pin: int, state: bool) -> bool:
        packet = self._protocol.create_set_gpio(pin, state)
        resp = await self._interface.send_and_wait(packet, CmdType.ACK)
        return resp is not None
