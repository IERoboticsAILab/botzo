import logging
from abc import ABC, abstractmethod
from typing import Callable, Optional

from client.protocol.packet import Packet
from client.protocol.types import CmdType


class Interface(ABC):
    def __init__(self, logger: logging.Logger):
        self._logger = logger

    @abstractmethod
    def connect(self):
        raise NotImplementedError

    @abstractmethod
    def disconnect(self):
        raise NotImplementedError

    @abstractmethod
    async def send(self, packet: Packet) -> bool:
        raise NotImplementedError

    @abstractmethod
    async def send_and_wait(
        self, packet: Packet, expected_response: CmdType, timeout: float = 1.0
    ) -> Optional[Packet]:
        raise NotImplementedError

    @abstractmethod
    def on(self, cmd_type: CmdType, callback: Callable):
        raise NotImplementedError
