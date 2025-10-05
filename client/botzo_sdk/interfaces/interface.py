import logging
from abc import ABC, abstractmethod
from typing import Callable, Optional

from botzo_sdk.protocol.packet import Packet
from botzo_sdk.protocol.types import CMDType


class Interface(ABC):
    """Abstract base class for MCU communication interfaces.

    This class defines the standard interface for communicating with a
    microcontroller unit (MCU) through various protocols such as serial,
    UDP, TCP, or others.

    All concrete implementations must provide their own connection handling,
    packet transmission, and event callback mechanisms.

    Attributes:
        _logger (logging.Logger): Logger instance for interface operations.
    """

    def __init__(self, logger: logging.Logger):
        """Initialize the interface with a logger.

        Args:
            logger (logging.Logger): Logger instance for recording interface
                operations and debugging information.
        """
        self._logger = logger

    @abstractmethod
    def connect(self):
        """Establish connection with the target MCU.

        This method must be called before any communication operations
        (send, send_and_wait, etc.) can be performed.

        Raises:
            NotImplementedError: This method must be implemented by subclasses.
        """
        raise NotImplementedError

    @abstractmethod
    def disconnect(self):
        """Terminate the connection with the MCU.

        Releases any resources held by the interface and closes the
        communication channel.

        Raises:
            NotImplementedError: This method must be implemented by subclasses.
        """
        raise NotImplementedError

    @abstractmethod
    async def send(self, packet: Packet) -> bool:
        """Send a packet to the MCU asynchronously.

        Args:
            packet (Packet): The packet to transmit to the MCU.

        Returns:
            bool: True if the packet was sent successfully, False otherwise.

        Raises:
            NotImplementedError: This method must be implemented by subclasses.
        """
        raise NotImplementedError

    @abstractmethod
    async def send_and_wait(
        self, packet: Packet, expected_response: CMDType, timeout: float = 1.0
    ) -> Optional[Packet]:
        """Send a packet and wait for a specific response from the MCU.

        This method transmits a packet and blocks until either the expected
        response is received or the timeout period expires.

        Args:
            packet (Packet): The packet to transmit to the MCU.
            expected_response (CmdType): The command type of the expected
                response packet (e.g., acknowledgment).
            timeout (float, optional): Maximum time in seconds to wait for
                the response. Defaults to 1.0.

        Returns:
            Optional[Packet]: The received response packet if it arrives
                within the timeout period, None otherwise.

        Raises:
            NotImplementedError: This method must be implemented by subclasses.
        """
        raise NotImplementedError

    @abstractmethod
    def on(self, cmd_type: CMDType, callback: Callable[[Packet], None]):
        """Register a callback for incoming packets of a specific command type.

        Sets up an event handler that triggers when a packet with the
        specified command type is received from the MCU.

        Args:
            cmd_type (CmdType): The command type to listen for.
            callback (Callable): The function to call when a matching packet
                is received. The callback should accept the received packet
                as its argument.

        Raises:
            NotImplementedError: This method must be implemented by subclasses.
        """
        raise NotImplementedError
