"""Client Library for Botzo Quadruped Robot

This is a client library targetted to be used by Botzo. However, this library
can be expanded to be used by other robots that use serial communication or
any other interface that is supported.

For now, only Serial communication is supported.
"""

from botzo_sdk.clients import BotzoClient
from botzo_sdk.interfaces import SerialInterface
from botzo_sdk.logger import get_logger, logger_manager
from botzo_sdk.protocol import CMDType, Packet, Protocol

# Version
__version__ = "0.1.0"

# Configure default logging (can be overridden by users)
import logging

logger_manager.configure(level=logging.INFO, use_colors=True)

__all__ = [
    # Core classes
    "BotzoClient",
    "get_logger",
    "SerialInterface",
    "Protocol",
    "Packet",
    "CMDType",
    "__version__",
]
