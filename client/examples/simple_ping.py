"""Simple Ping Example - Botzo SDK

This example demonstrates how to establish a serial connection with the Botzo
quadruped robot's microcontroller and verify connectivity using a ping command.

The example performs the following steps:
    1. Establishes serial connection to the MCU
    2. Sends a PING command
    3. Waits for ACK response (PONG)
    4. Cleanly disconnects

Hardware Requirements:
    - Botzo quadruped robot with MCU
    - USB-to-Serial adapter or direct USB connection
    - Proper serial drivers installed

Configuration:
    SERIAL_PORT: Serial port identifier
        - Linux/macOS: /dev/ttyUSB0, /dev/ttyACM0, /dev/cu.usbserial-*
        - Windows: COM3, COM4, etc.
    SERIAL_BAUD: Baud rate (must match MCU firmware configuration)
        - Default: 115200
        - Common: 9600, 115200, 921600

Expected Output:
    [Interface] INFO Connected to /dev/ttyUSB0:115200
    [example_simple_ping] INFO Sending PING to MCU
    [example_simple_ping] INFO Received PONG - Connection verified!
    [Interface] INFO Disconnected
    [example_simple_ping] INFO Disconnected successfully

Troubleshooting:
    - If connection fails: Check port name, ensure MCU is powered
    - If no PONG received: Verify baud rate matches firmware
    - Permission denied: Add user to dialout group (Linux) or check permissions

Usage:
    uv run examples/simple_ping.py

    # Or with Python directly
    python examples/simple_ping.py
"""

import asyncio
import sys
from argparse import ArgumentParser

from botzo_sdk import BotzoClient, get_logger

# Serial Configuration
SERIAL_PORT = "/dev/ttyUSB0"  # Update to match your system
SERIAL_BAUD = 115200  # Must match MCU firmware setting

# Setup logger for this example
logger = get_logger("example_simple_ping")


async def main():
    """Execute ping connectivity test.

    Establishes connection, sends ping command, waits for acknowledgment,
    and cleanly disconnects. Exits with error code 1 if connection fails.

    Returns:
        None

    Exits:
        0: Successful ping/pong exchange
        1: Connection failure
    """
    parser = ArgumentParser(
        prog="SimplePingExample",
        description="A simple example of how to ping the MCU connected to a serial port.",
    )
    parser.add_argument("--serial", "-s", default=SERIAL_PORT)
    parser.add_argument("--baud", "-b", default=SERIAL_BAUD)

    args = parser.parse_args()

    # Initialize client with serial parameters
    client = BotzoClient(args.serial, args.baud)

    # Attempt connection
    if not await client.connect():
        logger.error(f"Failed to connect to {SERIAL_PORT}")
        sys.exit(1)

    # Send ping command and wait for response
    logger.info("Sending PING to MCU")
    pong = await client.is_connected()

    if pong:
        logger.info("Received PONG - Connection verified!")
    else:
        logger.warning("No PONG received - Check MCU firmware")

    # Clean disconnect
    await client.disconnect()
    logger.info("Disconnected successfully")


if __name__ == "__main__":
    asyncio.run(main())
