import asyncio
import logging
import signal
import sys

from botzo_sdk import BotzoClient

FORMAT = "%(asctime)s %(name)s %(message)s"
logging.basicConfig(format=FORMAT)


async def main():
    client = BotzoClient("/dev/ttyUSB0", 112500)
    loop = asyncio.get_running_loop()

    # Use an asyncio Event to handle shutdown
    stop_event = asyncio.Event()

    def handle_signal():
        logging.info("Received SIGINT, disconnecting...")
        stop_event.set()

    # Register the signal handler
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, handle_signal)

    if not await client.connect():
        sys.exit(1)

    state = True
    count = 0
    await client.start_imu_streaming()
    try:
        while not stop_event.is_set():
            count += 1
            await client.set_gpio(26, state)
            state = not state
            await asyncio.sleep(1)
    finally:
        await client.disconnect()
        logging.info("Disconnected cleanly.")


if __name__ == "__main__":
    asyncio.run(main())
