import asyncio
import logging
import signal

from client.botzo_client import BotzoClient

FORMAT = "%(asctime)s %(name) %(message)s"
logging.basicConfig(format=FORMAT)


async def main():
    client = BotzoClient("/dev/ttyUSB1", 112500)

    async def signal_handler(sig, frame):
        del sig, frame
        await client.disconnect()
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    if not await client.connect():
        exit(1)

    await client.set_gpio(12, True)

    await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
