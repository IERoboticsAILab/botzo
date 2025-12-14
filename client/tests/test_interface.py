import asyncio
from unittest.mock import patch

import pytest

from client.interfaces import SerialInterface
from client.protocol import START_BYTE, CMDType, Packet
from tests.mocks import MockMCU, MockSerial


class TestInterface:
    """Test interface with mock serial"""

    @pytest.mark.asyncio
    async def test_connect_and_disconnect(self):
        """Test connection lifecycle"""
        mock_serial = MockSerial()

        with patch(
            "client.interfaces.serial_interface.Serial", return_value=mock_serial
        ):
            interface = SerialInterface("/dev/ttyMOCK", 115200)

            assert await interface.connect()
            assert interface._serial is not None

            await interface.disconnect()
            assert interface._serial is None

    @pytest.mark.asyncio
    async def test_send_packet(self):
        """Test sending a packet"""
        mock_serial = MockSerial()

        with patch(
            "client.interfaces.serial_interface.Serial", return_value=mock_serial
        ):
            interface = SerialInterface("/dev/ttyMOCK", 115200)
            await interface.connect()

            packet = Packet(cmd_type=CMDType.PING)
            result = await interface.send(packet)

            assert result is True
            written_data = mock_serial.get_written_data()
            assert len(written_data) > 0
            assert written_data[0] == START_BYTE

            await interface.disconnect()

    @pytest.mark.asyncio
    async def test_send_and_wait(self):
        """Test request-response pattern"""
        mock_serial = MockSerial()
        mock_mcu = MockMCU(mock_serial)

        with patch(
            "client.interfaces.serial_interface.Serial", return_value=mock_serial
        ):
            interface = SerialInterface("/dev/ttyMOCK", 115200)
            await interface.connect()

            # Send ping
            packet = Packet(cmd_type=CMDType.PING)

            # Simulate MCU processing in background
            async def simulate_mcu():
                await asyncio.sleep(0.1)
                mock_mcu.process_commands()

            task = asyncio.create_task(simulate_mcu())

            # Wait for response
            response = await interface.send_and_wait(packet, CMDType.ACK, timeout=1.0)

            await task

            assert response is not None
            assert response.cmd_type == CMDType.ACK

            await interface.disconnect()

    @pytest.mark.asyncio
    async def test_callback_registration(self):
        """Test callback mechanism"""
        mock_serial = MockSerial()
        mock_mcu = MockMCU(mock_serial)

        callback_called = False
        received_packet = None

        def on_ack(packet):
            nonlocal callback_called, received_packet
            callback_called = True
            received_packet = packet

        with patch(
            "client.interfaces.serial_interface.Serial", return_value=mock_serial
        ):
            interface = SerialInterface("/dev/ttyMOCK", 115200)
            await interface.connect()

            # Register callback
            interface.on(CMDType.ACK, on_ack)

            # Send ping and simulate response
            packet = Packet(cmd_type=CMDType.PING)
            await interface.send(packet)

            mock_mcu.process_commands()

            # Wait for callback
            await asyncio.sleep(0.2)

            assert callback_called
            assert received_packet is not None
            assert received_packet.cmd_type == CMDType.ACK

            await interface.disconnect()
