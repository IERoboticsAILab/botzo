from enum import Enum


class CMDType(Enum):
    """Command types supported by the MCU communication protocol."""

    PING = 0x01
    """Request connectivity check from MCU. Expects ACK response."""

    ACK = 0x02
    """Positive acknowledgment. Sent by MCU to confirm successful command execution."""

    NACK = 0x03
    """Negative acknowledgment. Sent by MCU when command fails or is invalid."""

    MOVE_SERVO = 0x10
    """Command single servo to move to target position. Payload contains servo ID and position."""

    MOVE_LEG = 0x11
    """Command entire leg assembly to move. Payload contains leg ID and joint angles."""

    SET_GPIO = 0x30
    """Set GPIO pin state. Payload contains pin number and state (high/low)."""

    GET_GPIO = 0x31
    """Request current GPIO pin state. Response contains pin number and state."""

    GET_IMU = 0x40
    """Request single IMU reading. Response is IMU_DATA packet."""

    START_IMU_STREAM = 0x41
    """Begin continuous IMU data streaming at configured rate."""

    STOP_IMU_STREAM = 0x42
    """Stop continuous IMU data streaming."""

    IMU_DATA = 0x43
    """IMU data packet sent by MCU. Contains accelerometer, gyroscope, and magnetometer readings."""
