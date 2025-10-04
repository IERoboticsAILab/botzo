from enum import Enum


class CmdType(Enum):
    # System commands
    PING = 0x01
    ACK = 0x02
    NACK = 0x03

    # Servo commands
    MOVE_SERVO = 0x10
    MOVE_LEG = 0x11

    # GPIO commands
    SET_GPIO = 0x30
    GET_GPIO = 0x31

    # Sensor commands
    GET_IMU = 0x40
    START_IMU_STREAM = 0x41
    STOP_IMU_STREAM = 0x42
    IMU_DATA = 0x43
