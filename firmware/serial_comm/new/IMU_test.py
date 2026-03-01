#!/usr/bin/env python3
import smbus2
import time
import math

# MPU-9150 default I2C address
MPU_ADDR = 0x68  

# Register addresses
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43
TEMP_OUT_H   = 0x41

# Scale modifiers
ACCEL_SCALE_MODIFIER_2G = 16384.0
GYRO_SCALE_MODIFIER_250DEG = 131.0

# Open I2C bus
bus = smbus2.SMBus(1)

def read_word(adr):
    high = bus.read_byte_data(MPU_ADDR, adr)
    low  = bus.read_byte_data(MPU_ADDR, adr+1)
    val = (high << 8) + low
    if val >= 0x8000:   # signed conversion
        val = -((65535 - val) + 1)
    return val

def get_accel_data():
    x = read_word(ACCEL_XOUT_H) / ACCEL_SCALE_MODIFIER_2G
    y = read_word(ACCEL_XOUT_H+2) / ACCEL_SCALE_MODIFIER_2G
    z = read_word(ACCEL_XOUT_H+4) / ACCEL_SCALE_MODIFIER_2G
    return {"x": x, "y": y, "z": z}

def get_gyro_data():
    x = read_word(GYRO_XOUT_H) / GYRO_SCALE_MODIFIER_250DEG
    y = read_word(GYRO_XOUT_H+2) / GYRO_SCALE_MODIFIER_250DEG
    z = read_word(GYRO_XOUT_H+4) / GYRO_SCALE_MODIFIER_250DEG
    return {"x": x, "y": y, "z": z}

def get_temp():
    raw_temp = read_word(TEMP_OUT_H)
    actual_temp = (raw_temp / 340.0) + 36.53  # from datasheet
    return actual_temp

def main():
    # Wake up the MPU-9150 (it starts in sleep mode)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    print("Reading IMU data (Ctrl+C to stop)...")
    while True:
        accel = get_accel_data()
        gyro  = get_gyro_data()
        temp  = get_temp()

        print(f"Accel (g): x={accel['x']:.2f}, y={accel['y']:.2f}, z={accel['z']:.2f}")
        print(f"Gyro (°/s): x={gyro['x']:.2f}, y={gyro['y']:.2f}, z={gyro['z']:.2f}")
        print(f"Temp (°C): {temp:.2f}")
        print("-"*40)
        time.sleep(0.5)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
