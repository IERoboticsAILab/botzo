#!/usr/bin/env python3

import tkinter as tk
import numpy as np
import serial
import time

# =====================================================
# Serial
# =====================================================

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 500000

# =====================================================
# Servo calibration coefficients
# (same values from your original node)
# =====================================================

COEFFS = [
    (0,      7.378, 616.0),      # FR Shoulder
    (0,      7.682, 578.142),    # FR Femur
    (0,      7.314, 619.028),    # FR Tibia

    (0,      7.649, 638.486),    # FL Shoulder
    (0,      7.603, 625.428),    # FL Femur
    (0.001,  7.234, 550.0),      # FL Tibia

    (0,      7.514, 626.428),    # BR Shoulder
    (0.001,  7.364, 548.742),    # BR Femur
    (0.001,  7.137, 559.2),      # BR Tibia

    (0.001,  7.673, 648.857),    # BL Shoulder
    (0,      7.704, 628.142),    # BL Femur
    (-0.001, 7.765, 634.285),    # BL Tibia
]

SERVO_NAMES = [
    "FR Shoulder",
    "FR Femur",
    "FR Tibia",

    "FL Shoulder",
    "FL Femur",
    "FL Tibia",

    "BR Shoulder",
    "BR Femur",
    "BR Tibia",

    "BL Shoulder",
    "BL Femur",
    "BL Tibia",
]

# =====================================================
# Initial angles
# =====================================================
#msg = f"{fr[0]},{fr[1]},{fr[2]},{fl[0]},{fl[1]},{fl[2]},{br[0]},{br[1]},{br[2]},{bl[0]},{bl[1]},{bl[2]}\n"
angles_deg = [90.0, 90.0, 0.0, 90.0, 90.0, 0.0, 90.0, 90.0, 0.0, 90.0, 90.0, 0.0]

labels = []
ser = None


# =====================================================
# Conversion functions
# =====================================================

def deg_to_pwm(angle_deg, coeffs):
    a, b, c = coeffs
    return int(round(a * angle_deg**2 + b * angle_deg + c))


def send_to_arduino():

    pwm = []

    for angle, coeff in zip(angles_deg, COEFFS):
        pwm.append(deg_to_pwm(angle, coeff))

    message = ",".join(str(x) for x in pwm) + "\n"

    print("PWM:", pwm)

    if ser is not None and ser.is_open:
        ser.write(message.encode())


# =====================================================
# Slider callback
# =====================================================

def slider_changed(index, value):

    angles_deg[index] = float(value)

    rad = np.deg2rad(angles_deg[index])

    labels[index]["text"] = f"{angles_deg[index]:6.1f}°   {rad:6.3f} rad"

    send_to_arduino()


# =====================================================
# Main
# =====================================================

def main():

    global ser

    print("Connecting to Arduino...")

    ser = serial.Serial(
        SERIAL_PORT,
        BAUDRATE,
        timeout=0.1
    )

    time.sleep(1)

    print("Connected!")

    root = tk.Tk()
    root.title("Move Real Robot Servos")

    for i in range(12):

        frame = tk.Frame(root)
        frame.pack(fill="x", padx=5, pady=2)

        tk.Label(
            frame,
            text=SERVO_NAMES[i],
            width=14,
            anchor="w"
        ).pack(side="left")

        slider = tk.Scale(
            frame,
            from_=0,
            to=180,
            orient="horizontal",
            length=350,
            command=lambda value, idx=i: slider_changed(idx, value)
        )

        slider.set(90)
        slider.pack(side="left")

        label = tk.Label(
            frame,
            text="90.0°   1.571 rad",
            width=18
        )

        label.pack(side="left")

        labels.append(label)

    send_to_arduino()

    root.mainloop()

    if ser.is_open:
        ser.close()


if __name__ == "__main__":
    main()