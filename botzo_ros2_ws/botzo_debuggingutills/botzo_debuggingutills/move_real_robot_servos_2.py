#!/usr/bin/env python3
"""
Move Real Robot Servos — non-freezing version.

Same initial positions / calibration as the original script.
All serial I/O (connecting + writing) runs on a background thread,
so dragging sliders never blocks the Tkinter event loop, and the
window opens instantly even if the Arduino isn't connected yet.
"""

import tkinter as tk
import numpy as np
import serial
import threading
import queue
import time

# =====================================================
# Serial
# =====================================================
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 500000

# =====================================================
# Servo calibration coefficients (unchanged)
# =====================================================
COEFFS = [
    (0, 7.378, 616.0),        # FR Shoulder
    (0, 7.682, 578.142),      # FR Femur
    (0, 7.314, 619.028),      # FR Tibia
    (0, 7.649, 638.486),      # FL Shoulder
    (0, 7.603, 625.428),      # FL Femur
    (0.001, 7.234, 550.0),    # FL Tibia
    (0, 7.514, 626.428),      # BR Shoulder
    (0.001, 7.364, 548.742),  # BR Femur
    (0.001, 7.137, 559.2),    # BR Tibia
    (0.001, 7.673, 648.857),  # BL Shoulder
    (0, 7.704, 628.142),      # BL Femur
    (-0.001, 7.765, 634.285), # BL Tibia
]

SERVO_NAMES = [
    "FR Shoulder", "FR Femur", "FR Tibia",
    "FL Shoulder", "FL Femur", "FL Tibia",
    "BR Shoulder", "BR Femur", "BR Tibia",
    "BL Shoulder", "BL Femur", "BL Tibia",
]

# =====================================================
# Initial angles (unchanged)
# =====================================================
angles_deg = [90.0, 90.0, 180.0, 90.0, 90.0, 0.0,
              90.0, 90.0, 180.0, 90.0, 90.0, 0.0]

labels = []


# =====================================================
# Conversion
# =====================================================
def deg_to_pwm(angle_deg, coeffs):
    a, b, c = coeffs
    return int(round(a * angle_deg ** 2 + b * angle_deg + c))


# =====================================================
# Background serial worker
#
# - Connects on its own thread (the original's time.sleep(1) used
#   to freeze the whole GUI at startup; now it just delays the
#   "Connected" status message).
# - Writes go through a 1-slot queue: every new send() overwrites
#   whatever hasn't been written yet, so a fast slider drag never
#   queues up a backlog of stale writes -> no lag, no freeze.
# - Any serial error (port missing, write failure, etc.) is caught
#   here and reported via status text instead of crashing the app.
# =====================================================
class SerialWorker:
    def __init__(self, port, baud, status_cb):
        self.port = port
        self.baud = baud
        self.status_cb = status_cb
        self.ser = None
        self.q = queue.Queue(maxsize=1)
        self.connected = False
        self._stop = False

        threading.Thread(target=self._connect, daemon=True).start()
        threading.Thread(target=self._writer_loop, daemon=True).start()

    def _connect(self):
        self.status_cb(f"Connecting to {self.port} ...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(1)  # board reset delay — off the GUI thread now
            self.connected = True
            self.status_cb(f"Connected to {self.port}")
        except Exception as e:
            self.connected = False
            self.status_cb(f"No serial connection ({e}) — sliders still work, nothing will move")

    def send(self, pwm_values):
        """Queue the latest pwm values, dropping any stale ones."""
        message = ",".join(str(x) for x in pwm_values) + "\n"
        try:
            while True:
                self.q.get_nowait()
        except queue.Empty:
            pass
        try:
            self.q.put_nowait(message)
        except queue.Full:
            pass

    def _writer_loop(self):
        while not self._stop:
            try:
                message = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if self.ser is not None and self.connected:
                try:
                    self.ser.write(message.encode())
                except Exception as e:
                    self.connected = False
                    self.status_cb(f"Write error: {e}")

    def close(self):
        self._stop = True
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass


# =====================================================
# GUI
# =====================================================
class App:
    def __init__(self, root):
        self.root = root
        root.title("Move Real Robot Servos")

        self.status_var = tk.StringVar(value="Starting...")
        tk.Label(root, textvariable=self.status_var, fg="blue", anchor="w").pack(
            fill="x", padx=5, pady=(5, 0)
        )

        for i in range(12):
            frame = tk.Frame(root)
            frame.pack(fill="x", padx=5, pady=2)

            tk.Label(frame, text=SERVO_NAMES[i], width=14, anchor="w").pack(side="left")

            slider = tk.Scale(
                frame, from_=0, to=180, orient="horizontal", length=350,
                command=lambda value, idx=i: self.slider_changed(idx, value),
            )
            slider.set(angles_deg[i])
            slider.pack(side="left")

            label = tk.Label(
                frame,
                text=f"{angles_deg[i]:.1f}\u00b0 {np.deg2rad(angles_deg[i]):.3f} rad",
                width=18,
            )
            label.pack(side="left")
            labels.append(label)

        # Background worker — connecting happens off-thread, so this
        # returns immediately and the window is responsive right away.
        self.worker = SerialWorker(SERIAL_PORT, BAUDRATE, self.set_status_threadsafe)

        # Push the initial pose once the GUI is up.
        self.send_pwm()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def set_status_threadsafe(self, text):
        # status_cb may be called from a background thread; route the
        # actual widget update back onto the Tk main loop.
        self.root.after(0, lambda: self.status_var.set(text))

    def slider_changed(self, index, value):
        angles_deg[index] = float(value)
        rad = np.deg2rad(angles_deg[index])
        labels[index]["text"] = f"{angles_deg[index]:6.1f}\u00b0 {rad:6.3f} rad"
        self.send_pwm()

    def send_pwm(self):
        pwm = [deg_to_pwm(a, c) for a, c in zip(angles_deg, COEFFS)]
        print("PWM:", pwm)
        self.worker.send(pwm)

    def on_close(self):
        self.worker.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()