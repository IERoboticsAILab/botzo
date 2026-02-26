import smbus2
import time
import math
import threading

# MPU-9150/MPU-6050 compatible register map
MPU_ADDR = 0x68

PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# Scale modifiers
ACCEL_SCALE_MODIFIER_2G = 16384.0
GYRO_SCALE_MODIFIER_250DEG = 131.0

class IMUStabilizer:
    """
    Background reader for MPU IMU that computes fused roll/pitch using a complementary filter.
    Call start() to begin updates; call get_orientation() to retrieve latest (roll, pitch, yaw).
    """
    def __init__(self, i2c_bus: int = 1, address: int = MPU_ADDR, update_hz: float = 100.0, alpha: float = 0.98):
        self.address = address
        self.bus = smbus2.SMBus(i2c_bus)
        self.update_period_s = 1.0 / max(1.0, update_hz)
        self.alpha = alpha  # complementary filter blend factor
        self.alpha_max = 0.995  # when accel is unreliable, bias more towards gyro
        self.accel_trust_band_g = 0.15  # |norm-1g| within this band keeps alpha near base

        # State
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._yaw_deg = 0.0  # simple gyro integration (drifts); not used for stabilization by default
        self._lock = threading.Lock()

        self._running = False
        self._thread = None

        # Gyro bias calibration
        self._gyro_bias_x = 0.0
        self._gyro_bias_y = 0.0
        self._gyro_bias_z = 0.0

        # Latest gyro rates (deg/s), low-pass filtered
        self._gx_dps = 0.0
        self._gy_dps = 0.0
        self._gz_dps = 0.0
        self._rate_alpha = 0.3  # 0..1; higher = smoother rates

        # Orientation zero offsets
        self._roll_offset_deg = 0.0
        self._pitch_offset_deg = 0.0
        self._yaw_offset_deg = 0.0

        # Initialize sensor
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.05)

    def _read_word(self, reg: int) -> int:
        high = self.bus.read_byte_data(self.address, reg)
        low  = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def _read_accel(self):
        ax = self._read_word(ACCEL_XOUT_H) / ACCEL_SCALE_MODIFIER_2G
        ay = self._read_word(ACCEL_XOUT_H + 2) / ACCEL_SCALE_MODIFIER_2G
        az = self._read_word(ACCEL_XOUT_H + 4) / ACCEL_SCALE_MODIFIER_2G
        return ax, ay, az

    def _read_gyro(self):
        gx = self._read_word(GYRO_XOUT_H) / GYRO_SCALE_MODIFIER_250DEG
        gy = self._read_word(GYRO_XOUT_H + 2) / GYRO_SCALE_MODIFIER_250DEG
        gz = self._read_word(GYRO_XOUT_H + 4) / GYRO_SCALE_MODIFIER_250DEG
        return gx, gy, gz

    def _calibrate_gyro_bias(self, samples: int = 200):
        sum_x = sum_y = sum_z = 0.0
        for _ in range(samples):
            gx, gy, gz = self._read_gyro()
            sum_x += gx
            sum_y += gy
            sum_z += gz
            time.sleep(0.002)
        self._gyro_bias_x = sum_x / samples
        self._gyro_bias_y = sum_y / samples
        self._gyro_bias_z = sum_z / samples

    def reset_device(self):
        """Hardware reset the IMU and wake it up."""
        try:
            # Device reset
            self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            # Wake from sleep and select internal clock
            self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
            time.sleep(0.05)
        except Exception:
            # Non-fatal; proceed even if reset not supported
            pass

    def start(self):
        if self._running:
            return
        # Reset device so we start from a clean state
        self.reset_device()
        # Prime orientation from accelerometer to avoid initial jump
        ax, ay, az = self._read_accel()
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        with self._lock:
            self._roll_deg = roll_acc
            self._pitch_deg = pitch_acc
            self._yaw_deg = 0.0
            # Zero reference at startup so current pose is treated as level
            self._roll_offset_deg = self._roll_deg
            self._pitch_offset_deg = self._pitch_deg
            self._yaw_offset_deg = self._yaw_deg
        self._calibrate_gyro_bias()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _loop(self):
        prev_time = time.time()
        while self._running:
            start = time.time()

            # Read sensors
            ax, ay, az = self._read_accel()
            gx, gy, gz = self._read_gyro()

            # Remove gyro bias
            gx -= self._gyro_bias_x
            gy -= self._gyro_bias_y
            gz -= self._gyro_bias_z

            # Low-pass filter gyro rates for external use
            with self._lock:
                self._gx_dps = self._gx_dps + self._rate_alpha * (gx - self._gx_dps)
                self._gy_dps = self._gy_dps + self._rate_alpha * (gy - self._gy_dps)
                self._gz_dps = self._gz_dps + self._rate_alpha * (gz - self._gz_dps)

            now = time.time()
            dt = max(1e-3, now - prev_time)
            prev_time = now

            # Integrate gyro to estimate angles (deg)
            with self._lock:
                roll_gyro = self._roll_deg + gx * dt
                pitch_gyro = self._pitch_deg + gy * dt
                yaw_gyro = self._yaw_deg + gz * dt

            # Compute roll/pitch from accel (deg)
            roll_acc = math.degrees(math.atan2(ay, az))
            pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

            # Dynamic complementary filter: reduce accel influence when under strong linear acceleration
            accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
            accel_error = abs(accel_norm - 1.0)
            # Map error -> [0,1] within trust band
            t = max(0.0, min(1.0, accel_error / self.accel_trust_band_g))
            dyn_alpha = min(self.alpha_max, self.alpha + t * (self.alpha_max - self.alpha))

            roll = dyn_alpha * roll_gyro + (1.0 - dyn_alpha) * roll_acc
            pitch = dyn_alpha * pitch_gyro + (1.0 - dyn_alpha) * pitch_acc
            yaw = yaw_gyro  # no magnetometer; allow to drift if used

            with self._lock:
                self._roll_deg = roll
                self._pitch_deg = pitch
                self._yaw_deg = yaw

            # Sleep to maintain update rate
            elapsed = time.time() - start
            time_to_sleep = self.update_period_s - elapsed
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

    def get_orientation(self):
        with self._lock:
            return {
                "roll": self._roll_deg - self._roll_offset_deg,
                "pitch": self._pitch_deg - self._pitch_offset_deg,
                "yaw": self._yaw_deg - self._yaw_offset_deg
            }

    def zero_reference(self):
        """Set the current fused orientation as the new zero reference."""
        with self._lock:
            self._roll_offset_deg = self._roll_deg
            self._pitch_offset_deg = self._pitch_deg
            self._yaw_offset_deg = self._yaw_deg

    def get_rates(self):
        """Get low-pass filtered gyro rates in deg/s as a dict."""
        with self._lock:
            return {"roll_rate": self._gx_dps, "pitch_rate": self._gy_dps, "yaw_rate": self._gz_dps}


