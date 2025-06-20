import numpy as np
import time
import platform
import socket

# Only import smbus2 if we're on Linux (where I2C is available)
SIMULATION_MODE = platform.system() != 'Linux'
if not SIMULATION_MODE:
    import smbus2 as smbus

class IMU:
    def __init__(self, simulation_mode=SIMULATION_MODE):
        self.simulation_mode = simulation_mode
        
        if not simulation_mode:
            # Real hardware initialization
            self.bus = smbus.SMBus(1)
            self.initialize_mpu()
        else:
            print("Running in simulation mode - using synthetic IMU data")
            self.last_update = time.time()
            self.simulated_angles = [0, 0, 0]  # pitch, roll, yaw
        
    def initialize_mpu(self):
        if not self.simulation_mode:
            # MPU-6050 Constants
            MPU_ADDRESS = 0x68
            PWR_MGMT_1 = 0x6B
            self.bus.write_byte_data(MPU_ADDRESS, PWR_MGMT_1, 0)
    
    def read_raw_data(self, addr):
        if not self.simulation_mode:
            # Real hardware reading
            MPU_ADDRESS = 0x68
            high = self.bus.read_byte_data(MPU_ADDRESS, addr)
            low = self.bus.read_byte_data(MPU_ADDRESS, addr + 1)
            value = (high << 8) | low
            if value > 32768:
                value -= 65536
            return value
        return 0  # In simulation mode, return 0 for raw data
    
    def read_imu_data(self):
        if not self.simulation_mode:
            # Real hardware reading
            ACCEL_XOUT_H = 0x3B
            GYRO_XOUT_H = 0x43
            
            accel_x = self.read_raw_data(ACCEL_XOUT_H)
            accel_y = self.read_raw_data(ACCEL_XOUT_H + 2)
            accel_z = self.read_raw_data(ACCEL_XOUT_H + 4)

            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
            gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)

            # Convert to degrees/sec and g
            accel_scale = 16384.0
            gyro_scale = 131.0

            accel_x /= accel_scale
            accel_y /= accel_scale
            accel_z /= accel_scale

            gyro_x /= gyro_scale
            gyro_y /= gyro_scale
            gyro_z /= gyro_scale

            pitch = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2)) * (180 / np.pi)
            roll = np.arctan2(-accel_x, accel_z) * (180 / np.pi)
            yaw = gyro_z
            
            return pitch, roll, yaw
        else:
            # Simulation mode - generate synthetic motion
            current_time = time.time()
            dt = current_time - self.last_update
            
            # Create smooth oscillating motion
            t = current_time
            self.simulated_angles[0] = 20 * np.sin(0.5 * t)  # Pitch
            self.simulated_angles[1] = 15 * np.cos(0.3 * t)  # Roll
            self.simulated_angles[2] = 10 * np.sin(0.2 * t)  # Yaw
            
            self.last_update = current_time
            return tuple(self.simulated_angles)

def compute_rotation_matrix(pitch, roll, yaw):
    # Define rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(np.radians(roll)), -np.sin(np.radians(roll))],
                    [0, np.sin(np.radians(roll)), np.cos(np.radians(roll))]])
    R_y = np.array([[np.cos(np.radians(pitch)), 0, np.sin(np.radians(pitch))],
                    [0, 1, 0],
                    [-np.sin(np.radians(pitch)), 0, np.cos(np.radians(pitch))]])
    R_z = np.array([[np.cos(np.radians(yaw)), -np.sin(np.radians(yaw)), 0],
                    [np.sin(np.radians(yaw)), np.cos(np.radians(yaw)), 0],
                    [0, 0, 1]])
    return np.dot(R_z, np.dot(R_y, R_x))

def stabilize_robot(rotation_matrix):
    # Import stabilization code only when needed
    from stabilization_control import stabilize_robot as stabilize
    stabilize(rotation_matrix)

def send_data_via_socket(data):
    # Send data to a server or another device using a socket
    host = '192.168.1.100'  # Replace with your server IP
    port = 5000  # Replace with your desired port

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        message = ','.join(map(str, data))
        s.sendto(message.encode(), (host, port))

if __name__ == "__main__":
    print(f"Running in {'simulation' if SIMULATION_MODE else 'hardware'} mode")
    imu = IMU()
    while True:
        try:
            pitch, roll, yaw = imu.read_imu_data()
            rotation_matrix = compute_rotation_matrix(pitch, roll, yaw)
            stabilize_robot(rotation_matrix)
            print(f"Pitch: {pitch:.1f}°, Roll: {roll:.1f}°, Yaw: {yaw:.1f}°")
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nProgram stopped by user")
            break
