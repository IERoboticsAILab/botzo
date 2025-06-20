# Quadruped Robot Stabilization System 🤖

[![Python](https://img.shields.io/badge/python-3.12+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%20%7C%20Arduino-orange.svg)](https://www.raspberrypi.org/)

A real-time stabilization system for Botzo, a quadruped robot using IMU-based orientation detection and inverse kinematics for leg position adjustment. Each leg features three degrees of freedom (3-DOF) for precise control and stabilization.

<div align="center">
  <img src="https://github.com/botzo-team/rotation_matrices_v1/blob/main/docs/images/robot_demo.gif" alt="Robot Demo" width="600"/>
</div>

## 🌟 Features

- Real-time IMU (MPU-6050) orientation detection
- Enhanced 3D visualization with:
  - Accurate leg segment visualization
  - IMU orientation display with coordinate axes
  - Real-time performance monitoring
  - Interactive view controls
- Advanced stabilization with:
  - Configurable pitch/roll/yaw compensation
  - Rate-limited servo movements
  - Safety constraints and movement limits
  - Smooth motion transitions
- Full 3-DOF control per leg:
  - Shoulder servo: Horizontal rotation
  - Femur servo: Upper leg lift
  - Tibia servo: Lower leg extension
- Simulation mode for testing without hardware
- Arduino-based servo control system
- Support for DS3225 servos (270° range)

## 🛠️ Hardware Specifications

- **Robot Dimensions**:
  - Body: 274mm × 140mm × 80mm
  - Leg Segments:
    - Coxa: 31mm (A)
    - Femur: 95mm (E) / 91mm (E')
    - Tibia: 98mm (F)
  - Focus Point Distance: 28mm

- **Hardware Requirements**:
  - Raspberry Pi 4 Model B
  - Arduino Mega 2560 Rev 3
  - MPU-6050 IMU Sensor
  - 12× DS3225 Servo Motors (three per leg):
    - 4× Shoulder servos
    - 4× Femur servos
    - 4× Tibia servos

## 📡 Hardware Setup

### MPU-6050 Connection to Raspberry Pi

1. **Enable I2C on Raspberry Pi:**
   ```bash
   sudo raspi-config
   # Navigate to: Interface Options -> I2C -> Enable
   sudo reboot
   ```

2. **Install I2C Tools:**
   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-smbus i2c-tools
   ```

3. **Connect MPU-6050 to Raspberry Pi:**
   | MPU-6050 Pin | Raspberry Pi Pin | Description |
   |--------------|------------------|-------------|
   | VCC         | Pin 1 (3.3V)     | Power       |
   | GND         | Pin 6 (Ground)   | Ground      |
   | SCL         | Pin 5 (GPIO 3)   | Clock       |
   | SDA         | Pin 3 (GPIO 2)   | Data        |

4. **Verify Connection:**
   ```bash
   sudo i2cdetect -y 1
   ```
   You should see "68" in the output grid (MPU-6050's I2C address).

### Arduino Setup

1. **Connect Arduino Mega to Raspberry Pi via USB**
2. **Check Arduino Connection:**
   ```bash
   ls /dev/ttyACM*
   ```
   Should show `/dev/ttyACM0`

3. **Set Permissions:**
   ```bash
   sudo usermod -a -G dialout $USER
   ```

4. **Servo Wiring:**
   ```
   Leg Servo Configuration:
   Front Left (FL):
   - Pin 5: Shoulder servo
   - Pin 6: Femur servo
   - Pin 7: Tibia servo
   
   Front Right (FR):
   - Pin 2: Shoulder servo
   - Pin 3: Femur servo
   - Pin 4: Tibia servo
   
   Back Left (BL):
   - Pin 11: Shoulder servo
   - Pin 12: Femur servo
   - Pin 13: Tibia servo
   
   Back Right (BR):
   - Pin 8: Shoulder servo
   - Pin 9: Femur servo
   - Pin 10: Tibia servo
   ```

## 🚀 Software Setup

1. **Clone Repository and Setup Environment:**
   ```bash
   git clone https://github.com/your-username/quadruped-stabilization.git
   cd quadruped-stabilization
   python3 -m venv venv
   source venv/bin/activate
   ```

2. **Install Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Upload Arduino Code:**
   - Open `quadruped_servo_controller/quadruped_servo_controller.ino` in Arduino IDE
   - Select "Arduino Mega 2560" board
   - Choose correct port
   - Click Upload

4. **Run the System:**
   ```bash
   # For hardware mode (with actual robot)
   python imu_stabilization.py

   # For simulation mode
   python test_imu_stabilization.py
   ```

## 🎮 Visualization Features

The 3D visualization provides:
- Real-time robot pose with accurate dimensions
- Color-coded legs with segment indicators:
  - 🔴 Front Left (3 joints)
  - 🔵 Front Right (3 joints)
  - 💛 Back Left (3 joints)
  - 🟣 Back Right (3 joints)
- Joint visualization:
  - Square markers: Servo positions
  - Circle markers: Joint connections
  - Star markers: Foot contact points
- IMU orientation display:
  - Red: X-axis
  - Green: Y-axis
  - Blue: Z-axis
- Performance metrics:
  - Update rate
  - Servo positions
  - Stabilization status
- Interactive controls:
  - Left-click & drag: Rotate view
  - Right-click & drag: Zoom
  - Middle-click & drag: Pan

## ⚙️ Stabilization Parameters

Adjustable parameters in `stabilization_control.py`:
```python
# Movement Ranges (cm)
X_RANGE = (-5, 5)    # Forward/backward
Y_RANGE = (-3, 3)    # Left/right
Z_RANGE = (-18, -12) # Height

# Compensation Factors
PITCH_COMPENSATION = 1.2  # Increased pitch response
ROLL_COMPENSATION = 1.0   # Normal roll response
YAW_COMPENSATION = 0.8    # Reduced yaw response

# Safety Limits
MAX_ANGLE_CHANGE = 30  # Maximum degrees per update
SERVO_SPEED = 0.5      # Seconds per 60 degrees

# Servo Angle Ranges (degrees)
SHOULDER_RANGE = (0, 270)   # Horizontal rotation
FEMUR_RANGE = (45, 225)     # Vertical lift
TIBIA_RANGE = (90, 270)     # Extension
```

## 🔍 Troubleshooting

| Issue | Solution |
|-------|----------|
| Visualization lag | - Reduce update rate<br>- Close other applications<br>- Check CPU usage |
| Servo jitter | - Verify power supply<br>- Check angle limits<br>- Adjust MAX_ANGLE_CHANGE |
| IMU drift | - Calibrate IMU<br>- Check update rate<br>- Verify connections |
| Unstable movement | - Adjust compensation factors<br>- Check movement ranges<br>- Verify servo speed |
| Joint misalignment | - Verify servo zero positions<br>- Check joint angle limits<br>- Calibrate servos |
| Leg coordination | - Check servo timing<br>- Verify IK calculations<br>- Adjust movement speed |

## 🛟 Debug Commands

```bash
# Monitor IMU data and performance
python imu_stabilization.py --debug

# Test visualization only
python test_imu_stabilization.py

# Check servo positions
python
>>> import serial
>>> arduino = serial.Serial('/dev/ttyACM0', 115200)
>>> # Test shoulder servo
>>> arduino.write(b"S,0,0,135;")  # Leg 0, servo 0 (shoulder), 135°
>>> # Test femur servo
>>> arduino.write(b"S,0,1,90;")   # Leg 0, servo 1 (femur), 90°
>>> # Test tibia servo
>>> arduino.write(b"S,0,2,180;")  # Leg 0, servo 2 (tibia), 180°

# System logs
journalctl -f | grep stabilization
```

## 📝 Development Notes

- System automatically detects hardware/simulation mode
- Real-time performance monitoring available
- Servo commands format: "S,leg,servo,angle;" (e.g., "S,0,1,90;")
  - leg: 0-3 (FL, FR, BL, BR)
  - servo: 0-2 (shoulder, femur, tibia)
  - angle: 0-270 degrees
- All dimensions in centimeters, angles in degrees
- Safety constraints prevent dangerous movements
- Smooth transitions between positions
- IK calculations handle all three joints per leg

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

- Create an issue for bugs
- Start a discussion for questions
- Pull requests are welcome!