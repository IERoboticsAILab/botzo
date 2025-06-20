<div align="center">
<h1>BOTZO 🐾</h1>

**`The good boy quadruped robot :)`**

<p align="center">
    <a href="https://www.instagram.com/botzo.ie/" target="_blank" rel="noopener noreferrer">
        <img alt="Instagram" src="https://img.shields.io/badge/Instagram-%232C3454.svg?style=for-the-badge&logo=Instagram&logoColor=white" />
    </a>
    <a href="" target="_blank" rel="noopener noreferrer">
        <img alt="LinkedIn" src="https://img.shields.io/badge/Youtube-%232C3454.svg?style=for-the-badge&logo=Youtube&logoColor=white" />
    </a>
    <a href="mailto:botzoteam@gmail.com">
        <img alt="Gmail" src="https://img.shields.io/badge/Gmail-2c3454?style=for-the-badge&logo=gmail&logoColor=white" />
    </a>
    <a href="">
        <img alt="Views" src="https://komarev.com/ghpvc/?username=botzo&color=blue&style=for-the-badge&abbreviated=true" />
    </a>

</p>

</div>

more [here](https://github.com/IERoboticsAILab/botzo)

# Quadruped Robot Stabilization System 🤖

[![Python](https://img.shields.io/badge/python-3.12+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%20%7C%20Arduino-orange.svg)](https://www.raspberrypi.org/)

A real-time stabilization system for quadruped robots using IMU-based orientation detection and inverse kinematics for leg position adjustment.

<div align="center">
  <img src="https://github.com/botzo-team/rotation_matrices_v1/blob/main/docs/images/robot_demo.gif" alt="Robot Demo" width="600"/>
</div>

## 🌟 Features

- Real-time IMU (MPU-6050) orientation detection
- 3D visualization of robot pose and leg positions
- Inverse kinematics for precise leg adjustments
- Simulation mode for testing without hardware
- Arduino-based servo control system
- Support for DS3225 servos (270° range)

## 🛠️ Hardware Requirements

- Raspberry Pi 4 Model B
- Arduino Mega 2560 Rev 3
- MPU-6050 IMU Sensor
- 12× DS3225 Servo Motors
- Quadruped Robot Frame
  - Body: 274mm × 140mm
  - Leg Length: 193mm
  - Standing Height: 125mm

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

## 🚀 Software Setup

1. **Clone Repository and Setup Environment:**
   ```bash
   # Create project directory
   mkdir ~/botzo_stabilization
   cd ~/botzo_stabilization

   # Create and activate virtual environment
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
   python imu_stabilization.py
   ```

## 🎮 Controls & Visualization

The 3D visualization provides:
- Real-time robot pose display
- Color-coded legs:
  - 🔴 Front Left
  - 🟢 Front Right
  - 🔵 Back Left
  - 🟡 Back Right
- Interactive view controls:
  - Left-click & drag: Rotate
  - Right-click & drag: Zoom
  - Middle-click & drag: Pan

## 📐 Servo Configuration

```python
Servo Layout:
FL (Front Left):  Pins 5, 6, 7   [Shoulder, Knee, Ankle]
FR (Front Right): Pins 2, 3, 4
BL (Back Left):   Pins 11, 12, 13
BR (Back Right):  Pins 8, 9, 10
```

## 🔧 Customization

Adjust robot dimensions in `stabilization_control.py`:
```python
self.BODY_LENGTH = 274  # mm
self.BODY_WIDTH = 140   # mm
self.LEG_LENGTH = 193   # mm
self.DEFAULT_HEIGHT = 125  # mm
```

## 🔍 Troubleshooting

| Issue | Solution |
|-------|----------|
| No IMU detected | 1. Check connections<br>2. Run `sudo i2cdetect -y 1`<br>3. Verify 3.3V power<br>4. Check I2C enabled |
| Arduino not found | 1. Check USB connection<br>2. Run `ls /dev/ttyACM*`<br>3. Set permissions with `sudo usermod -a -G dialout $USER` |
| Servo jitter | 1. Verify power supply capacity<br>2. Check ground connections<br>3. Reduce update frequency |
| Visualization lag | 1. Reduce visualization update rate<br>2. Close other applications<br>3. Check CPU usage |

## 🛟 Common Commands

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Monitor IMU data
python imu_stabilization.py

# Test servos individually
python
>>> import serial
>>> arduino = serial.Serial('/dev/ttyACM0', 115200)
>>> arduino.write(b"S,0,0,90;")  # Move leg 0, servo 0 to 90°

# Check system logs
journalctl -f  # Monitor system logs
dmesg | grep -i i2c  # Check I2C related messages
```

## 📝 Development Notes

- The system automatically detects if running on Raspberry Pi and switches to appropriate mode
- Hardware mode requires I2C and serial connections
- Simulation mode available for testing without hardware
- All angles are in degrees
- Servo commands format: "S,leg,servo,angle;" (e.g., "S,0,0,90;")

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