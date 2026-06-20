#!/usr/bin/env python3

'''
Generate GUI with 12 sliders to control the 12 servos of the real robot. 
The set angles are published to the `/real_robot_joint_states` topic. 
The `move_real_robot.py` script is waiting for a message in this topic to move the servos to the desired angles. The script will transform the angles from degrees to PWM using the calibration coefficients and send them to the Arduino via serial communication.
┌──────────────────────────────────────────────┐
│          BOTZO SERVO CONTROLLER              │
├──────────────────────────────────────────────┤
│ Front Left        │ Front Right              │
│ Hip    ─────  0°  │ Hip    ─────  0°         │
│ Upper  ───── 45°  │ Upper  ───── 45°         │
│ Lower  ─────-90°  │ Lower  ─────-90°         │
├──────────────────────────────────────────────┤
│ Back Left         │ Back Right               │
│ Hip    ─────  0°  │ Hip    ─────  0°         │
│ Upper  ───── 45°  │ Upper  ───── 45°         │
│ Lower  ─────-90°  │ Lower  ─────-90°         │
├──────────────────────────────────────────────┤
│ HOME   STAND   SIT   STOP                    │
├──────────────────────────────────────────────┤
│ Publishing: ● Connected                      │
└──────────────────────────────────────────────┘

Generate a GUI with 12 sliders to control the 12 servos of the real robot.

The GUI is organized into four panels, one for each leg:
    - Front Left
    - Front Right
    - Back Left
    - Back Right

Each panel contains three sliders corresponding to the three joints:
    - Hip
    - Femur (Upper)
    - Tibia (Lower)

Moving any slider updates the desired servo angle (in degrees) and
immediately publishes a message on the `/real_robot_joint_states`
topic.

The `move_real_robot.py` node subscribes to this topic, converts the
angles from degrees to PWM using the servo calibration coefficients,
and sends the commands to the Arduino over serial to move the robot.

/real_robot_joint_states topic message type: botzo_messages/msg/RealRobotJointStates.msg
float32 sfr
float32 ffr
float32 tfr

float32 sfl
float32 ffl
float32 tfl

float32 sbr
float32 fbr
float32 tbr

float32 sbl
float32 fbl
float32 tbl
'''

import sys

import rclpy
from rclpy.node import Node

from botzo_messages.msg import RealRobotJointStates

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QSlider,
    QGridLayout,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
)
from PySide6.QtCore import Qt
from PySide6.QtCore import QTimer



INITIAL_ANGLES = {
    "sfl": 90,
    "ffl": 90,
    "tfl": 180,

    "sfr": 90,
    "ffr": 90,
    "tfr": 0,

    "sbl": 90,
    "fbl": 90,
    "tbl": 180,

    "sbr": 90,
    "fbr": 90,
    "tbr": 0,
}


# ---------------- ROS NODE ---------------- #

class RealRobotJointStatesPublisher(Node):
    def __init__(self):
        super().__init__("real_robot_joint_states_publisher")
        self.publisher_ = self.create_publisher(RealRobotJointStates, "/real_robot_joint_states", 10)

    def publish_real_robot_joint_states(self, sfr, ffr, tfr, sfl, ffl, tfl, sbr, fbr, tbr, sbl, fbl, tbl):
        msg = RealRobotJointStates()
        msg.sfr = float(sfr)
        msg.ffr = float(ffr)
        msg.tfr = float(tfr)
        msg.sfl = float(sfl)
        msg.ffl = float(ffl)
        msg.tfl = float(tfl)
        msg.sbr = float(sbr)
        msg.fbr = float(fbr)
        msg.tbr = float(tbr)
        msg.sbl = float(sbl)
        msg.fbl = float(fbl)
        msg.tbl = float(tbl)
        self.publisher_.publish(msg)


# ---------------- GUI ---------------- #
# ---------------- GUI ---------------- #

class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node

        self.setWindowTitle("BOTZO SERVO CONTROLLER")
        self.resize(900, 550)

        self.sliders = {}
        self.value_labels = {}

        main_layout = QVBoxLayout()
        grid = QGridLayout()

        grid.addWidget(self.create_leg_box("Front Left", "fl"), 0, 0)
        grid.addWidget(self.create_leg_box("Front Right", "fr"), 0, 1)
        grid.addWidget(self.create_leg_box("Back Left", "bl"), 1, 0)
        grid.addWidget(self.create_leg_box("Back Right", "br"), 1, 1)

        main_layout.addLayout(grid)

        status = QLabel("Publishing ● Connected")
        status.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(status)

        self.setLayout(main_layout)

        self.update_joint_states()

    def create_leg_box(self, title, leg):

        box = QGroupBox(title)

        layout = QVBoxLayout()

        names = {
            "s": "Hip",
            "f": "Femur",
            "t": "Tibia"
        }

        for joint in ["s", "f", "t"]:
            slider_name = f"{joint}{leg}"

            row = QHBoxLayout()

            text = QLabel(names[joint])
            text.setFixedWidth(60)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(INITIAL_ANGLES[slider_name])

            value = QLabel(f"{slider.value()}°")
            value.setFixedWidth(45)

            slider.valueChanged.connect(
                lambda v, lbl=value: lbl.setText(f"{v}°")
            )

            slider.valueChanged.connect(self.update_joint_states)

            self.sliders[slider_name] = slider
            self.value_labels[slider_name] = value

            row.addWidget(text)
            row.addWidget(slider)
            row.addWidget(value)

            layout.addLayout(row)

        box.setLayout(layout)

        return box

    def update_joint_states(self):

        self.ros_node.publish_real_robot_joint_states(

            self.sliders["sfr"].value(),
            self.sliders["ffr"].value(),
            self.sliders["tfr"].value(),

            self.sliders["sfl"].value(),
            self.sliders["ffl"].value(),
            self.sliders["tfl"].value(),

            self.sliders["sbr"].value(),
            self.sliders["fbr"].value(),
            self.sliders["tbr"].value(),

            self.sliders["sbl"].value(),
            self.sliders["fbl"].value(),
            self.sliders["tbl"].value(),
        )



def main():

    rclpy.init()

    ros_node = RealRobotJointStatesPublisher()

    app = QApplication(sys.argv)

    window = MainWindow(ros_node)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()