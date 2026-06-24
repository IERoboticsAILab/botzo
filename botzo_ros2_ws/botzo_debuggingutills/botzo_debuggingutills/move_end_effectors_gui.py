#!/usr/bin/env python3

'''
Create GUI with sliders to move the end-effectors of the 4 legs in x, y, z coordinates.
Slide target for each leg, will trigger new message published to the /target_end_effectors topic,

which will be received by the joint_publisher IK node, which will calculate the corresponding joint angles using the IK solver 
and publish them to the /joint_states topic, which will update the robot's pose in Rviz accordingly.

/target_end_effectors topic message type: botzo_messages/msg/TargetEndEffectors.msg
    float32 x_fl
    float32 y_fl
    float32 z_fl

    float32 x_fr
    float32 y_fr
    float32 z_fr

    float32 x_bl
    float32 y_bl
    float32 z_bl

    float32 x_br
    float32 y_br
    float32 z_br

┌──────────────────────────────────────────────┐
│             BOTZO FOOT CONTROLLER            │
├──────────────────────────────────────────────┤
│ FL               │ FR                        │
│ X ───────── 0.0  │ X ───────── 0.0           │
│ Y ───────── 2.0  │ Y ───────── 2.0           │
│ Z ─────────16.0  │ Z ─────────16.0           │
├──────────────────────────────────────────────┤
│ BL               │ BR                        │
│ X ───────── 0.0  │ X ───────── 0.0           │
│ Y ───────── 2.0  │ Y ───────── 2.0           │
│ Z ─────────16.0  │ Z ─────────16.0           │
├──────────────────────────────────────────────┤
│  HOME   STAND   SIT   STOP                   │
├──────────────────────────────────────────────┤
│ Publishing: ● Connected                      │
└──────────────────────────────────────────────┘
'''

import sys

import rclpy
from rclpy.node import Node

from botzo_messages.msg import TargetEndEffectors

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


# ---------------- ROS NODE ---------------- #

class MoveEndEffectorsGui(Node):

    def __init__(self):
        super().__init__("move_end_effectors_gui")

        self.publisher = self.create_publisher(
            TargetEndEffectors,
            "target_end_effectors",
            10,
        )

    def publish(self, values):

        msg = TargetEndEffectors()

        msg.x_fl, msg.y_fl, msg.z_fl = values["fl"]
        msg.x_fr, msg.y_fr, msg.z_fr = values["fr"]
        msg.x_bl, msg.y_bl, msg.z_bl = values["bl"]
        msg.x_br, msg.y_br, msg.z_br = values["br"]

        self.publisher.publish(msg)


# ---------------- GUI ---------------- #

class MainWindow(QWidget):

    def __init__(self, ros_node):

        super().__init__()

        self.node = ros_node

        self.setWindowTitle("BOTZO Foot Controller")

        self.legs = {}

        layout = QVBoxLayout()

        grid = QGridLayout()

        grid.addWidget(self.create_leg("FL"), 0, 0)
        grid.addWidget(self.create_leg("FR"), 0, 1)
        grid.addWidget(self.create_leg("BL"), 1, 0)
        grid.addWidget(self.create_leg("BR"), 1, 1)

        layout.addLayout(grid)

        # Buttons
        buttons = QHBoxLayout()

        home = QPushButton("HOME")
        stand = QPushButton("STAND")
        sit = QPushButton("SIT")
        stop = QPushButton("STOP")

        home.clicked.connect(self.home_position)

        buttons.addWidget(home)
        buttons.addWidget(stand)
        buttons.addWidget(sit)
        buttons.addWidget(stop)

        layout.addLayout(buttons)

        self.status = QLabel("Publishing ● Connected")
        layout.addWidget(self.status)

        self.setLayout(layout)

    def create_leg(self, name):

        box = QGroupBox(name)

        layout = QVBoxLayout()

        sliders = {}

        defaults = {"X": 0, "Y": 2, "Z": 16}

        for axis in ["X", "Y", "Z"]:

            label = QLabel(f"{axis}: {defaults[axis]}")

            slider = QSlider(Qt.Horizontal)

            slider.setMinimum(-20)
            slider.setMaximum(20)

            slider.setValue(defaults[axis])

            slider.valueChanged.connect(
                lambda value, l=label, a=axis: self.slider_changed(l, a, value)
            )

            layout.addWidget(label)
            layout.addWidget(slider)

            sliders[axis] = slider

        box.setLayout(layout)

        self.legs[name.lower()] = sliders

        return box

    def slider_changed(self, label, axis, value):

        label.setText(f"{axis}: {value}")

        self.publish()

    def publish(self):

        values = {}

        for leg in self.legs:

            s = self.legs[leg]

            values[leg] = (
                float(s["X"].value()),
                float(s["Y"].value()),
                float(s["Z"].value()),
            )

        self.node.publish(values)

    def home_position(self):

        for leg in self.legs.values():

            leg["X"].setValue(0)
            leg["Y"].setValue(2)
            leg["Z"].setValue(16)


# ---------------- MAIN ---------------- #

def main():

    rclpy.init()

    node = MoveEndEffectorsGui()

    app = QApplication(sys.argv)

    window = MainWindow(node)
    window.resize(700, 500)
    window.show()

    # Allow ROS to process while Qt is running
    timer = app.timer = app.startTimer(10)

    def spin():
        rclpy.spin_once(node, timeout_sec=0)

    app.timerEvent = lambda event: spin()

    app.exec()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()