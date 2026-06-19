# ROS2 Jazzy Botzo Workspace Documentation

# Build Botzo Workspace for ROS2 Jazzy and run botzo

```bash
cd <your_ros2_ws>/src
git clone https://github.com/IERoboticsAILab/botzo.git
cd ..
colcon build
source install/setup.bash

ros2 launch botzo_description display.launch.py # make sure to close the joint publisher GUI before running the next command
ros2 run botzo_ik joint_publisher
ros2 run botzo_gaitplan gait_planner

# publish to /cmd_vel topic to move the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# OR with a joystick
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js0"
ros2 run botzo_joystick joy_to_cmd_vel

# to move the robot according to the target end-effectors, run this in the Raspberry Pi of the robot:
ros2 run botzo_serialcomm move_robot
```




# Packages:

## Launch RViz Botzo Description (`botzo_description`)

```bash
ros2 launch botzo_description display.launch.py
```
### Naive home pose botzo
```bash
ros2 run botzo_description move_joint
```



## Add custom messages (`botzo_messages`)

```bash
cd <your_ros2_ws>/src/botzo/botzo_ros2_ws/botzo_messages/msg
```
add your custom message file (e.g. `TargetEndEffectors.msg`)

Update `CMakeLists.txt` to include the new message file, e.g. add the following lines:

```bash
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetEndEffectors.msg"
)
```

Build and source the workspace again:
```bash
cd <your_ros2_ws>
colcon build
```

Check that the message is available:
```bash
ros2 interface show botzo_messages/msg/TargetEndEffectors
```



## Botzo IK (`botzo_ik`)
This package contains `joint_publisher.py` node that subscribes to the target end-effectors and publishes the corresponding joint states using our IK solver.

Target end-effectors topic: `/target_end_effectors`

Message type: `botzo_messages/msg/TargetEndEffectors` (custom message that contains the desired positions (x, y, z) for each end-effector (fl, fr, bl, br))

Recive the targets and publish the corresponding joint states to the topic: `/joint_states` (message type: `sensor_msgs/msg/JointState`) -> move the robot to the desired leg positions in Rviz

1. Run the `joint_publisher` node:

    ```bash
    ros2 run botzo_ik joint_publisher
    ```

2. Publish a target message example:

    ```bash
    ros2 topic pub /target_end_effectors botzo_messages/msg/TargetEndEffectors "{
    x_fl: 0.0, y_fl: 2.0, z_fl: 8.0,
    x_fr: 0.0, y_fr: 2.0, z_fr: 8.0,
    x_bl: 0.0, y_bl: 2.0, z_bl: 8.0,
    x_br: 0.0, y_br: 2.0, z_br: 8.0
    }"
    ```

WHATEVER YOU PUBLISH TO THE `/target_end_effectors` TOPIC, THE `joint_publisher` NODE WILL CALCULATE THE CORRESPONDING JOINT ANGLES USING THE IK SOLVER AND PUBLISH THEM TO THE `/joint_states` TOPIC, WHICH WILL UPDATE THE ROBOT'S POSE IN RVIZ ACCORDINGLY.




## Walk
GaitPlan: https://www.geogebra.org/calculator/d4hauhcg

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`gait_planner.py` subscribes to `/cmd_vel` (geometry_msgs/msg/Twist) and publishes the target end-effectors to `/target_end_effectors` on a sine wave (botzo_messages/msg/TargetEndEffectors) based on the desired linear and angular velocity commands.
```bash
ros2 run botzo_gaitplan gait_planner
```


## Connect joystick

```bash
ros2 run joy joy_node --ros-args -p dev:="/dev/input/js0"
```

this will publish the joystick buttons pressed and moved to `/joy` topic.

Transfrom this message:
```bash
axes:
- -0.0      # L lever <1;-1> left-right
- -0.0      # L lever <1;-1> forward-backward
- 1.0       # L2 <1;-1>
- -0.0      # R lever <1;-1> left-right
- -0.0      # R lever <1;-1> forward-backward
- 1.0       # R2 <1;-1>
- 0.0       # arrow left-right <1,-1>
- 0.0       # arrow forward-backward <1,-1>
buttons:
- 0         # x
- 0         # circle
- 0         # square
- 0         # triangle
- 0         # L1
- 0         # R1
- 0         # share
- 0         # options
- 0         # PS
- 0         # L3
- 0         # R3
---
```

Run 
```bash
ros2 run botzo_joystick joy_to_cmd_vel
```
script to transform the joystick messages to velocity commands (`/cmd_vel` topic) that can be used by the gait planner to move the robot.






## Gazebo

```bash
ros2 pkg list | grep gazebo
gazebo_dev
gazebo_msgs
gazebo_plugins          # <-- important
gazebo_ros              # <-- important
gazebo_ros2_control     # <-- important
gazebo_ros_pkgs
```

```bash
tree .
.
├── botzo_gazebo
│   ├── __init__.py
│   └── joint_controller.py
├── config
│   └── controllers.yaml
├── launch
│   └── gazebo.launch.py
├── package.xml
├── resource
│   └── botzo_gazebo
├── setup.cfg
├── setup.py
└── worlds
    └── empty.world
```

```bash
gazebo --version
Gazebo multi-robot simulator, version 11.10.2
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org


Gazebo multi-robot simulator, version 11.10.2
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org
```

1. Start Gazebo.
2. Load your robot URDF.
3. Spawn the robot.
4. Start robot_state_publisher.
5. Start ros2_control.
6. Spawn joint controllers.
7. Allow commanding joints

```bash
ros2 launch botzo_gazebo gazebo.launch.py
```

> working in progress











## Rotations

> working in progress

#### Publish IMU data

> working in progress














## Stabilization

> working in progress


















## Connect to the real robot

1. Subscribe to /joint_states
2. Transform current joint states angle from sim angles to real robot angles
3. Transfom radinats into PWM (using calibration coefficients)
4. Connect to Arduino
5. Send angles to servos 
```bash
ros2 run botzo_serialcomm move_robot
```

## Debuging Utils

1. Scipt to move the real robot servos to user specific angles (in degrees) using the calibration coefficients and the serial communication with the Arduino. Usefull to check if the calibration coefficients are correct and if the servos are working properly. The script will move the servos to the specified angles and then return them to the home position.
```bash
ros2 run botzo_debuggingutills move_real_robot_servos
```



















## Moveit

> working in progress

Install Moveit: https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html
Tutorial: https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```







## Contribute
```bash
ros2 pkg create --build-type ament_python botzo_<package_function> --dependencies rclpy std_msgs
```

```bash
ros2 pkg create --build-type ament_python botzo_<package_function> --dependencies rclpy std_msgs geometry_msgs botzo_messages
```