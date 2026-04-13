# ROS2 Jazzy Botzo Workspace Documentation

## Build Botzo Workspace for ROS2 Jazzy

```bash
cd <your_ros2_ws>/src
git clone https://github.com/IERoboticsAILab/botzo.git
cd ..
colcon build
source install/setup.bash
```

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













## Moveit

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