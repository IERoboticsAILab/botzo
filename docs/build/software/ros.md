# ROS2 Jazzy Botzo Workspace Documentation

## Build Botzo Workspace for ROS2 Jazzy

```bash
cd <your_ros2_ws>/src
git clone https://github.com/IERoboticsAILab/botzo.git
cd ..
colcon build
source install/setup.bash
```

## Launch RViz Botzo Description

```bash
ros2 launch botzo_description display.launch.py
```


## Naive home pose botzo
```bash
ros2 run botzo_description move_joint
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