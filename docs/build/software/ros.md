# ROS2 Jazzy Botzo Workspace Documentation

Build Botzo Workspace for ROS2 Jazzy

```bash
cd <your_ros2_ws>/src
git clone https://github.com/IERoboticsAILab/botzo.git
cd ..
colcon build
source install/setup.bash
```

Launch RViz Botzo Description

```bash
ros2 launch botzo_description display.launch.py
```


Naive home pose botzo
```bash
ros2 run botzo_description move_joint
```

Contribute
```bash
ros2 pkg create --build-type ament_python botzo_<package_function> --dependencies rclpy std_msgs
```