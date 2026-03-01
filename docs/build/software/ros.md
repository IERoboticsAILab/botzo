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
ros2 launch botzo_ros2_ws display.launch.py
```