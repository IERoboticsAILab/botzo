'''
Kinematic walking (no physics): the IK angles are drawn directly on the URDF.
  /cmd_vel -> gait_planner -> joint_publisher (IK) -> /joint_states -> robot_state_publisher
plus foxglove_bridge on ws://localhost:8765 for visualisation.

  ros2 launch botzo_bringup walk.launch.py
  ros2 launch botzo_bringup walk.launch.py gait:=legacy     # old planner, for comparison
  ros2 launch botzo_bringup walk.launch.py foxglove:=false

For physics use botzo_gazebo's sim.launch.py instead.
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('botzo_description'), 'urdf', 'botzo.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()
    control_launch = os.path.join(get_package_share_directory('botzo_bringup'), 'launch', 'control.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('gait', default_value='trot', choices=['trot', 'legacy']),
        DeclareLaunchArgument('foxglove', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        IncludeLaunchDescription(control_launch, launch_arguments={'gait': LaunchConfiguration('gait')}.items()),
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8765, 'address': '0.0.0.0'}],
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
