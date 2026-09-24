'''
Botzo in Gazebo Harmonic (gz sim), headless by default, watched through Foxglove.

  /cmd_vel -> gait_planner -> joint_publisher (IK) -> /ik/joint_states
           -> joint_command_relay -> position_controller (gz_ros2_control) -> physics
  physics  -> /joint_states (measured), /odom + odom->base TF (ground truth)

  ros2 launch botzo_gazebo sim.launch.py
  ros2 launch botzo_gazebo sim.launch.py gait:=legacy
  ros2 launch botzo_gazebo sim.launch.py gz_args:="-r <world.sdf>"   # with the gz GUI (needs a display)
'''

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from botzo_gazebo.sim_description import make_sim_urdf
from botzo_ik.joint_publisher import adjust_angles_to_sim, legIK


def home_joint_positions(gait_yaml):
    '''Joint angles (rad) of the gait's home stance, so the robot spawns standing.'''
    with open(gait_yaml) as f:
        params = yaml.safe_load(f)['gait_planner']['ros__parameters']
    legs = ('fl', 'fr', 'bl', 'br')
    angles = adjust_angles_to_sim(*[list(legIK(*params[f'home_{leg}'])) for leg in legs])
    positions = {}
    for leg, (shoulder, femur, tibia) in zip(legs, angles):
        for part, deg in (('shoulder', shoulder), ('femur', femur), ('tibia', tibia)):
            positions[f'{leg.upper()}_{part}_joint'] = math.radians(deg)
    return positions


def generate_launch_description():
    description_share = get_package_share_directory('botzo_description')
    gazebo_share = get_package_share_directory('botzo_gazebo')
    controllers_yaml = os.path.join(gazebo_share, 'config', 'controllers.yaml')
    world = os.path.join(gazebo_share, 'worlds', 'flat.sdf')

    gait_yaml = os.path.join(get_package_share_directory('botzo_bringup'), 'config', 'gait.yaml')
    with open(os.path.join(description_share, 'urdf', 'botzo.urdf')) as f:
        robot_description = make_sim_urdf(f.read(), controllers_yaml, home_joint_positions(gait_yaml))
    with open(controllers_yaml) as f:
        controller_joints = yaml.safe_load(f)['position_controller']['ros__parameters']['joints']

    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('gait', default_value='trot', choices=['trot', 'legacy']),
        DeclareLaunchArgument('gz_args', default_value=f'-r -s -v 2 {world}',
                              description='gz sim arguments; -s = server only (no GUI)'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('spawn_z', default_value='0.13'),  # just above the standing height

        # Lets gz resolve package://botzo_description/... mesh URIs.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description_share)),

        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args'), 'on_exit_shutdown': 'true'}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'botzo', '-topic', 'robot_description',
                        '-z', LaunchConfiguration('spawn_z')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/model/botzo/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/model/botzo/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
             remappings=[('/model/botzo/odometry', '/odom'), ('/model/botzo/pose', '/tf')]),

        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'position_controller',
                        '--controller-manager-timeout', '60']),
        Node(package='botzo_gazebo', executable='joint_command_relay',
             parameters=[{'joints': controller_joints}, sim_time]),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('botzo_bringup'), 'launch', 'control.launch.py'),
            launch_arguments={'gait': LaunchConfiguration('gait'), 'use_sim_time': 'true',
                              'ik_output': '/ik/joint_states'}.items()),

        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8765, 'address': '0.0.0.0'}, sim_time],
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
