'''
The control pipeline shared by the kinematic viewer and the simulator:
  /cmd_vel -> gait_planner -> /target_end_effectors -> joint_publisher (IK) -> joint angles

Included by walk.launch.py (angles go straight to /joint_states) and by
botzo_gazebo/sim.launch.py (angles go to the simulated servos).
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gait_params = os.path.join(get_package_share_directory('botzo_bringup'), 'config', 'gait.yaml')
    gait = LaunchConfiguration('gait')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument('gait', default_value='trot', choices=['trot', 'legacy']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ik_output', default_value='/joint_states',
                              description='Topic the IK joint angles are published on'),

        Node(package='botzo_ik', executable='joint_publisher', output='screen',
             parameters=[sim_time],
             remappings=[('/joint_states', LaunchConfiguration('ik_output'))]),
        Node(package='botzo_gaitplan', executable='gait_planner', output='screen',
             parameters=[gait_params, sim_time],
             condition=IfCondition(EqualsSubstitution(gait, 'trot'))),
        Node(package='botzo_gaitplan', executable='gait_planner_legacy', name='gait_planner',
             output='screen', parameters=[sim_time],
             condition=IfCondition(EqualsSubstitution(gait, 'legacy'))),
    ])
