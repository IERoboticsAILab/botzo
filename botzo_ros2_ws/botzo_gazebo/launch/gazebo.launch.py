import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Package paths ---
    botzo_gazebo_pkg   = get_package_share_directory('botzo_gazebo')
    botzo_desc_pkg     = get_package_share_directory('botzo_description')
    gazebo_ros_pkg     = get_package_share_directory('gazebo_ros')

    # --- URDF ---
    urdf_path = os.path.join(botzo_desc_pkg, 'urdf', 'botzo.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # --- World ---
    world_path = os.path.join(botzo_gazebo_pkg, 'worlds', 'empty.world')

    # ------------------------------------------------------------------ #
    #  Nodes                                                               #
    # ------------------------------------------------------------------ #

    # 1. Gazebo Classic server + client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'false'}.items(),
    )

    # 2. robot_state_publisher — publishes /tf from URDF + /joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # 3. Spawn the robot into Gazebo
    #    gazebo_ros spawn_entity reads /robot_description and drops the model in the world
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_botzo',
        output='screen',
        arguments=[
            '-entity', 'botzo',           # model name in Gazebo
            '-topic', 'robot_description', # reads from robot_state_publisher
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',                  # small offset so it doesn't clip the ground
        ],
    )

    # ------------------------------------------------------------------ #
    #  Future sensors — add bridge nodes here when ready                   #
    # ------------------------------------------------------------------ #
    # Example (uncomment when you add IMU):
    #
    # imu_bridge = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     arguments=['/imu/data', '/imu'],
    # )

    return LaunchDescription([
        # add arguments: use_sim_time is needed for Gazebo to sync the clock and use rviz or not
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('rviz', default_value='false', description='Whether to launch RViz'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])