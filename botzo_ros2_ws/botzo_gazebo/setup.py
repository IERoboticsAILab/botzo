from glob import glob

from setuptools import find_packages, setup

package_name = 'botzo_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gringo',
    maintainer_email='greg.orlando77@gmail.com',
    description='Botzo simulation in Gazebo Harmonic (gz sim) with gz_ros2_control.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'joint_command_relay = botzo_gazebo.joint_command_relay:main',
            'gait_eval = botzo_gazebo.gait_eval:main',
        ],
    },
)
