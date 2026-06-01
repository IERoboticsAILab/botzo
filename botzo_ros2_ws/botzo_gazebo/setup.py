from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'botzo_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gringo',
    maintainer_email='gringo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={},
)
