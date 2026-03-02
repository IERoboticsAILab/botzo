from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'botzo_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz_config'), glob('rviz_config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gringo',
    maintainer_email='gorlando.ieu2022@student.ie.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_joint = botzo_description.move_joint:main',
        ],
    },
)
