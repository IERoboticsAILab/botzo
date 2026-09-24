from glob import glob

from setuptools import setup

package_name = 'botzo_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botzo',
    maintainer_email='botzoteam@gmail.com',
    description='Launch files and parameters to run the botzo stack.',
    license='MIT',
)
