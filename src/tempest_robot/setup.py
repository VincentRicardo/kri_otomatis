import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'tempest_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tempest',
    maintainer_email='tempest@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_key_servo = tempest_robot.teleop_key_servo:main",
            "nano0_server = tempest_robot.nano0_server:main",
            "nano1_server = tempest_robot.nano1_server:main",
            "sending_angle = tempest_robot.sending_angle:main",
            "gyro_node = tempest_robot.gyro_node:main",
            "camera_node = tempest_robot.camera_node:main",
            "tof1_node = tempest_robot.tof1_node:main",
            "tof2_node = tempest_robot.tof2_node:main",
            "tof3_node = tempest_robot.tof3_node:main",
            "olah_node = tempest_robot.olah_node:main"
        ],
    },
)
