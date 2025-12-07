from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_car'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Models directory
        ('share/' + package_name + '/models', glob('models/*.pt')),
        # Launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alon Malka',
    maintainer_email='alon@example.com',
    description='Autonomous car with YOLO leg tracking and ROS 2 Jazzy',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = autonomous_car.camera_node:main',
            'detector_node = autonomous_car.detector_node:main',
            'motor_node = autonomous_car.motor_node:main',
            'mjpeg_server = autonomous_car.mjpeg_server:main',
        ],
    },
)
