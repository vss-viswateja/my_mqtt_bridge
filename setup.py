from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_mqtt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'paho-mqtt', 'PyYAML'],
    zip_safe=True,
    maintainer='viswa',
    maintainer_email='vss.viswatejabottu@gmail.com',
    description='MQTT to ROS 2 Nav2 bridge for multi-robot fleet control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = my_mqtt_bridge.bridge_node:main',
        ],
    },
)
