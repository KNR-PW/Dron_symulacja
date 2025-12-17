from setuptools import find_packages, setup
import os

package_name = 'mqtt_telemetry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['mqtt_telemetry/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS2 → MQTT bridge',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_bridge = mqtt_telemetry.__init__:main',
            'publish_image_dzialka = mqtt_telemetry.publish_image_dzialka:main',
        ],
    },
)
