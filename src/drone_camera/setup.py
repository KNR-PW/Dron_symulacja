from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stas',
    maintainer_email='stas.kolodziejczyk@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_publisher=drone_camera.camera_publisher:main",
            "image_publisher=drone_camera.image_publisher:main",
            "image_subscriber=drone_camera.image_subscriber:main",
            "images_recorder=drone_camera.images_recorder:main",
            "video_recorder=drone_camera.video_recorder:main",
            "mission_make_photo_server = drone_camera.mission_make_photo_server:main",
        ],
    },
)
