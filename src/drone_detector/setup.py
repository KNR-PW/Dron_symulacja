from setuptools import setup
from glob import glob
import os
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'drone_detector'
# cur_directory_path = os.path.abspath(os.path.dirname(__file__))
# inner_dir_path = os.path.join(cur_directory_path, 'drone_detector')
# cars_file_path = os.path.join(cur_directory_path, 'drone_detector', 'car_counting.mp4')
# d = generate_distutils_setup(
#     packages=[package_name],
#     package_dir={'': package_name}
# )
# setup(**d)
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('video_files/*.mp4')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stas',
    maintainer_email='stas.kolodziejczyk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "detector=drone_detector.detector:main",
            "detection_subscriber=drone_detector.detection_subscriber:main",
            "detector_server=drone_detector.detector_server:main",
            "detection_client=drone_detector.detection_client:main",
            "aruco_detector=drone_detector.aruco_detector:main",
            "pool_detector=drone_detector.pool_detector:main",
        ],
    },
)
