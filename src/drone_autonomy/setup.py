from setuptools import setup, find_packages

package_name = 'drone_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    # ← instaluj CAŁY pakiet wraz z podpakietami (drone_comunication itd.)
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stas',
    maintainer_email='stas.kolodziejczyk@gmail.com',
    description='Drone autonomy nodes (missions, ArUco follow, etc.)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_detection_client=drone_autonomy.goto_detection_client:main',
            'goto_detection_group=drone_autonomy.goto_detection_group:main',
            'mission=drone_autonomy.mission:main',
            'fly_to_aruco_example=drone_autonomy.fly_to_aruco_example:main',
            'follow_aruco_centroid=drone_autonomy.follow_aruco_centroid:main',
            'vtol_servo_test=drone_autonomy.vtol_servo_test:main',
        ],
    },
)
