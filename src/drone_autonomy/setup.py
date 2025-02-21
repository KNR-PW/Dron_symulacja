from setuptools import setup

package_name = 'drone_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "goto_detection_client=drone_autonomy.goto_detection_client:main",
            "goto_detection_group=drone_autonomy.goto_detection_group:main",
            "mission=drone_autonomy.mission:main",
        ],
    },
)
