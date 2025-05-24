from setuptools import setup
from glob import glob
package_name = 'drone_web'

data_files = []

data_files.append(('share/ament_index/resource_index/packages',
            ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stas',
    maintainer_email='stas.kolodziejczyk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "app=drone_web.app:main",
            "ros_mission_website=drone_web.ros_mission_website:main",
            "ros_report_website=drone_web.ros_report_website:main",
        ],
    },
)
