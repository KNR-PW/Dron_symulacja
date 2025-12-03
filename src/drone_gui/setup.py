from setuptools import setup

package_name = 'drone_gui'

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
            "detector_gui=drone_gui.detector_gui:main",
            "drone_control_gui=drone_gui.drone_control_gui:main",
            "control_gui=drone_gui.drone_controll_panel:main",
        ],
    },
)
