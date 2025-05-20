from setuptools import setup

package_name = 'drone_hardware'

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
    maintainer='kuba',
    maintainer_email='ntfr123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drone_handler=drone_hardware.drone_handler:main",
            "healthcheck=drone_hardware.healthcheck:main",
            "host_bridge=drone_hardware.host_hardware:main",
            "uart_bridge = drone_hardware.uartbridge:main",
            "hbridge = drone_hardware.hbridge:main",

        ],
    },
)
