from setuptools import find_packages, setup

package_name = 'droniada_inspekcja'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            "mission_reporter = droniada_inspekcja.mission_reporter:main",
            "simulate_mission = droniada_inspekcja.simulate_mission:main",
            "yolo_to_db = droniada_inspekcja.yolo_to_db:main",
        ],
    },
)
