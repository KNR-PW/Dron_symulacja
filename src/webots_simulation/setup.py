from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'webots_simulation'

data_files=[]

data_files.append(('share/ament_index/resource_index/packages',
            ['resource/' + package_name]))

data_files.append(('share/' + package_name, ['package.xml']))

# data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))

data_files.append(('share/' + package_name + '/objects', 
    # 'resource/worlds/iris_camera.wbt',
    glob('resource/objects/**', recursive=False)
    # 'resource/worlds/depth_mavic_world_no_gravity.wbt',
))
data_files.append(('share/' + package_name + '/worlds', 
    # 'resource/worlds/iris_camera.wbt',
    glob('resource/worlds/**', recursive=False)
    # 'resource/worlds/depth_mavic_world_no_gravity.wbt',
))
data_files.append(('share/' + package_name + '/protos', 
    # 'resource/worlds/iris_camera.wbt',
    glob('resource/protos/*.proto')
    # 'resource/worlds/depth_mavic_world_no_gravity.wbt',
))
data_files.append(('share/' + package_name + '/protos' + "/textures", 
    # 'resource/worlds/iris_camera.wbt',
    glob('resource/protos/textures/*.png')
    # 'resource/worlds/depth_mavic_world_no_gravity.wbt',
))

data_files.append(('share/' + package_name + '/protos' + "/meshes", 
    # 'resource/worlds/iris_camera.wbt',
    glob('resource/protos/meshes/*.dae')
    # 'resource/worlds/depth_mavic_world_no_gravity.wbt',
))

data_files.append(('share/' + package_name + '/controllers' + '/ardupilot_sitl_controller', 
    # 'resource/worlds/iris_camera.wbt',
    glob('resource/controllers/ardupilot_sitl_controller/*.py')
    # 'resource/worlds/depth_mavic_world_no_gravity.wbt',
))

data_files.append(('share/' + package_name + '/resource', [
    'resource/webots_sim_manager.urdf'
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            "ardupilot_camera_handler=webots_simulation.ardupilot_camera_handler:main",
            'tesla_circle_driver = webots_simulation.tesla_circle_driver:main',
        ],
    },
)
