from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    drone_handler_node = Node(
            package='drone_hardware',
            executable='drone_handler',
            parameters=[
                {'fc_ip': 'tcp:127.0.0.1:5762'}
            ]
        )

    aruco_node = Node(
            package='ros2_aruco',
            executable='aruco_node',
        )

    # Delay running drone_handler to wain for gazeboo init
    drone_handler_node_action = TimerAction(
            period=10.0,
            actions=[
                drone_handler_node,
                aruco_node
            ]
        )

    # Swiat wybierany argumentem, np.:  ros2 launch ... world:=terrain_tent_sunset.sdf
    world_arg = DeclareLaunchArgument(
            'world',
            default_value='aruco_plain.sdf',
            description='Plik swiata Gazebo (.sdf) z GZ_SIM_RESOURCE_PATH'
        )

    gazeboo = ExecuteProcess(
            cmd=[
                'gz', 'sim', '-v4', '-r', LaunchConfiguration('world')
            ]
        )
    return LaunchDescription([
        world_arg,
        SetEnvironmentVariable(
          'GZ_SIM_SYSTEM_PLUGIN_PATH',
          '/tools/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}'
        ),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            # Najpierw swiaty/modele z projektu (montowane przez src/) - maja
            # priorytet i pozwalaja nadpisywac wbudowane. Potem wbudowane w obraz.
            '/root/ros_ws/src/drone_bringup/gazebo/models:'
            '/root/ros_ws/src/drone_bringup/gazebo/worlds:'
            '/tools/ardupilot_gazebo/models:/tools/ardupilot_gazebo/worlds:'
            '${GZ_SIM_RESOURCE_PATH}'
        ),
        gazeboo,
        drone_handler_node_action
    ])