from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from launch.actions import SetEnvironmentVariable





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

    gazeboo = ExecuteProcess(
            cmd=[
                'gz', 'sim', '-v4', '-r', 'iris_runway.sdf'
            ]
        )
    return LaunchDescription([
        SetEnvironmentVariable(
          'GZ_SIM_SYSTEM_PLUGIN_PATH',
          '/tools/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}'
        ),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/tools/ardupilot_gazebo/models:/tools/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}'
        ),
        gazeboo,
        drone_handler_node_action
    ])