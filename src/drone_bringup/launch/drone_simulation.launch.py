import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    sim_package_dir = get_package_share_directory('webots_simulation')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([sim_package_dir, 'worlds', world]),
        ros2_supervisor=True
    )


    sim_manager_description_path = os.path.join(sim_package_dir, 'resource', 'webots_sim_manager.urdf')
    ros2_webots_sim_manager = WebotsController(
        robot_name='ros2_sim_manager',
        parameters=[
            {'robot_description': sim_manager_description_path},
        ],
        respawn=True
    )

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

    healthcheck = Node(
            package='drone_hardware',
            executable='healthcheck',
        )
    # Delay running drone_handler to wain for  webots init
    drone_handler_node_action = TimerAction(
            period=5.0,  # Delay of 5 seconds
            actions=[
                drone_handler_node,
                aruco_node
            ]
        )

    healthcheck_action = TimerAction(
            period=12.0,
            actions=[
                healthcheck
            ]
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='2_cameras.wbt',
            description='Choose one of the world files from `/webots_simulation/resource/worlds` directory'
        ),
        webots,
        webots._supervisor,
        ros2_webots_sim_manager,
        drone_handler_node_action,
        healthcheck_action,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
