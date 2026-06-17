import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    sim_package_dir = get_package_share_directory('webots_simulation')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([sim_package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    drone_handler_node = Node(
        package='drone_hardware',
        executable='drone_handler',
        parameters=[
            {'fc_ip': 'tcp:127.0.0.1:5762'}
        ]
    )

    drone_handler_node_action = TimerAction(
        period=10.0,
        actions=[
            drone_handler_node
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='swarm_test.wbt',
            description='Choose one of the world files from `/webots_simulation/resource/worlds` directory'
        ),
        webots,
        webots._supervisor,
        drone_handler_node_action,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
