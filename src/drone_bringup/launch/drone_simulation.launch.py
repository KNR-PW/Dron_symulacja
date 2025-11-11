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
    
    yolo_node = Node(
            package = 'ros2_yolo',
            executable = 'yolo_node',
    )
    
    mission_make_photo_server = Node(
        package='drone_camera',
        executable='images_recorder',
        output='screen',
        parameters=[{
            'camera_topic': '/gimbal_camera',
            # katalog bazowy na misje
            # 'save_directory_base': 'Dron_symulacja/src/drone_camera/drone_camera',
            # nazwa serwisu z ImagesRecorder (Trigger), który zwraca "Saved: <ścieżka>"
            # 'images_service_name': '/take_picture',
            # 'move_instead_of_copy': False,
    }],
    )
    healthcheck = Node(
            package='drone_hardware',
            executable='healthcheck',
            parameters=[
                {"required_nodes": ['aruco_node', 'yolo_node', 'ros_mission_website', 'ros_report_website', 'mission_reporter']},
            ]
        )
    # Delay running drone_handler to wait for  webots init
    drone_handler_node_action = TimerAction(
            period=10.0,
            actions=[
                drone_handler_node,
                aruco_node,
                yolo_node
            ]
        )

    healthcheck_action = TimerAction(
            period=16.0,
            actions=[
                healthcheck
            ]
        )

    web_telemetry = Node(
           package='drone_web',
           executable='ros_mission_website',
           parameters=[
               {'base_url': 'https://telemetria-osadniik.pythonanywhere.com/',
                "camera_topic": 'camera',}
           ]
        )
    
    web_inspekcja = Node(
           package='drone_web',
           executable='ros_report_website',
           parameters=[
               {'base_url': 'https://inspekcja-osadniik.pythonanywhere.com/'}
           ]
        )

    mission_reporter = Node(
        package='droniada_inspekcja',
        executable='mission_reporter',
        parameters=[
            {'db_path': 'drone_data.db'}
        ]
    )

    host_bridge = Node(
            package='drone_hardware',
            executable='host_bridge'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='aruco_gimbal.wbt',
            description='Choose one of the world files from `/webots_simulation/resource/worlds` directory'
        ),
        webots,
        webots._supervisor,
        drone_handler_node_action,
        web_telemetry,
        # web_inspekcja,
        healthcheck_action,
        #images_recorder,
        mission_make_photo_server,
        # mission_reporter,
        # host_bridge,

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
