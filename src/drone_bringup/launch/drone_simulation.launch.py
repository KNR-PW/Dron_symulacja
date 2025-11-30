import os
import launch
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    sim_package_dir = get_package_share_directory('webots_simulation')
    
    # --- Arguments ---
    world = LaunchConfiguration('world')
    detector_mode = LaunchConfiguration('detector')     # 'yolo' or 'hybrid'
    tracker_type = LaunchConfiguration('tracker_type')  # 'KCF', 'CSRT', etc.

    webots = WebotsLauncher(
        world=PathJoinSubstitution([sim_package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    drone_handler_node = Node(
            package='drone_hardware',
            executable='drone_handler',
            parameters=[{'fc_ip': 'tcp:127.0.0.1:5762'}]
        )

    aruco_node = Node(
            package='ros2_aruco',
            executable='aruco_node',
        )
    
    # --- Conditional Nodes ---
    
    # 1. Standard YOLO Node (Runs only if detector=='yolo')
    yolo_node = Node(
            package='ros2_yolo',
            executable='yolo_node',
            condition=IfCondition(PythonExpression(["'", detector_mode, "' == 'yolo'"]))
    )

    # 2. Hybrid Tracker Node (Runs only if detector=='hybrid')
    hybrid_tracker_node = Node(
            package='drone_detector',
            executable='hybrid_tracker_node',
            parameters=[{'tracker_type': tracker_type}],
            condition=IfCondition(PythonExpression(["'", detector_mode, "' == 'hybrid'"]))
    )
    
    mission_make_photo_server = Node(
        package='drone_camera',
        executable='images_recorder',
        output='screen',
        parameters=[{'camera_topic': '/gimbal_camera'}],
    )
    
    # --- Dynamic Healthcheck ---
    # We need to tell the healthcheck which node to expect based on the mode
    # This is a bit complex in launch files, so we'll just list both as optional 
    # or create two healthcheck definitions. For simplicity, let's just check common nodes here.
    healthcheck = Node(
            package='drone_hardware',
            executable='healthcheck',
            parameters=[
                {"required_nodes": ['aruco_node', 'ros_mission_website', 'ros_report_website', 'mission_reporter']},
            ]
        )

    # Delay running drone_handler to wait for webots init
    drone_handler_node_action = TimerAction(
            period=10.0,
            actions=[
                drone_handler_node,
                aruco_node,
                yolo_node,           # Will only start if condition is met
                hybrid_tracker_node  # Will only start if condition is met
            ]
        )

    healthcheck_action = TimerAction(
            period=16.0,
            actions=[healthcheck]
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
        # --- New Arguments Definitions ---
        DeclareLaunchArgument(
            'detector',
            default_value='yolo',
            description='Choose detector mode: "yolo" or "hybrid"'
        ),
        DeclareLaunchArgument(
            'tracker_type',
            default_value='KCF',
            description='Tracker algorithm for hybrid mode: KCF, CSRT, MIL, MOSSE'
        ),
        # ---------------------------------
        webots,
        webots._supervisor,
        drone_handler_node_action,
        web_telemetry,
        healthcheck_action,
        mission_make_photo_server,
        
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
