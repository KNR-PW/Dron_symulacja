from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

report_simulation = Node(
            package='droniada_inspekcja',
            executable='simulate_mission',
            # parameters=[
            #     {'db_path': 'drone_data.db'}
            # ]
        )
delayed_report_simulation = TimerAction(
    period=10.0,  # delay in seconds
    actions=[report_simulation]
)
def generate_launch_description():
    return LaunchDescription([
        Node(
           package='drone_web',
           executable='ros_report_website',
           parameters=[
               {'base_url': 'https://inspekcja-osadniik.pythonanywhere.com/'}
           ]
        ),
        Node(
            package='droniada_inspekcja',
            executable='mission_reporter',
            parameters=[
                {'db_path': 'drone_data.db'}
            ]
        ),
        # delayed_report_simulation
      
    ])