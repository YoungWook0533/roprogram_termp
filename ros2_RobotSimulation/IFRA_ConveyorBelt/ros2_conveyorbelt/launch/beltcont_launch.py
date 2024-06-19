from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_conveyorbelt',
            executable='beltcont_call.py',
            name='beltcont_call',
            output='screen'
        ),
        Node(
            package='ros2_conveyorbelt',
            executable='beltcont_srv.py',
            name='beltcont_srv',
            output='screen'
        ),
    ])
