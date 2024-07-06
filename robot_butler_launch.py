from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_butler',
            executable='robot_butler',
            name='robot_butler',
            output='screen'
        ),
        # Add other nodes such as navigation2 bringup here
    ])
