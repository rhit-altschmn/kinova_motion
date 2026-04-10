from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rec_rep2',
            executable='replayer',
            name='motion_replayer',
            output='screen',
        )
    ])