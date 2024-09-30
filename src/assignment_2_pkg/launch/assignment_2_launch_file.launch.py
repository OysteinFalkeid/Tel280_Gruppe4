from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='assignment_2_pkg',
            executable='wall_follower',
            output='screen'),
    ])