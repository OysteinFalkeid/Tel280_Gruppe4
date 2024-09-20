'''
#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_included_launch', default_value='true',
                              description='Whether to include the included_launch.py'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("turtlebot3_gazebo"),
                "launch/empty_world.launch.py")
                )
         ),
       Node(
            package='imrt_virtual_joy',
            executable='gamepad_talker',
            name='gamepad_talker'
        ),
       ,
       )
    ])
'''

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='asignment_1_pkg',
            executable='laser_subscriber',
            output='screen'),
    ])