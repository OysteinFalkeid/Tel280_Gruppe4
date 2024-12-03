import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    turtlebot_bringup_directory = get_package_share_directory('turtlebot3_bringup')
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('localization_server'), 'config', 'turtlebot_area.yaml')

    # Create launch descriptions
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot_bringup_directory, '/launch/robot.launch.py'])
        )
    
    
    return LaunchDescription([
        bringup_launch,
        TimerAction(
            actions[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': True}, 
                                {'yaml_filename':map_file}]
                ),
                    
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml]
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{'use_sim_time': True},
                                {'autostart': True},
                                {'node_names': ['map_server', 'amcl']}]
                )
            ], period = 5.0
        )
    ])