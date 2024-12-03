import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    turtlebot_bringup_directory = get_package_share_directory('turtlebot3_bringup')
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'
    
    # Create launch descriptions
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot_bringup_directory, '/launch/robot.launch.py'])
    )

    return LaunchDescription([
        bringup_launch,
        TimerAction(
            actions = [
                Node(
                    package='cartographer_ros', 
                    executable='cartographer_node', 
                    name='cartographer_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                    arguments=['-configuration_directory', cartographer_config_dir,
                            '-configuration_basename', configuration_basename]),

                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    output='screen',
                    name='occupancy_grid_node',
                    parameters=[{'use_sim_time': False}],
                    arguments=['-resolution', '0.03', '-publish_period_sec', '0.25']
                ),
            ], period = 5.0
        )
    ]) 