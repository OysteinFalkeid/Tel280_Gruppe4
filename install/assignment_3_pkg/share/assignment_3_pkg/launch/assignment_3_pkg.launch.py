from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import time


def generate_launch_description():
    # Find paths to packages
    turtlebot_bringup_directory = get_package_share_directory('turtlebot3_bringup')
    cartographer_slam_directory = get_package_share_directory('cartographer_slam')
    assignment_2_directory = get_package_share_directory('assignment_2_pkg')
    
    # Create launch descriptions
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot_bringup_directory, '/launch/robot.launch.py'])
        )
    
    cartographer_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([cartographer_slam_directory, '/launch/cartographer.launch.py'])
        )
    
    assignment_2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([assignment_2_directory, '/assignment_2_launch_file.launch.py'])
        )

    return LaunchDescription([
        bringup_launch,
        TimerAction(
            actions = [
                cartographer_slam_launch
                , assignment_2_launch
                ], period = 5.0)
        ])
    
    
    
