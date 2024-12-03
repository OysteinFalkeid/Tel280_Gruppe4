import os

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    
    turtlebot_bringup_directory = get_package_share_directory('turtlebot3_bringup')
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'turtlebot_area.yaml')

    # Create launch descriptions
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot_bringup_directory, '/launch/robot.launch.py'])
    )
    
    return LaunchDescription([ 
        bringup_launch,
        TimerAction(
            actions = [
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': False}, 
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
                    parameters=[{'use_sim_time': False},
                                {'autostart': True},
                                {'node_names': ['map_server', 'amcl']}]
                ),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[controller_yaml]),

                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[planner_yaml]),
                    
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='recoveries_server',
                    parameters=[recovery_yaml],
                    output='screen'),

                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[bt_navigator_yaml]),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_pathplanner',
                    output='screen',
                    parameters=[{'autostart': True},
                                {'node_names': ['planner_server',
                                                'controller_server',
                                                'recoveries_server',
                                                'bt_navigator']}]
                ),
            ], period = 5.0
        )
    ])