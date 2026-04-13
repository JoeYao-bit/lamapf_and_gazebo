#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    TimerAction, IncludeLaunchDescription, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')


    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/yaozhuo/code/ros2_ws/src/lamapf_and_gazebo/map/my_map_2602052038.yaml',
        description='Full path to map file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    

    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )


    # 地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file,
                     'use_sim_time': use_sim_time}]
    )

    # 生命周期管理器节点（自动激活 map_server）
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server']}]
    )



    return LaunchDescription([
        declare_map_cmd,
        declare_use_sim_time_cmd,
        
        lifecycle_manager_node,
        map_server_node,
        rviz
    ])
