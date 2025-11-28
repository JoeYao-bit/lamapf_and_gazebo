#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    TimerAction, IncludeLaunchDescription, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_ns = ''#'robot0'
    
    # LIDAR, remap ok
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lamapf_and_gazebo'),
                'launch',
                'rplidar_a2m8_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'serial_baudrate': '115200',
            'scan_topic': '/scan', # f'{robot_ns}/scan'   
        }.items()
    )

    # Kobuki, odom->base_footprint tf, rename ok
    kobuki_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lamapf_and_gazebo'),
                'launch',
                'kobuki_node-launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/kobuki',
            'serial_baudrate': '115200',
            'odom_frame': f'{robot_ns}/odom',
            'base_frame': f'{robot_ns}/base_footprint'
        }.items()
    )

    # static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','1','0',f'{robot_ns}/base_footprint',f'{robot_ns}/scan']
    )

    # AMCL
    amcl_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lamapf_and_gazebo'),
                'launch',
                'turtlebot2_amcl_localization.launch.py'
            )
        ),
        launch_arguments={
            #'use_sim_time': 'false',
            #'map_topic':'map',
            #'scan_topic': '/scan',                  # 指定订阅话题
            'base_frame_id': f'{robot_ns}/base_footprint',
            'odom_frame_id': f'{robot_ns}/odom',
            #'global_frame_id': 'map'
        }.items()
    )

    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    # 顺序控制：延迟 2 秒启动 AMCL，延迟 3 秒启动 RVIZ
    delayed_amcl = TimerAction(period=2.0, actions=[amcl_node])
    delayed_rviz = TimerAction(period=3.0, actions=[rviz])

    # ⭐⭐ 关键部分：把所有节点放入 GroupAction + PushRosNamespace
    ns_group = GroupAction([
        PushRosNamespace(robot_ns),

        lidar_node,
        kobuki_node,
        static_tf,
        delayed_amcl,
        delayed_rviz,
    ])

    return LaunchDescription([
        ns_group
    ])